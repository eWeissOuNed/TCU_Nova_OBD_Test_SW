#!/usr/bin/env python3
"""
TCU NOVA OBD – Serial GUI
Deps:   pip install pyserial matplotlib
Usage:  python serial_gui.py [PORT] [BAUD]
"""
import sys
import re
import threading
import collections
import tkinter as tk
from tkinter import ttk, scrolledtext, colorchooser
import serial
from serial.tools import list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

BAUD_DEFAULT = 115200
IMU_WINDOW   = 200

# ── Colours ───────────────────────────────────────────────────────────────────
BG        = "#1e1e1e"
BG2       = "#252526"
BG3       = "#2d2d2d"
FG        = "#d4d4d4"
FG_DIM    = "#888888"
ACCENT    = "#0e639c"
ACCENT2   = "#1177bb"
RED       = "#c04040"
GREEN     = "#4ec94e"
PLOT_BG   = "#141414"
FONT      = ("Consolas", 10)
FONT_SM   = ("Consolas", 9)
FONT_BOLD = ("Consolas", 10, "bold")

TAG_COLOURS = {
    "ok":   "#4ec94e",
    "err":  "#f44747",
    "imu":  "#4fc1ff",
    "can":  "#ce9178",
    "info": "#888888",
    "cmd":  "#dcdcaa",
}

IMU_RE = re.compile(
    r"IMU\s+ax=([+-]?\d+\.\d+)\s+g\s+ay=([+-]?\d+\.\d+)\s+g\s+az=([+-]?\d+\.\d+)"
)


def styled_btn(parent, text, cmd, bg=BG3, fg=FG, font=FONT_SM, padx=6, pady=2):
    return tk.Button(parent, text=text, command=cmd, font=font,
                     bg=bg, fg=fg, relief=tk.FLAT, padx=padx, pady=pady,
                     activebackground=ACCENT2, activeforeground="white",
                     cursor="hand2")


def section_label(parent, text):
    tk.Label(parent, text=text, bg=BG2, fg=FG_DIM,
             font=FONT_SM).pack(side=tk.LEFT, padx=(8, 2))


# ── LED colour wheel widget ───────────────────────────────────────────────────
class LedWheel(tk.Frame):
    def __init__(self, parent, idx: int, send_fn, **kw):
        super().__init__(parent, bg=BG2, **kw)
        self.idx     = idx
        self.send_fn = send_fn
        self.colour  = "#000000"

        tk.Label(self, text=f"{idx}", bg=BG2, fg=FG_DIM,
                 font=FONT_SM).pack()

        self.canvas = tk.Canvas(self, width=36, height=36, bg=BG2,
                                highlightthickness=0, cursor="hand2")
        self.canvas.pack()
        self._draw(self.colour)
        self.canvas.bind("<Button-1>", self._pick)

        styled_btn(self, "off", self._off, padx=4, pady=0).pack(pady=(2, 0))

    def _draw(self, colour):
        self.canvas.delete("all")
        self.canvas.create_oval(3, 3, 33, 33, fill=colour,
                                outline="#555", width=1)

    def _pick(self, _=None):
        result = colorchooser.askcolor(color=self.colour,
                                       title=f"LED {self.idx}")
        if result and result[0]:
            r, g, b = (int(v) for v in result[0])
            self.colour = f"#{r:02x}{g:02x}{b:02x}"
            self._draw(self.colour)
            self.send_fn(f"LED {self.idx} {r} {g} {b}")

    def _off(self):
        self.colour = "#000000"
        self._draw(self.colour)
        self.send_fn(f"LED {self.idx} 0 0 0")


# ── Main GUI ──────────────────────────────────────────────────────────────────
class SerialGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("TCU NOVA OBD – Serial Console")
        self.root.configure(bg=BG)
        self.root.geometry("1200x720")
        self.root.minsize(800, 500)

        self.port_obj: serial.Serial | None = None
        self.reader_stop = threading.Event()
        self.history: list[str] = []
        self.hist_idx = 0

        self._imu_ax: collections.deque = collections.deque(maxlen=IMU_WINDOW)
        self._imu_ay: collections.deque = collections.deque(maxlen=IMU_WINDOW)
        self._imu_az: collections.deque = collections.deque(maxlen=IMU_WINDOW)
        self._imu_active = False
        self._plot_job   = None

        self._build_ui()
        self._populate_ports()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self._build_topbar()
        self._build_toolbar()
        self._build_led_panel()
        self._build_main()
        self._build_inputbar()

    def _build_topbar(self):
        bar = tk.Frame(self.root, bg=BG, pady=5)
        bar.pack(fill=tk.X, padx=8)

        tk.Label(bar, text="Port:", bg=BG, fg=FG, font=FONT).pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(bar, textvariable=self.port_var,
                                      width=12, font=FONT)
        self.port_cb.pack(side=tk.LEFT, padx=(4, 8))

        tk.Label(bar, text="Baud:", bg=BG, fg=FG, font=FONT).pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        ttk.Combobox(bar, textvariable=self.baud_var, width=8, font=FONT,
                     values=["9600","19200","38400","57600",
                             "115200","230400","460800","921600"]
                     ).pack(side=tk.LEFT, padx=(4, 8))

        self.connect_btn = styled_btn(bar, "Connect", self._toggle_connect,
                                      bg=ACCENT, fg="white", padx=12, pady=3)
        self.connect_btn.pack(side=tk.LEFT)
        styled_btn(bar, "↻", self._populate_ports, padx=6
                   ).pack(side=tk.LEFT, padx=4)

        # right side
        self.status_lbl = tk.Label(bar, text="● Disconnected",
                                   bg=BG, fg="#f44747", font=FONT)
        self.status_lbl.pack(side=tk.RIGHT, padx=8)

        tk.Label(bar, text="HB:", bg=BG, fg=FG_DIM, font=FONT
                 ).pack(side=tk.RIGHT)
        self.hb_lbl = tk.Label(bar, text="–", bg=BG, fg=FG_DIM,
                                font=FONT, width=12, anchor="w")
        self.hb_lbl.pack(side=tk.RIGHT, padx=(0, 4))

        tk.Label(bar, text="State:", bg=BG, fg=FG_DIM, font=FONT
                 ).pack(side=tk.RIGHT)
        self.state_lbl = tk.Label(bar, text="–", bg=BG, fg="#dcdcaa",
                                   font=FONT_BOLD, width=14, anchor="w")
        self.state_lbl.pack(side=tk.RIGHT, padx=(0, 4))

    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg=BG2, pady=4)
        bar.pack(fill=tk.X, padx=0)

        # ── System ──
        section_label(bar, "SYSTEM")
        for lbl, cmd in [("Ping", "PING"), ("Help", "HELP"),
                          ("State?", "STATE?")]:
            styled_btn(bar, lbl, lambda c=cmd: self._send(c)
                       ).pack(side=tk.LEFT, padx=2)

        for lbl, state in [("→ Idle", "IDLE"), ("→ Sleep", "SLEEP"),
                             ("→ Test", "TEST_RUNNING")]:
            styled_btn(bar, lbl,
                       lambda s=state: self._send(f"STATE SET {s}"),
                       bg="#3a3a3a").pack(side=tk.LEFT, padx=2)

        tk.Frame(bar, bg="#444", width=1).pack(side=tk.LEFT,
                                               fill=tk.Y, padx=6, pady=2)

        # ── IMU ──
        section_label(bar, "IMU")
        for lbl, cmd in [("Read", "IMU?"), ("Stream ON", "IMU STREAM ON"),
                          ("Stream OFF", "IMU STREAM OFF")]:
            styled_btn(bar, lbl, lambda c=cmd: self._send(c)
                       ).pack(side=tk.LEFT, padx=2)

        tk.Frame(bar, bg="#444", width=1).pack(side=tk.LEFT,
                                               fill=tk.Y, padx=6, pady=2)

        # ── CAN ──
        section_label(bar, "CAN")
        for lbl, cmd in [("RX ON", "CAN STREAM ON"),
                          ("RX OFF", "CAN STREAM OFF")]:
            styled_btn(bar, lbl, lambda c=cmd: self._send(c)
                       ).pack(side=tk.LEFT, padx=2)

        tk.Frame(bar, bg="#444", width=1).pack(side=tk.LEFT,
                                               fill=tk.Y, padx=6, pady=2)

        # ── SD ──
        section_label(bar, "SD CARD")
        for lbl, cmd in [("Init",   "SD INIT"),
                          ("List",   "SD LIST"),
                          ("Read",   "SD READ"),
                          ("Wipe",   "SD WIPE"),
                          ("EN ON",  "SD EN ON"),
                          ("EN OFF", "SD EN OFF")]:
            styled_btn(bar, lbl, lambda c=cmd: self._send(c)
                       ).pack(side=tk.LEFT, padx=2)

        tk.Frame(bar, bg="#444", width=1).pack(side=tk.LEFT,
                                               fill=tk.Y, padx=6, pady=2)

        # ── LED quick ──
        section_label(bar, "LED")
        styled_btn(bar, "All OFF", lambda: self._send("LED OFF"),
                   bg="#5a2a2a", fg="#ffaaaa").pack(side=tk.LEFT, padx=2)

        for i, (r, g, b) in enumerate([(255,255,255),(255,0,0),(0,255,0),(0,0,255)]):
            label = ["White","Red","Green","Blue"][i]
            styled_btn(bar, label,
                       lambda c=f"LED OFF\nLED 0 {r} {g} {b}\nLED 1 {r} {g} {b}\nLED 2 {r} {g} {b}":
                           [self._send(l) for l in c.split("\n")]
                       ).pack(side=tk.LEFT, padx=2)

    def _build_led_panel(self):
        panel = tk.Frame(self.root, bg=BG2, pady=4)
        panel.pack(fill=tk.X, padx=0, pady=(1, 0))

        tk.Label(panel, text=" LEDs", bg=BG2, fg=FG_DIM,
                 font=FONT_BOLD).pack(side=tk.LEFT, padx=(8, 8))

        self.led_wheels = []
        for i in range(3):
            w = LedWheel(panel, i, self._send)
            w.pack(side=tk.LEFT, padx=6)
            self.led_wheels.append(w)

        tk.Frame(panel, bg="#444", width=1).pack(side=tk.LEFT,
                                                  fill=tk.Y, padx=8, pady=4)
        styled_btn(panel, "All OFF", lambda: self._send("LED OFF"),
                   bg="#5a2a2a", fg="#ffaaaa").pack(side=tk.LEFT, padx=4)

    def _build_main(self):
        self._pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                                    bg=BG, sashwidth=5, sashrelief=tk.FLAT)
        self._pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        self.output = scrolledtext.ScrolledText(
            self._pane, bg=BG, fg=FG, font=FONT,
            insertbackground=FG, wrap=tk.WORD,
            state=tk.DISABLED, relief=tk.FLAT, width=55,
        )
        self._pane.add(self.output, stretch="always")
        for tag, colour in TAG_COLOURS.items():
            self.output.tag_config(tag, foreground=colour)

        self.graph_frame = tk.Frame(self._pane, bg=PLOT_BG)
        self._build_graph(self.graph_frame)

    def _build_graph(self, parent):
        fig = Figure(figsize=(4, 3), dpi=90, facecolor=PLOT_BG)
        self._ax_plot = fig.add_subplot(111, facecolor=PLOT_BG)
        ax = self._ax_plot
        ax.tick_params(colors=FG, labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#444")
        ax.set_title("IMU Acceleration", color=FG, fontsize=9)
        ax.set_ylabel("g", color=FG, fontsize=8)
        ax.set_xlabel("samples", color=FG, fontsize=8)
        self._line_ax, = ax.plot([], [], color="#f44747", lw=1.2, label="aX")
        self._line_ay, = ax.plot([], [], color="#4ec94e", lw=1.2, label="aY")
        self._line_az, = ax.plot([], [], color="#4fc1ff", lw=1.2, label="aZ")
        ax.legend(fontsize=8, loc="upper right",
                  facecolor=BG3, edgecolor="#444", labelcolor=FG)
        fig.tight_layout(pad=1.2)
        self._canvas = FigureCanvasTkAgg(fig, master=parent)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_inputbar(self):
        bar = tk.Frame(self.root, bg=BG, pady=6)
        bar.pack(fill=tk.X, padx=8)

        tk.Label(bar, text=">", bg=BG, fg=TAG_COLOURS["cmd"],
                 font=FONT).pack(side=tk.LEFT)

        self.input_var   = tk.StringVar()
        self.input_entry = tk.Entry(bar, textvariable=self.input_var,
                                    bg=BG3, fg=FG, font=FONT,
                                    insertbackground=FG, relief=tk.FLAT)
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6))
        self.input_entry.bind("<Return>", self._on_enter)
        self.input_entry.bind("<Up>",     self._hist_up)
        self.input_entry.bind("<Down>",   self._hist_down)
        self.input_entry.focus_set()

        styled_btn(bar, "Send", self._on_enter, bg=ACCENT, fg="white",
                   padx=12, pady=3).pack(side=tk.LEFT)
        styled_btn(bar, "Clear", self._clear_output,
                   padx=8).pack(side=tk.LEFT, padx=(4, 0))

    # ── IMU graph ─────────────────────────────────────────────────────────────
    def _show_graph(self):
        if self.graph_frame not in self._pane.panes():
            self._pane.add(self.graph_frame, stretch="always", width=400)
        self._imu_active = True
        self._schedule_plot()

    def _hide_graph(self):
        self._imu_active = False
        if self._plot_job:
            self.root.after_cancel(self._plot_job)
            self._plot_job = None
        if self.graph_frame in self._pane.panes():
            self._pane.remove(self.graph_frame)

    def _schedule_plot(self):
        self._plot_job = self.root.after(100, self._update_plot)

    def _update_plot(self):
        if not self._imu_active:
            return
        xs = list(range(len(self._imu_ax)))
        self._line_ax.set_data(xs, list(self._imu_ax))
        self._line_ay.set_data(xs, list(self._imu_ay))
        self._line_az.set_data(xs, list(self._imu_az))
        self._ax_plot.relim()
        self._ax_plot.autoscale_view()
        self._canvas.draw_idle()
        self._schedule_plot()

    # ── serial ────────────────────────────────────────────────────────────────
    def _populate_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.port_obj and self.port_obj.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        baud = int(self.baud_var.get())
        try:
            self.port_obj = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self._append(f"[error] {e}\n", "err")
            return
        self.reader_stop.clear()
        threading.Thread(target=self._reader, daemon=True).start()
        self.connect_btn.config(text="Disconnect", bg=RED)
        self.status_lbl.config(text=f"● {port} @ {baud}", fg=GREEN)
        self._append(f"[connected to {port} @ {baud}]\n", "info")

    def _disconnect(self):
        self.reader_stop.set()
        if self.port_obj:
            self.port_obj.close()
            self.port_obj = None
        self._hide_graph()
        self.connect_btn.config(text="Connect", bg=ACCENT)
        self.status_lbl.config(text="● Disconnected", fg="#f44747")
        self.state_lbl.config(text="–")
        self._append("[disconnected]\n", "info")

    def _reader(self):
        buf = b""
        while not self.reader_stop.is_set():
            try:
                chunk = self.port_obj.read(self.port_obj.in_waiting or 1)
            except serial.SerialException:
                self.root.after(0, self._disconnect)
                break
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").strip()
                    if text and text != ">":
                        self.root.after(0, self._append_line, text)

    def _append_line(self, text: str):
        # heartbeat → status label
        if text.startswith("HB"):
            parts = text.split()
            self.hb_lbl.config(text=parts[-1] if parts else "?", fg=GREEN)
            return

        # IMU → graph buffers
        m = IMU_RE.search(text)
        if m:
            self._imu_ax.append(float(m.group(1)))
            self._imu_ay.append(float(m.group(2)))
            self._imu_az.append(float(m.group(3)))
            if not self._imu_active:
                self._append(text + "\n", "imu")
            return

        # state change → state label
        if text.startswith("OK") and any(s in text for s in
                                          ("IDLE","SLEEP","TEST_RUNNING")):
            state = text.replace("OK", "").strip()
            self.state_lbl.config(text=state)

        # IMU stream toggle → graph show/hide
        if "streaming ON"  in text: self._show_graph()
        if "streaming OFF" in text: self._hide_graph()

        # colour-coded console output
        if text.startswith("OK"):      tag = "ok"
        elif text.startswith("ERR"):   tag = "err"
        elif text.startswith("STD") or text.startswith("EXT"): tag = "can"
        elif text.startswith("I (") or text.startswith("W (") or \
             text.startswith("E ("):   tag = "info"
        else:                          tag = None
        self._append(text + "\n", tag)

    # ── output ────────────────────────────────────────────────────────────────
    def _append(self, text: str, tag: str | None = None):
        self.output.config(state=tk.NORMAL)
        if tag:
            self.output.insert(tk.END, text, tag)
        else:
            self.output.insert(tk.END, text)
        self.output.see(tk.END)
        self.output.config(state=tk.DISABLED)

    def _clear_output(self):
        self.output.config(state=tk.NORMAL)
        self.output.delete("1.0", tk.END)
        self.output.config(state=tk.DISABLED)

    # ── input ─────────────────────────────────────────────────────────────────
    def _on_enter(self, _=None):
        cmd = self.input_var.get().strip()
        if not cmd:
            return
        self._send(cmd)
        self.history.append(cmd)
        self.hist_idx = len(self.history)
        self.input_var.set("")

    def _send(self, cmd: str):
        self._append(f"> {cmd}\n", "cmd")
        if self.port_obj and self.port_obj.is_open:
            try:
                self.port_obj.write((cmd + "\n").encode())
            except serial.SerialException as e:
                self._append(f"[send error] {e}\n", "err")
        else:
            self._append("[not connected]\n", "err")

        if "IMU STREAM ON"  in cmd: self._show_graph()
        if "IMU STREAM OFF" in cmd: self._hide_graph()

    def _hist_up(self, _=None):
        if self.history and self.hist_idx > 0:
            self.hist_idx -= 1
            self.input_var.set(self.history[self.hist_idx])
            self.input_entry.icursor(tk.END)

    def _hist_down(self, _=None):
        if self.hist_idx < len(self.history) - 1:
            self.hist_idx += 1
            self.input_var.set(self.history[self.hist_idx])
        else:
            self.hist_idx = len(self.history)
            self.input_var.set("")


# ── entry point ───────────────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = SerialGUI(root)
    if len(sys.argv) > 1: app.port_var.set(sys.argv[1])
    if len(sys.argv) > 2: app.baud_var.set(sys.argv[2])
    root.protocol("WM_DELETE_WINDOW", lambda: (app._disconnect(), root.destroy()))
    root.mainloop()

if __name__ == "__main__":
    main()
