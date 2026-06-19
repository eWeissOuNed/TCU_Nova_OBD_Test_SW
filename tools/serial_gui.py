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
from tkinter import ttk, scrolledtext
import serial
from serial.tools import list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

BAUD_DEFAULT  = 115200
IMU_WINDOW    = 200   # number of samples shown in the rolling graph

# ── Colour scheme ─────────────────────────────────────────────────────────────
BG       = "#1e1e1e"
FG       = "#d4d4d4"
INPUT_BG = "#2d2d2d"
PLOT_BG  = "#141414"
FONT     = ("Consolas", 10)

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


class SerialGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("TCU NOVA OBD – Serial Console")
        self.root.configure(bg=BG)
        self.root.geometry("1100x650")
        self.root.minsize(700, 450)

        self.port_obj: serial.Serial | None = None
        self.reader_stop = threading.Event()
        self.history: list[str] = []
        self.hist_idx = 0

        # IMU rolling buffers (thread-safe: written from main thread via after())
        self._imu_ax: collections.deque = collections.deque(maxlen=IMU_WINDOW)
        self._imu_ay: collections.deque = collections.deque(maxlen=IMU_WINDOW)
        self._imu_az: collections.deque = collections.deque(maxlen=IMU_WINDOW)
        self._imu_active = False
        self._plot_job   = None

        self._build_ui()
        self._populate_ports()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── top bar ──
        bar = tk.Frame(self.root, bg=BG, pady=4)
        bar.pack(fill=tk.X, padx=8)

        tk.Label(bar, text="Port:", bg=BG, fg=FG, font=FONT).pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(bar, textvariable=self.port_var, width=14, font=FONT)
        self.port_cb.pack(side=tk.LEFT, padx=(4, 8))

        tk.Label(bar, text="Baud:", bg=BG, fg=FG, font=FONT).pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        ttk.Combobox(bar, textvariable=self.baud_var, width=8, font=FONT,
                     values=["9600","19200","38400","57600",
                             "115200","230400","460800","921600"]
                     ).pack(side=tk.LEFT, padx=(4, 12))

        self.connect_btn = tk.Button(bar, text="Connect", font=FONT,
                                     bg="#0e639c", fg="white", relief=tk.FLAT,
                                     padx=10, command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT)
        tk.Button(bar, text="↻", font=FONT, bg=INPUT_BG, fg=FG, relief=tk.FLAT,
                  command=self._populate_ports).pack(side=tk.LEFT, padx=4)

        self.status_lbl = tk.Label(bar, text="● Disconnected",
                                   bg=BG, fg="#f44747", font=FONT)
        self.status_lbl.pack(side=tk.RIGHT, padx=8)

        tk.Label(bar, text="HB:", bg=BG, fg="#555555", font=FONT).pack(side=tk.RIGHT)
        self.hb_lbl = tk.Label(bar, text="–", bg=BG, fg="#555555",
                                font=FONT, width=14, anchor="w")
        self.hb_lbl.pack(side=tk.RIGHT, padx=(0, 6))

        # ── quick-command buttons ──
        btn_bar = tk.Frame(self.root, bg=BG)
        btn_bar.pack(fill=tk.X, padx=8, pady=(0, 4))
        for label, cmd in [
            ("PING",    "PING"),
            ("HELP",    "HELP"),
            ("STATE?",  "STATE?"),
            ("IMU?",    "IMU?"),
            ("IMU ON",  "IMU STREAM ON"),
            ("IMU OFF", "IMU STREAM OFF"),
            ("CAN ON",  "CAN STREAM ON"),
            ("CAN OFF", "CAN STREAM OFF"),
            ("LED OFF", "LED OFF"),
            ("SLEEP",   "SLEEP"),
        ]:
            tk.Button(btn_bar, text=label, font=("Consolas", 9),
                      bg=INPUT_BG, fg=FG, relief=tk.FLAT, padx=6,
                      command=lambda c=cmd: self._send(c)
                      ).pack(side=tk.LEFT, padx=2)

        # ── main content: console left, graph right (PanedWindow) ──
        pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                              bg=BG, sashwidth=5, sashrelief=tk.FLAT)
        pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 4))

        # console
        self.output = scrolledtext.ScrolledText(
            pane, bg=BG, fg=FG, font=FONT,
            insertbackground=FG, wrap=tk.WORD,
            state=tk.DISABLED, relief=tk.FLAT, width=50,
        )
        pane.add(self.output, stretch="always")
        for tag, colour in TAG_COLOURS.items():
            self.output.tag_config(tag, foreground=colour)

        # graph panel (hidden until IMU ON)
        self.graph_frame = tk.Frame(pane, bg=PLOT_BG)
        self._build_graph(self.graph_frame)
        # not added to pane yet – added on IMU ON

        # ── input row ──
        input_row = tk.Frame(self.root, bg=BG)
        input_row.pack(fill=tk.X, padx=8, pady=(0, 8))

        tk.Label(input_row, text=">", bg=BG, fg=TAG_COLOURS["cmd"],
                 font=FONT).pack(side=tk.LEFT)

        self.input_var   = tk.StringVar()
        self.input_entry = tk.Entry(input_row, textvariable=self.input_var,
                                    bg=INPUT_BG, fg=FG, font=FONT,
                                    insertbackground=FG, relief=tk.FLAT)
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6))
        self.input_entry.bind("<Return>", self._on_enter)
        self.input_entry.bind("<Up>",     self._hist_up)
        self.input_entry.bind("<Down>",   self._hist_down)
        self.input_entry.focus_set()

        tk.Button(input_row, text="Send", font=FONT,
                  bg="#0e639c", fg="white", relief=tk.FLAT, padx=10,
                  command=self._on_enter).pack(side=tk.LEFT)
        tk.Button(input_row, text="Clear", font=FONT,
                  bg=INPUT_BG, fg=FG, relief=tk.FLAT, padx=6,
                  command=self._clear_output).pack(side=tk.LEFT, padx=(4, 0))

        self._pane = pane   # keep ref for show/hide graph

    def _build_graph(self, parent):
        fig = Figure(figsize=(4, 3), dpi=90, facecolor=PLOT_BG)
        self._ax_plot = fig.add_subplot(111, facecolor=PLOT_BG)
        ax = self._ax_plot

        ax.tick_params(colors=FG, labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#444444")
        ax.set_facecolor(PLOT_BG)
        ax.set_title("IMU Acceleration", color=FG, fontsize=9)
        ax.set_ylabel("g", color=FG, fontsize=8)
        ax.set_xlabel("samples", color=FG, fontsize=8)
        ax.yaxis.label.set_color(FG)
        ax.xaxis.label.set_color(FG)

        self._line_ax, = ax.plot([], [], color="#f44747", linewidth=1.2, label="aX")
        self._line_ay, = ax.plot([], [], color="#4ec94e", linewidth=1.2, label="aY")
        self._line_az, = ax.plot([], [], color="#4fc1ff", linewidth=1.2, label="aZ")

        legend = ax.legend(fontsize=8, loc="upper right",
                           facecolor="#2d2d2d", edgecolor="#444444",
                           labelcolor=FG)

        fig.tight_layout(pad=1.2)

        self._canvas = FigureCanvasTkAgg(fig, master=parent)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._fig = fig

    # ── IMU graph show / hide / update ───────────────────────────────────────
    def _show_graph(self):
        if self.graph_frame not in self._pane.panes():
            self._pane.add(self.graph_frame, stretch="always", width=380)
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
        ax = self._ax_plot
        ax.relim()
        ax.autoscale_view()
        self._canvas.draw_idle()
        self._schedule_plot()

    # ── port helpers ─────────────────────────────────────────────────────────
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
        self.connect_btn.config(text="Disconnect", bg="#c04040")
        self.status_lbl.config(text=f"● {port} @ {baud}", fg="#4ec94e")
        self._append(f"[connected to {port} @ {baud}]\n", "info")

    def _disconnect(self):
        self.reader_stop.set()
        if self.port_obj:
            self.port_obj.close()
            self.port_obj = None
        self._hide_graph()
        self.connect_btn.config(text="Connect", bg="#0e639c")
        self.status_lbl.config(text="● Disconnected", fg="#f44747")
        self._append("[disconnected]\n", "info")

    # ── serial reader thread ──────────────────────────────────────────────────
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
        # ── heartbeat → status label only ──
        if text.startswith("HB"):
            parts = text.split()
            self.hb_lbl.config(text=parts[-1] if parts else "?", fg="#4ec94e")
            return

        # ── IMU → graph + (skip console if streaming is on) ──
        m = IMU_RE.search(text)
        if m:
            self._imu_ax.append(float(m.group(1)))
            self._imu_ay.append(float(m.group(2)))
            self._imu_az.append(float(m.group(3)))
            # only print IMU lines to console when graph is NOT showing
            # (avoids flooding the console while the graph is visible)
            if not self._imu_active:
                self._append(text + "\n", "imu")
            return

        # ── detect IMU STREAM ON/OFF responses to show/hide graph ──
        if "streaming ON" in text:
            self._show_graph()
        elif "streaming OFF" in text:
            self._hide_graph()

        # ── everything else → console ──
        if text.startswith("OK"):      tag = "ok"
        elif text.startswith("ERR"):   tag = "err"
        elif text.startswith("STD") or text.startswith("EXT"): tag = "can"
        elif text.startswith("I (") or text.startswith("W (") or \
             text.startswith("E ("):   tag = "info"
        else:                          tag = None
        self._append(text + "\n", tag)

    # ── output helpers ────────────────────────────────────────────────────────
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

    # ── input helpers ─────────────────────────────────────────────────────────
    def _on_enter(self, _event=None):
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

        # mirror IMU ON/OFF locally so graph reacts even before device responds
        if "IMU STREAM ON"  in cmd: self._show_graph()
        if "IMU STREAM OFF" in cmd: self._hide_graph()

    def _hist_up(self, _event=None):
        if self.history and self.hist_idx > 0:
            self.hist_idx -= 1
            self.input_var.set(self.history[self.hist_idx])
            self.input_entry.icursor(tk.END)

    def _hist_down(self, _event=None):
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
