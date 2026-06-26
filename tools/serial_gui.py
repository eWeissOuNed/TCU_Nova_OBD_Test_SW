#!/usr/bin/env python3
"""
TCU NOVA OBD – Serial GUI
Deps:   pip install pyserial matplotlib tkintermapview
Usage:  python serial_gui.py [PORT] [BAUD]
"""
import sys
import re
import threading
import collections
import tkinter as tk
from tkinter import ttk, scrolledtext, colorchooser, simpledialog, messagebox
import serial
from serial.tools import list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import tkintermapview
    HAS_MAP = True
except ImportError:
    HAS_MAP = False

BAUD_DEFAULT   = 115200
IMU_WINDOW     = 200
MODEM_WINDOW   = 120

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

MODEM_RE = re.compile(
    r"MS rssi=([+-]?\d+) rsrp=([+-]?\d+) sinr=([+-]?\d+) rsrq=([+-]?\d+) gps=(\S+) sats=(\d+)"
)
# Matches manual GNSS READ response: OK lat=47.28420 lon= 7.70853 alt=… sats=11 …
GNSS_READ_RE = re.compile(
    r"lat=\s*([+-]?\d+\.\d+)\s+lon=\s*([+-]?\d+\.\d+).*?sats=(\d+)"
)

IMU_RE = re.compile(
    r"IMU\s+ax=([+-]?\d+\.\d+)\s+g\s+ay=([+-]?\d+\.\d+)\s+g\s+az=([+-]?\d+\.\d+)"
    r"(?:\s+g\s+gx=([+-]?\d+\.\d+)\s+d/s\s+gy=([+-]?\d+\.\d+)\s+d/s\s+gz=([+-]?\d+\.\d+))?"
)


def styled_btn(parent, text, cmd, bg=BG3, fg=FG, font=FONT_SM, padx=6, pady=2):
    return tk.Button(parent, text=text, command=cmd, font=font,
                     bg=bg, fg=fg, relief=tk.FLAT, padx=padx, pady=pady,
                     activebackground=ACCENT2, activeforeground="white",
                     cursor="hand2")


# ── Simple input dialog ───────────────────────────────────────────────────────
def ask(title, prompt, initial=""):
    return simpledialog.askstring(title, prompt, initialvalue=initial)


# ── LED colour wheel widget ───────────────────────────────────────────────────
class LedWheel(tk.Frame):
    def __init__(self, parent, idx: int, send_fn, **kw):
        super().__init__(parent, bg=BG2, **kw)
        self.idx     = idx
        self.send_fn = send_fn
        self.colour  = "#000000"

        tk.Label(self, text=f"LED {idx}", bg=BG2, fg=FG_DIM,
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
        self.root.geometry("1100x680")
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

        self._modem_rssi: collections.deque = collections.deque(maxlen=MODEM_WINDOW)
        self._modem_rsrp: collections.deque = collections.deque(maxlen=MODEM_WINDOW)
        self._modem_sinr: collections.deque = collections.deque(maxlen=MODEM_WINDOW)
        self._modem_active = False
        self._modem_plot_job = None
        self._gps_str  = "–"
        self._gps_lat  = None
        self._gps_lon  = None
        self._gps_sats = 0
        self._map_marker = None
        self._map_path   = None
        self._gps_track: list = []   # [(lat, lon), …]

        self._build_menubar()
        self._build_topbar()
        self._build_led_panel()
        self._build_main()
        self._build_inputbar()
        self._populate_ports()

    # ── Menubar ───────────────────────────────────────────────────────────────
    def _build_menubar(self):
        mb = tk.Menu(self.root, bg=BG2, fg=FG, activebackground=ACCENT,
                     activeforeground="white", relief=tk.FLAT,
                     bd=0, tearoff=False)
        self.root.config(menu=mb)

        # ── System ──
        m = tk.Menu(mb, bg=BG2, fg=FG, activebackground=ACCENT,
                    activeforeground="white", tearoff=False)
        mb.add_cascade(label="System", menu=m)
        m.add_command(label="Ping",          command=lambda: self._send("PING"))
        m.add_command(label="Help",          command=lambda: self._send("HELP"))
        m.add_command(label="State?",        command=lambda: self._send("STATE?"))
        m.add_separator()
        m.add_command(label="→ Idle",        command=lambda: self._send("STATE SET IDLE"))
        m.add_command(label="→ Test Running",command=lambda: self._send("STATE SET TEST_RUNNING"))
        m.add_command(label="→ Sleep",       command=lambda: self._send("STATE SET SLEEP"))
        m.add_separator()
        m.add_command(label="Sleep now",     command=lambda: self._send("SLEEP"))

        # ── IMU ──
        m = tk.Menu(mb, bg=BG2, fg=FG, activebackground=ACCENT,
                    activeforeground="white", tearoff=False)
        mb.add_cascade(label="IMU", menu=m)
        m.add_command(label="Read",          command=lambda: self._send("IMU?"))
        m.add_separator()
        m.add_command(label="Stream ON",     command=lambda: self._send("IMU STREAM ON"))
        m.add_command(label="Stream OFF",    command=lambda: self._send("IMU STREAM OFF"))

        # ── CAN ──
        m = tk.Menu(mb, bg=BG2, fg=FG, activebackground=ACCENT,
                    activeforeground="white", tearoff=False)
        mb.add_cascade(label="CAN", menu=m)
        m.add_command(label="RX Stream ON",  command=lambda: self._send("CAN STREAM ON"))
        m.add_command(label="RX Stream OFF", command=lambda: self._send("CAN STREAM OFF"))
        m.add_separator()
        m.add_command(label="Send frame…",   command=self._dlg_can_send)

        # ── SD Card ──
        m = tk.Menu(mb, bg=BG2, fg=FG, activebackground=ACCENT,
                    activeforeground="white", tearoff=False)
        mb.add_cascade(label="SD Card", menu=m)
        m.add_command(label="Detect",        command=lambda: self._send("SD DET"))
        m.add_command(label="Init / Mount",  command=lambda: self._send("SD INIT"))
        m.add_command(label="List files",    command=lambda: self._send("SD LIST"))
        m.add_command(label="Read TEST.TXT", command=lambda: self._send("SD READ"))
        m.add_command(label="Write line…",   command=self._dlg_sd_write)
        m.add_command(label="Wipe TEST.TXT", command=self._dlg_sd_wipe)
        m.add_separator()
        m.add_command(label="Enable card",   command=lambda: self._send("SD EN ON"))
        m.add_command(label="Disable card",  command=lambda: self._send("SD EN OFF"))

        # ── GPIO ──
        m = tk.Menu(mb, bg=BG2, fg=FG, activebackground=ACCENT,
                    activeforeground="white", tearoff=False)
        mb.add_cascade(label="GPIO", menu=m)
        m.add_command(label="Set pin…",      command=self._dlg_gpio_set)
        m.add_command(label="Get pin…",      command=self._dlg_gpio_get)
        m.add_command(label="Toggle pin…",   command=self._dlg_gpio_toggle)
        m.add_separator()
        m.add_command(label="Stop all toggles", command=lambda: self._send("GPIO STOP ALL"))

        # ── UWB ──
        m = tk.Menu(mb, bg=BG2, fg=FG, activebackground=ACCENT,
                    activeforeground="white", tearoff=False)
        mb.add_cascade(label="UWB", menu=m)
        m.add_command(label="Init",          command=lambda: self._send("UWB INIT"))
        m.add_command(label="Read Device ID",command=lambda: self._send("UWB READ ID"))
        m.add_command(label="Range",         command=lambda: self._send("UWB RANGE"))

        # ── Modem ──
        m = tk.Menu(mb, bg=BG2, fg=FG, activebackground=ACCENT,
                    activeforeground="white", tearoff=False)
        mb.add_cascade(label="Modem", menu=m)
        m.add_command(label="Power ON",        command=lambda: self._send("MODEM POWER ON"))
        m.add_command(label="Power OFF",       command=lambda: self._send("MODEM POWER OFF"))
        m.add_separator()
        m.add_command(label="Signal strength", command=lambda: self._send("MODEM SIGNAL"))
        m.add_command(label="Stream ON (5s)",  command=lambda: self._modem_stream_on(5))
        m.add_command(label="Stream ON (10s)", command=lambda: self._modem_stream_on(10))
        m.add_command(label="Stream OFF",      command=self._modem_stream_off)
        m.add_separator()
        m.add_command(label="GNSS ON",         command=lambda: self._send("MODEM GNSS ON"))
        m.add_command(label="GNSS Read",       command=lambda: self._send("MODEM GNSS READ"))
        m.add_command(label="GNSS OFF",        command=lambda: self._send("MODEM GNSS OFF"))
        m.add_separator()
        m.add_command(label="Send SMS…",       command=self._dlg_sms)
        m.add_command(label="AT command…",     command=self._dlg_at)

        # ── View ──
        m = tk.Menu(mb, bg=BG2, fg=FG, activebackground=ACCENT,
                    activeforeground="white", tearoff=False)
        mb.add_cascade(label="View", menu=m)
        m.add_command(label="Clear console", command=self._clear_output)
        m.add_separator()
        m.add_command(label="IMU graph ON",  command=self._show_graph)
        m.add_command(label="IMU graph OFF", command=self._hide_graph)
        m.add_separator()
        m.add_command(label="GPS Map",       command=self._show_gps_tab)

    # ── Dialogs ───────────────────────────────────────────────────────────────
    def _dlg_can_send(self):
        id_hex = ask("CAN Send", "CAN ID (hex):", "001")
        if not id_hex: return
        data_hex = ask("CAN Send", "Data bytes (hex, space-separated):", "00 00")
        if data_hex is None: return
        self._send(f"CAN SEND {id_hex} {data_hex}")

    def _dlg_sd_write(self):
        msg = ask("SD Write", "Message to write:")
        if msg: self._send(f"SD WRITE {msg}")

    def _dlg_sd_wipe(self):
        if messagebox.askyesno("SD Wipe", "Delete TEST.TXT?"):
            self._send("SD WIPE")

    def _dlg_gpio_set(self):
        pin = ask("GPIO Set", "Pin number:")
        if not pin: return
        level = ask("GPIO Set", "Level (HIGH / LOW):", "HIGH")
        if level: self._send(f"GPIO SET {pin} {level.upper()}")

    def _dlg_gpio_get(self):
        pin = ask("GPIO Get", "Pin number:")
        if pin: self._send(f"GPIO GET {pin}")

    def _dlg_gpio_toggle(self):
        pin = ask("GPIO Toggle", "Pin number:")
        if not pin: return
        freq = ask("GPIO Toggle", "Frequency Hz (blank = STOP):", "1")
        if freq is None: return
        if freq.strip() == "":
            self._send(f"GPIO TOGGLE {pin} STOP")
        else:
            self._send(f"GPIO TOGGLE {pin} {freq}")

    def _dlg_sms(self):
        number = ask("Send SMS", "Phone number (e.g. +41793640605):", "+")
        if not number: return
        text = ask("Send SMS", "Message text:")
        if text: self._send(f"MODEM SMS {number} {text}")

    def _dlg_at(self):
        cmd = ask("AT Command", "AT command (without leading AT):", "+CREG?")
        if cmd: self._send(f"MODEM AT {cmd}")

    # ── Top bar (connection) ──────────────────────────────────────────────────
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

        # quick buttons
        tk.Frame(bar, bg="#444", width=1).pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=2)
        styled_btn(bar, "Ping",   lambda: self._send("PING"), padx=8
                   ).pack(side=tk.LEFT, padx=2)
        styled_btn(bar, "Help",   lambda: self._send("HELP"), padx=8
                   ).pack(side=tk.LEFT, padx=2)
        styled_btn(bar, "Clear",  self._clear_output, padx=8
                   ).pack(side=tk.LEFT, padx=2)

        # right side status
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

    # ── LED panel ─────────────────────────────────────────────────────────────
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

        # preset colours
        tk.Frame(panel, bg="#444", width=1).pack(side=tk.LEFT,
                                                  fill=tk.Y, padx=8, pady=4)
        tk.Label(panel, text="Presets:", bg=BG2, fg=FG_DIM,
                 font=FONT_SM).pack(side=tk.LEFT, padx=(0, 4))
        for label, r, g, b in [("White",255,255,255),("Red",255,0,0),
                                 ("Green",0,255,0),("Blue",0,0,255)]:
            styled_btn(panel, label,
                       lambda r=r,g=g,b=b: [
                           self._send(f"LED {i} {r} {g} {b}") for i in range(3)
                       ]).pack(side=tk.LEFT, padx=2)

    # ── Main pane ─────────────────────────────────────────────────────────────
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

        # Right panel: notebook with IMU and Modem tabs
        self._nb_frame = tk.Frame(self._pane, bg=BG2)
        self._notebook = ttk.Notebook(self._nb_frame)
        self._notebook.pack(fill=tk.BOTH, expand=True)

        self.graph_frame = tk.Frame(self._notebook, bg=PLOT_BG)
        self._notebook.add(self.graph_frame, text=" IMU ")
        self._build_imu_graph(self.graph_frame)

        self._modem_frame = tk.Frame(self._notebook, bg=PLOT_BG)
        self._notebook.add(self._modem_frame, text=" Modem ")
        self._build_modem_graph(self._modem_frame)

        self._gps_frame = tk.Frame(self._notebook, bg=BG2)
        self._notebook.add(self._gps_frame, text=" GPS Map ")
        self._build_gps_tab(self._gps_frame)

    def _build_imu_graph(self, parent):
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

    def _build_modem_graph(self, parent):
        fig = Figure(figsize=(4, 3), dpi=90, facecolor=PLOT_BG)
        self._modem_ax = fig.add_subplot(111, facecolor=PLOT_BG)
        ax = self._modem_ax
        ax.tick_params(colors=FG, labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#444")
        ax.set_title("Signal Strength", color=FG, fontsize=9)
        ax.set_ylabel("dBm", color=FG, fontsize=8)
        ax.set_xlabel("samples", color=FG, fontsize=8)
        self._line_rssi, = ax.plot([], [], color="#4fc1ff", lw=1.2, label="RSSI")
        self._line_rsrp, = ax.plot([], [], color="#ce9178", lw=1.2, label="RSRP")
        self._line_sinr, = ax.plot([], [], color="#4ec94e", lw=1.2, label="SINR (dB)")
        ax.legend(fontsize=8, loc="upper right",
                  facecolor=BG3, edgecolor="#444", labelcolor=FG)
        fig.tight_layout(pad=1.2)
        self._modem_canvas = FigureCanvasTkAgg(fig, master=parent)
        self._modem_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # GPS label below graph
        self._gps_label = tk.Label(parent, text="GPS: –", bg=BG2, fg=FG_DIM,
                                    font=FONT_SM, anchor="w")
        self._gps_label.pack(fill=tk.X, padx=6, pady=(0, 4))

    def _build_gps_tab(self, parent):
        # ── Info bar ──────────────────────────────────────────────────────────
        info = tk.Frame(parent, bg=BG2, pady=4)
        info.pack(fill=tk.X, padx=6)

        tk.Label(info, text="Satellites:", bg=BG2, fg=FG_DIM,
                 font=FONT_SM).pack(side=tk.LEFT)
        self._sats_lbl = tk.Label(info, text="–", bg=BG3, fg=FG_DIM,
                                   font=FONT_BOLD, width=4, padx=6,
                                   relief=tk.FLAT)
        self._sats_lbl.pack(side=tk.LEFT, padx=(2, 14))

        tk.Label(info, text="Position:", bg=BG2, fg=FG_DIM,
                 font=FONT_SM).pack(side=tk.LEFT)
        self._coords_lbl = tk.Label(info, text="–", bg=BG2, fg=FG,
                                     font=FONT_SM)
        self._coords_lbl.pack(side=tk.LEFT, padx=(4, 0))

        styled_btn(info, "Clear track", self._clear_track,
                   padx=8).pack(side=tk.RIGHT, padx=4)

        # ── Map area ──────────────────────────────────────────────────────────
        if HAS_MAP:
            self._map_widget = tkintermapview.TkinterMapView(
                parent, width=420, height=320, corner_radius=0)
            self._map_widget.pack(fill=tk.BOTH, expand=True)
            self._map_widget.set_tile_server(
                "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")
            self._map_widget.set_position(47.0, 8.0)  # Switzerland default
            self._map_widget.set_zoom(7)
        else:
            tk.Label(parent,
                     text="Map not available.\nInstall: pip install tkintermapview",
                     bg=BG2, fg=FG_DIM, font=FONT, justify=tk.CENTER
                     ).pack(expand=True)

    def _update_gps_display(self, lat: float, lon: float, sats: int):
        self._gps_lat  = lat
        self._gps_lon  = lon
        self._gps_sats = sats

        # Colour the sat count: green ≥4, yellow 1-3, red 0
        if sats >= 4:
            sat_fg = GREEN
        elif sats > 0:
            sat_fg = "#dcdcaa"
        else:
            sat_fg = RED
        self._sats_lbl.config(text=str(sats), fg=sat_fg)
        self._coords_lbl.config(
            text=f"lat={lat:.5f}   lon={lon:.5f}")

        if HAS_MAP and hasattr(self, "_map_widget"):
            # Accumulate track points (skip duplicates)
            if not self._gps_track or self._gps_track[-1] != (lat, lon):
                self._gps_track.append((lat, lon))

            # Update current-position marker
            if self._map_marker:
                self._map_marker.delete()
            self._map_marker = self._map_widget.set_marker(
                lat, lon, text=f"{sats} sats")

            # Draw/redraw track path (needs ≥2 points)
            if len(self._gps_track) >= 2:
                if self._map_path:
                    self._map_path.delete()
                self._map_path = self._map_widget.set_path(
                    self._gps_track,
                    color="#4fc1ff", width=3)

            self._map_widget.set_position(lat, lon)

    def _clear_track(self):
        self._gps_track.clear()
        if HAS_MAP and hasattr(self, "_map_widget"):
            if self._map_path:
                self._map_path.delete()
                self._map_path = None

    # ── Input bar ─────────────────────────────────────────────────────────────
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

    # ── IMU graph ─────────────────────────────────────────────────────────────
    def _show_graph(self):
        if self._nb_frame not in self._pane.panes():
            self._pane.add(self._nb_frame, stretch="always", width=420)
        self._notebook.select(self.graph_frame)
        self._imu_active = True
        self._schedule_plot()

    def _hide_graph(self):
        self._imu_active = False
        if self._plot_job:
            self.root.after_cancel(self._plot_job)
            self._plot_job = None
        if not self._modem_active and self._nb_frame in self._pane.panes():
            self._pane.remove(self._nb_frame)

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

    def _show_gps_tab(self):
        if self._nb_frame not in self._pane.panes():
            self._pane.add(self._nb_frame, stretch="always", width=420)
        self._notebook.select(self._gps_frame)

    # ── Modem graph ───────────────────────────────────────────────────────────
    def _modem_stream_on(self, interval_s: int):
        self._send(f"MODEM STREAM ON {interval_s}")
        if self._nb_frame not in self._pane.panes():
            self._pane.add(self._nb_frame, stretch="always", width=420)
        self._notebook.select(self._modem_frame)
        self._modem_active = True
        self._schedule_modem_plot()

    def _modem_stream_off(self):
        self._send("MODEM STREAM OFF")
        self._modem_active = False
        if self._modem_plot_job:
            self.root.after_cancel(self._modem_plot_job)
            self._modem_plot_job = None
        if not self._imu_active and self._nb_frame in self._pane.panes():
            self._pane.remove(self._nb_frame)

    def _schedule_modem_plot(self):
        self._modem_plot_job = self.root.after(500, self._update_modem_plot)

    def _update_modem_plot(self):
        if not self._modem_active:
            return
        xs = list(range(len(self._modem_rssi)))
        self._line_rssi.set_data(xs, list(self._modem_rssi))
        self._line_rsrp.set_data(xs, list(self._modem_rsrp))
        # SINR is in 0.2 dB units from modem; scale to dB for display
        sinr_db = [v * 0.2 for v in self._modem_sinr]
        self._line_sinr.set_data(xs, sinr_db)
        self._modem_ax.relim()
        self._modem_ax.autoscale_view()
        self._modem_canvas.draw_idle()
        self._gps_label.config(text=f"GPS: {self._gps_str}")
        self._schedule_modem_plot()

    # ── Serial ────────────────────────────────────────────────────────────────
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
        self._modem_stream_off()
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
        if text.startswith("HB"):
            parts = text.split()
            self.hb_lbl.config(text=parts[-1] if parts else "?", fg=GREEN)
            return

        m = IMU_RE.search(text)
        if m:
            self._imu_ax.append(float(m.group(1)))
            self._imu_ay.append(float(m.group(2)))
            self._imu_az.append(float(m.group(3)))
            if not self._imu_active:
                self._append(text + "\n", "imu")
            return

        m = MODEM_RE.search(text)
        if m:
            self._modem_rssi.append(int(m.group(1)))
            self._modem_rsrp.append(int(m.group(2)))
            self._modem_sinr.append(int(m.group(3)))
            self._gps_str = m.group(5).strip()
            sats = int(m.group(6))
            if self._gps_str != "no_fix":
                try:
                    lat, lon = map(float, self._gps_str.split(","))
                    self._update_gps_display(lat, lon, sats)
                except (ValueError, AttributeError):
                    pass
            else:
                self._sats_lbl.config(text="0", fg=RED)
                self._coords_lbl.config(text="no fix")
            return  # don't print MS lines to console

        m = GNSS_READ_RE.search(text)
        if m:
            try:
                lat  = float(m.group(1))
                lon  = float(m.group(2))
                sats = int(m.group(3))
                self._update_gps_display(lat, lon, sats)
                self._show_gps_tab()
            except (ValueError, AttributeError):
                pass

        if text.startswith("OK") and any(s in text for s in
                                          ("IDLE","SLEEP","TEST_RUNNING")):
            self.state_lbl.config(text=text.replace("OK", "").strip())

        if "streaming ON"  in text: self._show_graph()
        if "streaming OFF" in text: self._hide_graph()

        if text.startswith("OK"):      tag = "ok"
        elif text.startswith("ERR"):   tag = "err"
        elif text.startswith("STD") or text.startswith("EXT"): tag = "can"
        elif text.startswith("I (") or text.startswith("W (") or \
             text.startswith("E ("):   tag = "info"
        else:                          tag = None
        self._append(text + "\n", tag)

    # ── Output ────────────────────────────────────────────────────────────────
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

    # ── Input ─────────────────────────────────────────────────────────────────
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


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = SerialGUI(root)
    if len(sys.argv) > 1: app.port_var.set(sys.argv[1])
    if len(sys.argv) > 2: app.baud_var.set(sys.argv[2])
    root.protocol("WM_DELETE_WINDOW", lambda: (app._disconnect(), root.destroy()))
    root.mainloop()

if __name__ == "__main__":
    main()
