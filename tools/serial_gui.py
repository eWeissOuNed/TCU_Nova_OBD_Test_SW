#!/usr/bin/env python3
"""
TCU NOVA OBD – Serial GUI
Deps:   pip install pyserial matplotlib numpy
        STL files:  pip install numpy-stl
        STEP files: pip install cadquery          (or export STEP as STL first)
Usage:  python serial_gui.py [PORT] [BAUD]
"""
import sys
import re
import threading
import collections
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, colorchooser, filedialog
import serial
from serial.tools import list_ports
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

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
    r"\s+g\s+gx=([+-]?\d+\.\d+)\s+d/s\s+gy=([+-]?\d+\.\d+)\s+d/s\s+gz=([+-]?\d+\.\d+)"
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


# ── 3-D Orientation Viewer ────────────────────────────────────────────────────
class OrientationViewer(tk.Toplevel):
    """
    Standalone Toplevel window showing a 3-D mesh that rotates in real-time
    based on accelerometer data forwarded via update_imu().

    Mesh loading:
      • STEP / STP  →  requires `cadquery`  (pip install cadquery)
      • STL         →  requires `numpy-stl` (pip install numpy-stl)

    If cadquery is not installed the user can export their STEP as STL from
    SolidWorks / FreeCAD / Fusion 360 (File → Export → STL) and open that.

    Orientation via complementary filter:
      Gyro  integrates angular rate  → fast, responsive, drifts over time
      Accel gives absolute roll/pitch → noisy during motion, stable long-term
      Blend: R = 0.98 × R_gyro  +  0.02 × R_accel  (re-orthogonalised each frame)
      Yaw:  observable from gyro (short-term); drifts without magnetometer
    """

    _UPDATE_MS  = 130    # redraw interval in milliseconds (~7.5 fps)
    _CF_ALPHA   = 0.98   # complementary filter weight for gyro (0=accel only, 1=gyro only)

    def __init__(self, parent):
        super().__init__(parent)
        self.title("3D Orientation – TCU NOVA OBD")
        self.geometry("560x600")
        self.configure(bg=BG)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.resizable(True, True)

        # mesh data (N,3,3) float32 – normalised triangles
        self._triangles: np.ndarray | None = None

        # rotation state
        self._rot        = np.eye(3, dtype=np.float64)
        self._ax_val     = 0.0
        self._ay_val     = 0.0
        self._az_val     = 1.0      # board flat on table → gravity in +Z
        self._gx_dps     = 0.0
        self._gy_dps     = 0.0
        self._gz_dps     = 0.0
        self._last_t     = time.monotonic()

        self._running    = True
        self._poly3d     = None     # current Poly3DCollection in axis

        self._build_ui()
        self._schedule_update()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        # top bar
        bar = tk.Frame(self, bg=BG2, pady=5)
        bar.pack(fill=tk.X)
        styled_btn(bar, "Load STEP / STL", self._load_mesh,
                   bg=ACCENT, fg="white", padx=10
                   ).pack(side=tk.LEFT, padx=8)
        self.info_lbl = tk.Label(bar,
                                  text="No model loaded  –  STEP or STL supported",
                                  bg=BG2, fg=FG_DIM, font=FONT_SM)
        self.info_lbl.pack(side=tk.LEFT, padx=6)

        # angle readout bar
        ang = tk.Frame(self, bg=BG, pady=3)
        ang.pack(fill=tk.X, padx=10)
        for lbl_text, attr in [("Roll:", "roll_lbl"),
                                ("Pitch:", "pitch_lbl"),
                                ("Yaw:", "yaw_lbl")]:
            tk.Label(ang, text=lbl_text, bg=BG, fg=FG_DIM,
                     font=FONT_SM).pack(side=tk.LEFT)
            lbl = tk.Label(ang, text="  +0.0°", bg=BG, fg="#4fc1ff",
                           font=FONT_SM, width=8, anchor="w")
            lbl.pack(side=tk.LEFT, padx=(2, 10))
            setattr(self, attr, lbl)
        tk.Label(ang, text="(yaw drifts – no magnetometer)", bg=BG, fg=FG_DIM,
                 font=FONT_SM).pack(side=tk.LEFT)

        # matplotlib 3-D figure
        fig = Figure(figsize=(5.5, 4.8), dpi=90, facecolor=PLOT_BG)
        self._ax3d = fig.add_subplot(111, projection="3d")
        self._ax3d.set_facecolor(PLOT_BG)
        self._init_axes()
        fig.tight_layout(pad=0.4)

        self._canvas3d = FigureCanvasTkAgg(fig, master=self)
        self._canvas3d.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._fig3d = fig

    def _init_axes(self):
        ax = self._ax3d
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.tick_params(colors=FG_DIM, labelsize=7)
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor("#3a3a3a")
        ax.grid(True, color="#222", linewidth=0.4)
        ax.set_xlabel("X", color=FG_DIM, fontsize=7)
        ax.set_ylabel("Y", color=FG_DIM, fontsize=7)
        ax.set_zlabel("Z", color=FG_DIM, fontsize=7)

    # ── mesh loading ──────────────────────────────────────────────────────────
    def _load_mesh(self):
        path = filedialog.askopenfilename(
            title="Open STEP or STL model",
            filetypes=[("CAD files",  "*.step *.stp *.stl"),
                       ("STEP",       "*.step *.stp"),
                       ("STL",        "*.stl"),
                       ("All files",  "*.*")],
        )
        if not path:
            return

        self.info_lbl.config(text="Loading…", fg=FG_DIM)
        self.update_idletasks()

        ext = path.lower().rsplit(".", 1)[-1]
        try:
            if ext in ("step", "stp"):
                tris = self._load_step(path)
            else:
                tris = self._load_stl(path)
        except (ImportError, Exception) as e:
            msg = str(e)
            print(f"[3D viewer] load error: {msg}")   # full detail in terminal
            self.info_lbl.config(text=msg[:80], fg="#f44747")
            return

        # centre and normalise to ±1
        flat = tris.reshape(-1, 3)
        tris -= flat.mean(axis=0)
        scale = np.abs(tris).max()
        if scale > 1e-9:
            tris /= scale

        # decimate if mesh is too dense for smooth matplotlib rendering
        n_orig = len(tris)
        if n_orig > 4000:
            step = max(1, n_orig // 4000)
            tris = tris[::step]

        self._triangles = tris.astype(np.float32)
        name = path.replace("\\", "/").split("/")[-1]
        shown = len(tris)
        self.info_lbl.config(
            fg=FG_DIM,
            text=f"{name}  –  {n_orig} faces" +
                 (f" → {shown} shown (decimated)" if shown < n_orig else "")
        )

    @staticmethod
    def _load_step(path: str) -> np.ndarray:
        """Load STEP via cadquery → (N,3,3) float32."""
        try:
            import cadquery as cq
        except ImportError as e:
            raise ImportError(f"cadquery import failed: {e}")
        except Exception as e:
            raise ImportError(f"cadquery import error ({type(e).__name__}): {e}")
        shape  = cq.importers.importStep(path)
        vl, fl = shape.val().tessellate(0.05)
        verts  = np.array([[v.x, v.y, v.z] for v in vl], dtype=np.float32)
        faces  = np.array(fl, dtype=np.int32)
        return verts[faces]         # (N,3,3)

    @staticmethod
    def _load_stl(path: str) -> np.ndarray:
        """Load STL via numpy-stl → (N,3,3) float32."""
        try:
            from stl import mesh as stl_mesh
        except ImportError:
            raise ImportError(
                "numpy-stl not installed.\n"
                "Fix:  pip install numpy-stl"
            )
        return stl_mesh.Mesh.from_file(path).vectors.astype(np.float32)

    # ── IMU interface ─────────────────────────────────────────────────────────
    def update_imu(self, ax_g: float, ay_g: float, az_g: float,
                   gx_dps: float = 0.0, gy_dps: float = 0.0, gz_dps: float = 0.0):
        """Called from the serial reader thread (thread-safe write to floats)."""
        self._ax_val  = ax_g
        self._ay_val  = ay_g
        self._az_val  = az_g
        self._gx_dps  = gx_dps
        self._gy_dps  = gy_dps
        self._gz_dps  = gz_dps

    # ── update loop ───────────────────────────────────────────────────────────
    def _schedule_update(self):
        if self._running:
            self._update()
            self.after(self._UPDATE_MS, self._schedule_update)

    def _update(self):
        ax_g   = self._ax_val;  ay_g   = self._ay_val;  az_g   = self._az_val
        gx_dps = self._gx_dps;  gy_dps = self._gy_dps;  gz_dps = self._gz_dps

        # ── measure real elapsed time ──────────────────────────────────────
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now
        dt  = min(dt, 0.5)   # clamp: ignore stalls > 500 ms

        # ── gyro integration via Rodrigues' rotation formula ───────────────
        omega = np.array([gx_dps, gy_dps, gz_dps]) * (np.pi / 180.0)  # → rad/s
        theta = float(np.linalg.norm(omega)) * dt                       # angle this step
        if theta > 1e-8:
            axis = omega / np.linalg.norm(omega)
            K = np.array([[       0, -axis[2],  axis[1]],
                           [ axis[2],        0, -axis[0]],
                           [-axis[1],  axis[0],        0]])
            dR = (np.eye(3)
                  + np.sin(theta) * K
                  + (1.0 - np.cos(theta)) * (K @ K))
            R_gyro = self._rot @ dR
        else:
            R_gyro = self._rot

        # ── accel correction (absolute roll/pitch reference) ───────────────
        R_accel = self._accel_to_rotation(ax_g, ay_g, az_g)

        # ── complementary filter blend + re-orthogonalise via SVD ─────────
        alpha   = self._CF_ALPHA
        blended = alpha * R_gyro + (1.0 - alpha) * R_accel
        U, _, Vt = np.linalg.svd(blended)
        self._rot = U @ Vt

        # ── angle readout from current rotation matrix ─────────────────────
        # Extract Euler angles (ZYX convention: yaw-pitch-roll)
        R = self._rot
        pitch_rad = np.arcsin(-R[2, 0])
        roll_rad  = np.arctan2(R[2, 1], R[2, 2])
        yaw_rad   = np.arctan2(R[1, 0], R[0, 0])
        self.roll_lbl.config( text=f"{np.degrees(roll_rad):+6.1f}°")
        self.pitch_lbl.config(text=f"{np.degrees(pitch_rad):+6.1f}°")
        self.yaw_lbl.config(  text=f"{np.degrees(yaw_rad):+6.1f}°")

        # ── rebuild the 3-D scene ──────────────────────────────────────────
        self._redraw_scene()

    def _redraw_scene(self):
        ax = self._ax3d

        # remove previous collection without clearing whole axis (faster)
        if self._poly3d is not None:
            try:
                self._poly3d.remove()
            except Exception:
                pass
            self._poly3d = None

        R = self._rot

        if self._triangles is not None:
            poly = self._make_mesh_poly(self._triangles, R)
            ax.set_title("3D Orientation (live IMU)", color=FG, fontsize=9,
                         pad=2)
        else:
            poly = self._make_box_poly(R)
            ax.set_title("Load STEP / STL to show your PCB", color=FG_DIM,
                         fontsize=8, pad=2)

        self._poly3d = poly
        ax.add_collection3d(poly)
        self._canvas3d.draw_idle()

    # ── mesh builders ─────────────────────────────────────────────────────────
    @staticmethod
    def _make_mesh_poly(tris: np.ndarray, R: np.ndarray) -> Poly3DCollection:
        """Rotate triangles and colour by face normal (diffuse shading)."""
        rotated = tris @ R.T        # (N,3,3)

        # per-face normals
        e1 = rotated[:, 1, :] - rotated[:, 0, :]
        e2 = rotated[:, 2, :] - rotated[:, 0, :]
        normals = np.cross(e1, e2)
        nlen = np.linalg.norm(normals, axis=1, keepdims=True)
        nlen[nlen < 1e-10] = 1.0
        normals /= nlen

        # diffuse light from upper-front direction
        light = np.array([0.25, 0.35, 1.0])
        light /= np.linalg.norm(light)
        shade = np.clip(normals @ light, 0.0, 1.0)     # (N,)

        # PCB green: dark → bright
        c_dark  = np.array([0.07, 0.38, 0.10])
        c_light = np.array([0.18, 0.72, 0.25])
        colours = c_dark[None, :] + shade[:, None] * (c_light - c_dark)[None, :]
        colours = np.clip(colours, 0.0, 1.0)

        poly = Poly3DCollection(rotated, zsort="average", alpha=0.96)
        poly.set_facecolor(colours)
        poly.set_edgecolor("none")
        return poly

    @staticmethod
    def _make_box_poly(R: np.ndarray) -> Poly3DCollection:
        """Thin box (PCB-shaped placeholder) rotated by R."""
        dx, dy, dz = 1.0, 0.62, 0.06
        c = np.array([
            [-dx, -dy, -dz], [ dx, -dy, -dz],
            [ dx,  dy, -dz], [-dx,  dy, -dz],
            [-dx, -dy,  dz], [ dx, -dy,  dz],
            [ dx,  dy,  dz], [-dx,  dy,  dz],
        ], dtype=np.float64)
        c = (R @ c.T).T     # rotate all 8 corners

        faces = [
            [c[4], c[5], c[6], c[7]],   # top   (bright green)
            [c[0], c[1], c[2], c[3]],   # bottom
            [c[0], c[1], c[5], c[4]],   # front
            [c[2], c[3], c[7], c[6]],   # back
            [c[0], c[3], c[7], c[4]],   # left
            [c[1], c[2], c[6], c[5]],   # right
        ]
        fc = ["#3a8a3a", "#1a4a1a", "#1e4e1e", "#1e4e1e", "#1e4e1e", "#1e4e1e"]
        poly = Poly3DCollection(faces, zsort="average", alpha=0.92)
        poly.set_facecolor(fc)
        poly.set_edgecolor("#4a7a4a")
        poly.set_linewidth(0.6)
        return poly

    # ── helpers ───────────────────────────────────────────────────────────────
    @staticmethod
    def _accel_to_rotation(ax: float, ay: float, az: float) -> np.ndarray:
        """
        Build a 3×3 rotation matrix from accelerometer readings.
        The gravity vector (0,0,g) tells us roll & pitch; yaw stays at 0.
        """
        norm = (ax**2 + ay**2 + az**2) ** 0.5
        if norm < 0.05:
            return np.eye(3)
        ax, ay, az = ax / norm, ay / norm, az / norm

        roll  = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        cr, sr = np.cos(roll),  np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)

        Rx = np.array([[1,  0,   0 ],
                        [0,  cr, -sr],
                        [0,  sr,  cr]])
        Ry = np.array([[ cp, 0, sp],
                        [ 0,  1,  0],
                        [-sp, 0, cp]])
        return Ry @ Rx

    def _on_close(self):
        self._running = False
        self.destroy()


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

        self._orient_viewer: OrientationViewer | None = None

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

        self._orient_btn = styled_btn(bar, "3D View", self._toggle_3d_view,
                                       bg="#3a3a3a", fg=FG_DIM)
        self._orient_btn.pack(side=tk.LEFT, padx=(6, 2))

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

    # ── 3D orientation viewer ─────────────────────────────────────────────────
    def _toggle_3d_view(self):
        if self._orient_viewer is None or not self._orient_viewer.winfo_exists():
            self._orient_viewer = OrientationViewer(self.root)
            self._orient_btn.config(bg=ACCENT, fg="white")
        else:
            self._orient_viewer._on_close()
            self._orient_viewer = None
            self._orient_btn.config(bg="#3a3a3a", fg=FG_DIM)

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
        # heartbeat → status label only
        if text.startswith("HB"):
            parts = text.split()
            self.hb_lbl.config(text=parts[-1] if parts else "?", fg=GREEN)
            return

        # IMU → graph buffers + 3D viewer
        m = IMU_RE.search(text)
        if m:
            ax_g   = float(m.group(1))
            ay_g   = float(m.group(2))
            az_g   = float(m.group(3))
            gx_dps = float(m.group(4)) if m.lastindex >= 4 else 0.0
            gy_dps = float(m.group(5)) if m.lastindex >= 5 else 0.0
            gz_dps = float(m.group(6)) if m.lastindex >= 6 else 0.0
            self._imu_ax.append(ax_g)
            self._imu_ay.append(ay_g)
            self._imu_az.append(az_g)
            # forward to 3D viewer if open
            if (self._orient_viewer is not None and
                    self._orient_viewer.winfo_exists()):
                self._orient_viewer.update_imu(ax_g, ay_g, az_g,
                                               gx_dps, gy_dps, gz_dps)
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
