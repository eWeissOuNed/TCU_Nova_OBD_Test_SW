#!/usr/bin/env python3
"""
TCU NOVA OBD – Serial Console
Usage:  python serial_console.py [PORT] [BAUD]
        python serial_console.py COM3 115200
"""
import sys
import threading
import time
import serial

# ── ANSI colours ─────────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
GREY   = "\033[90m"
RESET  = "\033[0m"

def coloured(text: str) -> str:
    if text.startswith("OK"):
        return GREEN + text + RESET
    if text.startswith("ERR"):
        return RED + text + RESET
    if text.startswith("IMU"):
        return CYAN + text + RESET
    if text.startswith("HB") or text.startswith("STD") or text.startswith("EXT"):
        return GREY + text + RESET
    return text

# ── Reader thread (prints incoming data) ─────────────────────────────────────
def reader(port: serial.Serial, stop: threading.Event) -> None:
    buf = b""
    while not stop.is_set():
        try:
            chunk = port.read(port.in_waiting or 1)
        except serial.SerialException:
            print(RED + "[disconnected]" + RESET)
            stop.set()
            break
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").rstrip()
                if text and text != ">":
                    # move to start of line, print, re-print prompt
                    print(f"\r{coloured(text)}")
                    sys.stdout.write("> ")
                    sys.stdout.flush()

# ── Built-in helper commands (client-side only) ───────────────────────────────
HELP_LOCAL = """
Local commands (not sent to device):
  .help       this message
  .quit       exit console
  .ports      list available serial ports
"""

def list_ports() -> None:
    try:
        from serial.tools import list_ports as lp
        ports = lp.comports()
        if ports:
            for p in ports:
                print(f"  {p.device:15s}  {p.description}")
        else:
            print("  (no ports found)")
    except Exception as e:
        print(f"  error: {e}")

# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    port_name = sys.argv[1] if len(sys.argv) > 1 else None
    baud      = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    if port_name is None:
        print("Available ports:")
        list_ports()
        port_name = input("Port: ").strip()

    print(f"Connecting to {port_name} @ {baud} baud …")
    try:
        port = serial.Serial(port_name, baud, timeout=0.1)
    except serial.SerialException as e:
        print(RED + f"Cannot open {port_name}: {e}" + RESET)
        sys.exit(1)

    time.sleep(0.5)  # let device settle after DTR toggle

    stop = threading.Event()
    t = threading.Thread(target=reader, args=(port, stop), daemon=True)
    t.start()

    print(f"{GREEN}Connected{RESET}. Type HELP for device commands, .help for local commands.")
    print("Press Ctrl-C to exit.\n")

    try:
        while not stop.is_set():
            try:
                line = input("> ").strip()
            except EOFError:
                break

            if not line:
                continue
            if line == ".quit":
                break
            if line == ".help":
                print(HELP_LOCAL)
                continue
            if line == ".ports":
                list_ports()
                continue

            port.write((line + "\n").encode())
            time.sleep(0.05)  # give device a moment to respond

    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        port.close()
        print("\nDisconnected.")

if __name__ == "__main__":
    main()
