"""Game controller -> tau_ceti.  RUN THIS ON WINDOWS (not in Ubuntu):

    python ros2\\gamepad_windows.py            (from the tau-ceti folder in PowerShell)
    python ros2\\gamepad_windows.py --debug    (prints raw stick/button values)

Why on Windows: WSL's Linux can't see game controllers (its kernel has no
joystick drivers), but Windows can. This script reads the controller and sends
the stick + button values over the local network to gamepad_bridge.py in
Ubuntu, which turns them into ROS messages (/cmd_vel and /cmd).
Needs nothing installed: it talks to Windows' own controller APIs.

Controls (Xbox-style names; PlayStation in brackets):
    Left stick   walk forward/back and sideways
    Right stick  turn left/right (left-right only)
    A  [Cross]     stand up            (g)
    B  [Circle]    EMERGENCY RELAX     (x)  all servos limp
    X  [Square]    stop walking, back to stand (s)
    Y  [Triangle]  leveling on/off     (l)
    Start [Options] print status       (p)
    RB / LB [R1 / L1]   walk faster / slower   (] / [)
    D-pad up / down     balance margin more / less (m / n): less = less rocking, closer to tipping
"""
import ctypes
import json
import socket
import sys
import time

BRIDGE = ("127.0.0.1", 9870)   # gamepad_bridge.py listens here (WSL shares localhost with Windows)
RATE_HZ = 20
DEADZONE = 0.15
DEBUG = "--debug" in sys.argv

# ---------------- XInput (Xbox and most PC controllers) ----------------
class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [("wButtons", ctypes.c_ushort), ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte), ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short), ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]

class XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_uint), ("Gamepad", XINPUT_GAMEPAD)]

XBUTTONS = {0x1000: "g", 0x2000: "x", 0x4000: "s", 0x8000: "l", 0x0010: "p",   # A B X Y Start
            0x0100: "[", 0x0200: "]",          # LB slower, RB faster
            0x0001: "m", 0x0002: "n"}          # D-pad up: more balance margin, down: less

xinput = None
for dll in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
    try:
        xinput = getattr(ctypes.windll, dll)
        break
    except OSError:
        pass

def read_xinput():
    """Returns (lx, ly, rx, set_of_letters_held) or None if no XInput pad."""
    if not xinput:
        return None
    for idx in range(4):
        st = XINPUT_STATE()
        if xinput.XInputGetState(idx, ctypes.byref(st)) == 0:
            g = st.Gamepad
            held = {letter for bit, letter in XBUTTONS.items() if g.wButtons & bit}
            if DEBUG:
                print(f"\rXInput pad {idx}: LX {g.sThumbLX:6d} LY {g.sThumbLY:6d} RX {g.sThumbRX:6d} "
                      f"buttons 0x{g.wButtons:04x}   ", end="")
            return g.sThumbLX / 32768, g.sThumbLY / 32768, g.sThumbRX / 32768, held
    return None

# ---------------- winmm (DirectInput pads, e.g. PlayStation) ----------------
class JOYINFOEX(ctypes.Structure):
    _fields_ = [(n, ctypes.c_uint) for n in (
        "dwSize", "dwFlags", "dwXpos", "dwYpos", "dwZpos", "dwRpos", "dwUpos", "dwVpos",
        "dwButtons", "dwButtonNumber", "dwPOV", "dwReserved1", "dwReserved2")]

# PlayStation (DirectInput) button bits: 0 Square, 1 Cross, 2 Circle, 3 Triangle, 4 L1, 5 R1, 9 Options
WBUTTONS = {1 << 1: "g", 1 << 2: "x", 1 << 0: "s", 1 << 3: "l", 1 << 9: "p",
            1 << 4: "[", 1 << 5: "]"}
winmm = ctypes.windll.winmm

def read_winmm():
    for jid in range(16):
        info = JOYINFOEX()
        info.dwSize = ctypes.sizeof(JOYINFOEX)
        info.dwFlags = 0xFF   # JOY_RETURNALL
        if winmm.joyGetPosEx(jid, ctypes.byref(info)) == 0:
            ax = lambda v: (v - 32767.5) / 32767.5
            held = {letter for bit, letter in WBUTTONS.items() if info.dwButtons & bit}
            if info.dwPOV == 0:          # D-pad up (POV is in hundredths of a degree)
                held.add("m")
            elif info.dwPOV == 18000:    # D-pad down
                held.add("n")
            if DEBUG:
                print(f"\rwinmm pad {jid}: X {info.dwXpos:5d} Y {info.dwYpos:5d} Z {info.dwZpos:5d} "
                      f"R {info.dwRpos:5d} U {info.dwUpos:5d} buttons 0x{info.dwButtons:04x}   ", end="")
            # left stick X/Y, right stick X = Z on PlayStation pads. Y axis is down-positive here.
            return ax(info.dwXpos), -ax(info.dwYpos), ax(info.dwZpos), held
    return None

def dz(v):
    """Deadzone: ignore small stick wobble, rescale the rest to 0..1."""
    if abs(v) < DEADZONE:
        return 0.0
    return (abs(v) - DEADZONE) / (1 - DEADZONE) * (1 if v > 0 else -1)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    prev_held = set()
    seen = None
    print("Controls" + __doc__.split("Controls", 1)[1])   # the controls list from the top of this file
    print("Reading controller... (Ctrl+C to quit)")
    while True:
        r = read_xinput()
        kind = "Xbox-style (XInput)"
        if r is None:
            r = read_winmm()
            kind = "DirectInput (PlayStation-style mapping)"
        if r is None:
            if seen is not False:
                print("\nNo controller found. Plug one in (USB or Bluetooth)...")
                seen = False
            sock.sendto(json.dumps({"vx": 0, "vy": 0, "wz": 0, "press": []}).encode(), BRIDGE)
            time.sleep(0.5)
            continue
        if seen is not True:
            print(f"\nController found: {kind}")
            seen = True
        lx, ly, rx, held = r
        pressed = sorted(held - prev_held)   # only send a button the moment it goes down
        prev_held = held
        msg = {"vx": dz(ly), "vy": -dz(lx), "wz": -dz(rx), "press": pressed}
        sock.sendto(json.dumps(msg).encode(), BRIDGE)
        if pressed and not DEBUG:
            print(f"button -> {' '.join(pressed)}")
        time.sleep(1 / RATE_HZ)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
