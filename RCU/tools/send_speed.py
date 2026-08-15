#!/usr/bin/env python3
"""Send thruster/servo setpoints to an RCU or LCU board over USB serial.

The firmware neutrals the motor if it hears nothing for 1 s, so this keeps
repeating the current setpoint as a keepalive. Closing the port (or Ctrl-C)
therefore stops the motor within a second even if this script dies badly.

Both boards accept the same line protocol, so this drives either one. Pick a
board with --board when both are plugged in; --list shows what is attached.

Interactive:

    ./tools/send_speed.py --board lcu
    speed> 0.3          set 30% forward
    speed> -0.2         20% reverse
    speed> s2 -0.5      servo 2 (LCU only)
    speed> can          MCP2515 self test
    speed> s            stop
    speed> ?            ask the board for its state
    speed> q            stop and quit

One-shot:

    ./tools/send_speed.py --speed 0.3 --hold 5
"""

import argparse
import queue
import re
import sys
import threading
import time

import serial
from serial.tools import list_ports

RP2040_VID = 0x2E8A
KEEPALIVE_HZ = 10
BAUD = 115200  # ignored by USB CDC, but pyserial wants a number


def find_boards():
    """All Raspberry Pi USB serial devices, newest SDK builds report a name."""
    return [p for p in list_ports.comports() if p.vid == RP2040_VID]


def pick_port(want):
    """Choose a board by name fragment (rcu/lcu). Both boards share a VID and
    PID, so they are told apart by the USB product string set in CMake."""
    boards = find_boards()
    if want:
        boards = [p for p in boards
                  if p.product and want.lower() in p.product.lower()]
    if not boards:
        return None, "no matching Raspberry Pi serial device found"
    if len(boards) > 1:
        listing = ", ".join(f"{p.device} ({p.product})" for p in boards)
        return None, f"several boards found: {listing}\nnarrow it with --board rcu|lcu or --port"
    return boards[0].device, None


class Rcu:
    def __init__(self, port, verbose=True):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.verbose = verbose
        self.setpoint = 0.0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()      # shuts down the reader
        self.keepalive_stop = threading.Event()  # shuts down the keepalive
        self.replies = queue.Queue()
        self._last_printed = None

        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.keepalive = threading.Thread(target=self._keepalive_loop, daemon=True)
        self.reader.start()
        self.keepalive.start()

    def _read_loop(self):
        buf = b""
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(256)
            except serial.SerialException:
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if not line:
                    continue
                self.replies.put(line)
                # Errors always surface. Acks are deduped, because the 10 Hz
                # keepalive otherwise repeats the same ack forever.
                if line.startswith("err"):
                    print(f"<< {line}")
                elif self.verbose and line != self._last_printed:
                    print(f"<< {line}")
                self._last_printed = line

    def _keepalive_loop(self):
        period = 1.0 / KEEPALIVE_HZ
        while not self.keepalive_stop.is_set():
            with self.lock:
                value = self.setpoint
            self._write(f"{value:.3f}")
            time.sleep(period)

    def _write(self, text):
        try:
            self.ser.write((text + "\n").encode())
            self.ser.flush()
        except serial.SerialException as exc:
            print(f"write failed: {exc}", file=sys.stderr)
            self.stop_event.set()

    def set_speed(self, value):
        value = max(-1.0, min(1.0, float(value)))
        with self.lock:
            self.setpoint = value
        self._write(f"{value:.3f}")
        return value

    def command(self, text):
        """Send a raw command line (stop, arm, can, fs off, sN, ?)."""
        if text in ("s", "stop"):
            # Otherwise the keepalive thread cheerfully re-sends the old
            # setpoint 100 ms later and undoes the stop.
            with self.lock:
                self.setpoint = 0.0
        self._write(text)

    def close(self):
        """Stop the motor, then shut down. Safe to call twice."""
        if self.keepalive_stop.is_set():
            return
        with self.lock:
            self.setpoint = 0.0
        # Silence the keepalive first, so "s" is the last thing on the wire
        # rather than being followed by more setpoints.
        self.keepalive_stop.set()
        self.keepalive.join(timeout=1.0)
        self._write("s")
        time.sleep(0.2)  # let the ack come back before we kill the reader
        # Wait for the reader to leave ser.read() before closing the port
        # underneath it, or it dies noisily during interpreter shutdown.
        self.stop_event.set()
        self.reader.join(timeout=1.0)
        self.ser.close()


def interactive(rcu):
    print("commands: <number> | sN <number> | s(top) | arm | can | "
          "fs on|off | ? | q(uit)")
    while True:
        try:
            raw = input("speed> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not raw:
            continue
        if raw in ("q", "quit", "exit"):
            return
        if (raw in ("s", "stop", "arm", "?", "can")
                or raw.startswith("fs ")
                or re.match(r"^s[0-3]\s", raw)):
            rcu.command(raw)
            continue
        try:
            applied = rcu.set_speed(raw)
        except ValueError:
            print(f"not a number: {raw!r}")
            continue
        if abs(applied - float(raw)) > 1e-6:
            print(f"clamped to {applied:+.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="serial device (default: autodetect)")
    ap.add_argument("-b", "--board", help="pick by USB product name, e.g. rcu or lcu")
    ap.add_argument("-l", "--list", action="store_true",
                    help="list attached boards and exit")
    ap.add_argument("--speed", type=float,
                    help="one-shot setpoint instead of the interactive prompt")
    ap.add_argument("--hold", type=float, default=3.0,
                    help="seconds to hold --speed before stopping (default: 3)")
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="only print errors from the board")
    args = ap.parse_args()

    if args.list:
        boards = find_boards()
        if not boards:
            sys.exit("no Raspberry Pi serial devices attached")
        for p in boards:
            print(f"{p.device}\t{p.product or '(unnamed)'}\t{p.serial_number or ''}")
        return

    if args.port:
        port = args.port
    else:
        port, err = pick_port(args.board)
        if port is None:
            sys.exit(f"{err}\nis the firmware running (not in BOOTSEL)?")

    try:
        rcu = Rcu(port, verbose=not args.quiet)
    except serial.SerialException as exc:
        sys.exit(f"cannot open {port}: {exc}")

    print(f"connected to {port}")
    print("!! thruster will spin -- keep clear of the prop !!")

    try:
        if args.speed is None:
            interactive(rcu)
        else:
            applied = rcu.set_speed(args.speed)
            print(f"holding {applied:+.3f} for {args.hold:.1f}s")
            time.sleep(args.hold)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        rcu.close()
        print("stopped")


if __name__ == "__main__":
    main()
