#!/usr/bin/env python3
"""uart_stress - the host end of test_samc_uart.

The suite's own console is the wire under test, so every streaming letter
needs a peer that can pump a known pattern, verify what comes back, and
change baud rate and frame format on demand. That peer is this script; the
board asks for it one line at a time.

THE PROTOCOL, printed by the board and parsed here:

    HOST <op> <mode> <baud> <format> <window_ms> <count>

      op       echo | sink | source | burst | poke
      mode     0 irqTX+irqRX, 1 dmaTX+irqRX, 2 irqTX+dmaRX, 3 dmaTX+dmaRX
      baud     the rate the board is about to switch to
      format   e.g. 8N1, 8E1, 7O2 - bits, parity, stop bits
      window   how long the board's window lasts, in milliseconds
      count    bytes the board will emit (source only)

On seeing that line this script waits out the board's own settle, moves
its port to the announced rate and frame, runs the op for LESS than the
window - the wire must be quiet before the board speaks again, or its
report is read as payload - then goes back to 115200 8N1 and keeps reading
the console.

`poke` is the STM32G0 suite's addition and the one op whose TIMING is the
point: the script waits out HALF the window, sends `count` bytes at the
announced rate, and then waits out the rest. It is what a board that is
ASLEEP needs - the bytes have to arrive while it is in Stop, not before
it gets there and not after it has given up.

THE PATTERN is a 32-bit xorshift, low byte per step, seeded 0x12345678:
the same three shifts the firmware runs, so either end can verify the
other without a return path. A narrow frame (5, 6 or 7 bits) carries only
the low bits, and the comparison masks accordingly.

WHAT A HOST CANNOT MEASURE. A USB-serial bridge on a general-purpose
operating system is not a real-time instrument: the CH340 buffers, the
kernel schedules, and neither this script's timing nor its throughput
figures mean anything about the board. What it IS good for is CONTENT -
which byte, in which order, with which frame - and every number it prints
is a count of bytes, never a rate.

USE

    python3 tools/bench.py flash C test_samc_uart
    python3 tools/uart_stress.py --letters efghijklmnp

    python3 tools/uart_stress.py --letters h --repeat 5
    python3 tools/uart_stress.py --port /dev/ttyUSB0 --letters k

ON THE STM32G0 (board E, test_stm32_serial), whose console is the
ST-LINK's own virtual COM port and is therefore addressed by-id:

    python3 tools/bench.py flash E test_stm32_serial
    python3 tools/uart_stress.py --letters ywv \
        --port /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_\
0670FF534871754867182752-if02
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("uart_stress: pyserial is missing (pip install pyserial)")

DEFAULT_PORT = "/dev/serial/by-path/pci-0000:67:00.0-usb-0:1.2:1.0-port0"
CONSOLE_BAUD = 115200
LFSR_SEED = 0x12345678

BITS = {"5": serial.FIVEBITS, "6": serial.SIXBITS,
        "7": serial.SEVENBITS, "8": serial.EIGHTBITS,
        "9": serial.EIGHTBITS}          # pyserial has no 9-bit frame
PARITY = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN,
          "O": serial.PARITY_ODD}
STOP = {"1": serial.STOPBITS_ONE, "2": serial.STOPBITS_TWO}
MASK = {"5": 0x1F, "6": 0x3F, "7": 0x7F, "8": 0xFF, "9": 0xFF}


def lfsr_stream(n, mask=0xFF, seed=LFSR_SEED):
    """The firmware's own xorshift, n bytes of it."""
    out = bytearray(n)
    s = seed
    for i in range(n):
        s ^= (s << 13) & 0xFFFFFFFF
        s ^= s >> 17
        s ^= (s << 5) & 0xFFFFFFFF
        out[i] = (s & 0xFF) & mask
    return bytes(out)


class Board:
    def __init__(self, port, verbose=True):
        self.ser = serial.Serial(port, CONSOLE_BAUD, timeout=0.05)
        self.verbose = verbose
        self.buf = b""
        self.legs = []

    # ---- console ----------------------------------------------------------
    def read_line(self, timeout=6.0):
        end = time.time() + timeout
        while time.time() < end:
            if b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                text = line.decode("ascii", "replace").rstrip("\r")
                if self.verbose:
                    print("  |", text)
                return text
            n = self.ser.in_waiting
            self.buf += self.ser.read(n if n else 1)
        return None

    def send(self, text):
        self.ser.write(text.encode())
        self.ser.flush()

    def reconfigure(self, baud, fmt="8N1"):
        self.ser.flush()
        self.ser.baudrate = baud
        self.ser.bytesize = BITS[fmt[0]]
        self.ser.parity = PARITY[fmt[1]]
        self.ser.stopbits = STOP[fmt[2]]

    # ---- the ops ----------------------------------------------------------
    def run_op(self, op, mode, baud, fmt, window_ms, count):
        self.buf = b""
        self.ser.reset_input_buffer()
        self.reconfigure(baud, fmt)
        time.sleep(0.14)                      # the board's own settle
        mask = MASK[fmt[0]]
        # Leave half a second of the window unused, so the wire is silent
        # before the board prints its report.
        pump_s = max(0.15, window_ms / 1000.0 - 0.5)

        got = bytearray()
        sent = 0
        payload = b""
        if op == "poke":
            # THE TIMING IS THE OP: the board is asleep in the middle of
            # its own window, so the bytes go out at half of it and the
            # rest of the window is spent waiting, quietly.
            time.sleep(pump_s / 2.0)
            payload = lfsr_stream(max(count, 1), mask)
            self.ser.write(payload)
            self.ser.flush()
            sent = len(payload)
            t0 = time.time()
            while time.time() - t0 < pump_s / 2.0:
                n = self.ser.in_waiting
                if n:
                    got += self.ser.read(n)
                else:
                    time.sleep(0.005)
        elif op in ("echo", "sink", "burst"):
            payload = lfsr_stream(int(baud / 10 * pump_s * 1.1) + 256, mask)
            chunk = max(64, baud // 2000)
            t0 = time.time()
            while time.time() - t0 < pump_s and sent < len(payload):
                self.ser.write(payload[sent:sent + chunk])
                sent += chunk
                n = self.ser.in_waiting
                if n:
                    got += self.ser.read(n)
        # Collect whatever is still coming, then wait for silence.
        quiet = time.time() + 0.3
        hard = time.time() + max(0.6, pump_s + 0.6)
        while time.time() < quiet and time.time() < hard:
            n = self.ser.in_waiting
            if n:
                got += self.ser.read(n)
                quiet = time.time() + 0.08
            else:
                time.sleep(0.005)
        self.reconfigure(CONSOLE_BAUD)

        result = {"op": op, "mode": mode, "baud": baud, "format": fmt,
                  "host_sent": sent, "host_got": len(got), "first_bad": None}
        if op == "poke":
            result["host_got"] = len(got)
        elif op == "source":
            expect = lfsr_stream(max(count, len(got)), mask)
            result["host_got"] = len(got)
            for i in range(min(len(got), len(expect))):
                if got[i] != expect[i]:
                    result["first_bad"] = i
                    break
        elif op in ("echo", "burst"):
            expect = payload[:sent]
            for i in range(min(len(got), len(expect))):
                if got[i] != expect[i]:
                    result["first_bad"] = i
                    break
        else:  # sink: the board verifies, the host only pumps
            result["host_got"] = None
        return result

    # ---- letters ----------------------------------------------------------
    def run_letter(self, key, timeout=120.0):
        self.send(key)
        self.legs = []
        end = time.time() + timeout
        while time.time() < end:
            line = self.read_line(timeout=min(8.0, max(0.2, end - time.time())))
            if line is None:
                continue
            text = line.strip()
            if text.startswith("HOST "):
                parts = text.split()
                if len(parts) >= 7:
                    self.legs.append(self.run_op(parts[1], int(parts[2]),
                                                 int(parts[3]), parts[4],
                                                 int(parts[5]), int(parts[6])))
                continue
            if text.startswith("-> ") and "pass" in text:
                return self.legs, text
            if text.startswith("ALL:"):
                return self.legs, text
        return self.legs, None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help="the board's console (default: bench board C)")
    ap.add_argument("--letters", default="efghijklmnp",
                    help="which suite letters to drive, in order (the "
                         "default is the SAM's test_samc_uart, which is what "
                         "DEFAULT_PORT points at; the STM32G0's "
                         "test_stm32_serial wants 'ywv' and test_stm32_dma "
                         "wants 'u', both with an explicit --port)")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="do not echo the board's console")
    args = ap.parse_args()

    b = Board(args.port, verbose=not args.quiet)
    time.sleep(0.3)
    b.ser.reset_input_buffer()

    failures = 0
    for _ in range(args.repeat):
        for key in args.letters:
            print("=" * 64)
            print("letter", key)
            legs, tally = b.run_letter(key)
            for leg in legs:
                print("  host:", leg)
            print("  tally:", tally)
            if tally is None or " 0 fail" not in tally:
                failures += 1
    print("=" * 64)
    print("letters with failures or no tally:", failures)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
