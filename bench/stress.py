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

THE LEGS STOP AT THE BRIDGE'S CEILING. A USB-serial bridge carries only
what it carries: the ST-LINK's virtual COM port on board E is byte-exact
to 921600 in both directions, corrupt at 2 Mbaud, and at 3 Mbaud seven
bytes of twelve thousand arrive. Pumping AT such a rate costs more than
the wasted bytes - the operating system goes on delivering the queue
after the board has gone home to 115200, where the console's ring reads
it as MENU LETTERS and the letter's closing tally is lost with it. So a
HOST line announcing a rate above VCP_CEILING is run PASSIVELY by
default (the port follows the announced rate and drains, but nothing is
pumped) and is run in full only with --beyond-vcp. The firmware's own
default ladders stop there too; test_stm32_dma keeps the rungs above it
in a separate letter, `w`, outside `z`.

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

    brio flash C test_samc_uart
    brio stress --letters efghijklmnp

    brio stress --letters h --repeat 5
    brio stress --port /dev/ttyUSB0 --letters k

ON THE STM32G0 (board E, test_stm32_serial), whose console is the
ST-LINK's own virtual COM port and is therefore addressed by-id:

    brio flash E test_stm32_serial
    brio stress --letters ywv \
        --port /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_\
0670FF534871754867182752-if02

and test_stm32_dma's letter u, whose ladder stops at that bridge's own
ceiling - with letter w, and only with --beyond-vcp, for the rungs above:

    brio flash E test_stm32_dma
    brio stress --letters u --port <board E's console>
    brio stress --letters w --beyond-vcp --port <the same>
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
# The highest rate any bridge on this desk is MEASURED to carry byte-exact
# (board E's ST-LINK VCP; the CH340s reach 3 Mbaud and say so with
# --beyond-vcp). Above it a leg is passive unless asked for.
VCP_CEILING = 921600

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


def default_ceiling(port):
    """The rate this BRIDGE is measured to carry byte-exact, or None.

    It is a property of the wire and not of the board, so it is decided
    by which port was opened: board E's console is the ST-LINK's own
    virtual COM port, measured corrupt above 921600 (docs/stm32g0/
    dma.md), while the CH340s on the AVR and SAM boards carry 3 Mbaud
    (docs/samc21/sercom.md) and get no ceiling at all. `--ceiling` and
    `--beyond-vcp` override this either way.
    """
    return VCP_CEILING if "stlink" in port.lower() else None


class Board:
    def __init__(self, port, verbose=True, ceiling=None):
        self.ser = serial.Serial(port, CONSOLE_BAUD, timeout=0.05)
        self.verbose = verbose
        # None = pump at whatever the board announces; a number = run any
        # leg above it PASSIVELY (follow the rate, drain, pump nothing).
        self.ceiling = ceiling
        self.buf = b""
        self.legs = []
        self.last_window_s = 0.0

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
    def run_op(self, op, mode, baud, fmt, window_ms, count, t_host=None):
        self.buf = b""
        self.ser.reset_input_buffer()
        self.reconfigure(baud, fmt)
        time.sleep(0.14)                      # the board's own settle
        mask = MASK[fmt[0]]
        # Leave half a second of the window unused, so the wire is silent
        # before the board prints its report.
        pump_s = max(0.15, window_ms / 1000.0 - 0.5)

        # THE WINDOW IS A DEADLINE AND NOT A SUGGESTION. Everything below
        # runs at the leg's own baud rate and frame; the moment the
        # board's window ends it goes back to 115200 8N1 and starts
        # printing, and a host still reading at 3 Mbaud 7E1 turns that
        # report into payload - the tally with it. The board's own
        # window started when it printed the HOST line, so that is what
        # this clock is set by, and every phase below stops at it.
        #
        # (test_stm32_serial repairs this from the FIRMWARE side, by
        # waiting out the window plus nine hundred milliseconds before it
        # speaks. test_stm32_dma does not, and its letter u lost its
        # closing tally to exactly this. Fixing it here fixes it for both
        # and for anything written next.)
        if t_host is None:
            t_host = time.time()
        deadline = t_host + window_ms / 1000.0

        got = bytearray()
        sent = 0
        payload = b""
        # ABOVE THE BRIDGE'S CEILING, PUMP NOTHING. Feeding a wire that
        # cannot take the bytes does not just waste them: the operating
        # system delivers the queue AFTER the board is home at 115200,
        # where the console's ring reads it as menu letters and the
        # letter's closing tally goes with it. The port still follows the
        # announced rate and still drains, so a leg the BOARD sources is
        # observed; only the host's own pump is withheld.
        declined = self.ceiling is not None and baud > self.ceiling
        if declined:
            if self.verbose:
                print("  ! %d baud is above this bridge's measured ceiling "
                      "(%d): pumping nothing (--beyond-vcp to force)"
                      % (baud, self.ceiling))
            while time.time() < deadline:
                n = self.ser.in_waiting
                if n:
                    got += self.ser.read(n)
                else:
                    time.sleep(0.005)
        elif op == "poke":
            # THE TIMING IS THE OP: the board is asleep in the middle of
            # its own window, so the bytes go out at half of it and the
            # rest of the window is spent waiting, quietly.
            time.sleep(min(pump_s / 2.0, max(0.0, deadline - time.time())))
            payload = lfsr_stream(max(count, 1), mask)
            self.ser.write(payload)
            self.ser.flush()
            sent = len(payload)
            t0 = time.time()
            while time.time() - t0 < pump_s / 2.0 and time.time() < deadline:
                n = self.ser.in_waiting
                if n:
                    got += self.ser.read(n)
                else:
                    time.sleep(0.005)
        elif op in ("echo", "sink", "burst"):
            payload = lfsr_stream(int(baud / 10 * pump_s * 1.1) + 256, mask)
            chunk = max(64, baud // 2000)
            t0 = time.time()
            while (time.time() - t0 < pump_s and sent < len(payload)
                   and time.time() < deadline):
                self.ser.write(payload[sent:sent + chunk])
                sent += chunk
                n = self.ser.in_waiting
                if n:
                    got += self.ser.read(n)
        # AND STOP FEEDING A BRIDGE THAT CANNOT KEEP UP. At a rate the
        # VCP cannot carry, the pump above queues far more than the wire
        # will take inside the window, and the operating system goes on
        # delivering it AFTER the board has gone back to 115200 - where
        # it lands in the console's receive ring and the menu loop reads
        # it as LETTERS. Whatever has not left yet is dropped here,
        # which is the host's half of that; the board's half is to drain
        # its own ring before it looks for the next command.
        try:
            self.ser.reset_output_buffer()
        except Exception:
            pass
        # Collect whatever is still coming, then wait for silence - but
        # NEVER past the window. A leg that ends early gets its 300 ms of
        # quiet; one that runs to the edge gets whatever is left, and the
        # port is back at the console's own rate before the board speaks.
        quiet = time.time() + 0.3
        hard = min(deadline, time.time() + max(0.6, pump_s + 0.6))
        while time.time() < quiet and time.time() < hard:
            n = self.ser.in_waiting
            if n:
                got += self.ser.read(n)
                quiet = time.time() + 0.08
            else:
                time.sleep(0.005)
        self.reconfigure(CONSOLE_BAUD)
        self.last_window_s = window_ms / 1000.0

        result = {"op": op, "mode": mode, "baud": baud, "format": fmt,
                  "host_sent": sent, "host_got": len(got), "first_bad": None,
                  "declined": declined}
        if declined:
            pass
        elif op == "poke":
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
        """Drive one suite letter.

        THE DEADLINE GROWS WITH THE WORK. `timeout` bounds how long the
        script waits for the board to say ANYTHING, not how long a letter
        may take: a letter of ten legs whose windows are 600 ms each is
        not a hung board, and test_stm32_dma's letter u used to be
        declared one at the flat two-minute mark - the legs all ran and
        the tally was never waited for. Every announced window is added
        back, so the budget is spent on silence and never on work.
        """
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
                    t_host = time.time()
                    window_ms = int(parts[5])
                    self.legs.append(self.run_op(parts[1], int(parts[2]),
                                                 int(parts[3]), parts[4],
                                                 window_ms, int(parts[6]),
                                                 t_host))
                    end += window_ms / 1000.0 + 2.0
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
                         "test_stm32_serial wants 'ywv', test_stm32_dma "
                         "wants 'u' - or 'w' with --beyond-vcp - all with an "
                         "explicit --port)")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--ceiling", type=int, default=None,
                    help="the highest rate this bridge is trusted to carry "
                         "byte-exact; a leg announced above it is run "
                         "PASSIVELY (the port follows the rate and drains, "
                         "but nothing is pumped). The default is 921600 for "
                         "board E's ST-LINK virtual COM port, measured, and "
                         "none for any other port - the CH340s carry 3 Mbaud")
    ap.add_argument("--beyond-vcp", action="store_true",
                    help="lift that ceiling and pump at whatever the board "
                         "announces (test_stm32_dma's letter w wants this; "
                         "what comes back above the ceiling is the bridge's "
                         "noise, and the letter judges nothing)")
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="do not echo the board's console")
    args = ap.parse_args()

    if args.beyond_vcp:
        ceiling = None
    elif args.ceiling is not None:
        ceiling = args.ceiling if args.ceiling > 0 else None
    else:
        ceiling = default_ceiling(args.port)
    if ceiling is not None:
        print("bridge ceiling: %d baud (--beyond-vcp lifts it)" % ceiling)

    b = Board(args.port, verbose=not args.quiet, ceiling=ceiling)
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
