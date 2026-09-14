"""brio view <name> - watch a framebuffer a host program is publishing.

A program using brio/host/sim_display.hpp puts its pixels in POSIX
shared memory; this attaches to the same object and paints what it
finds, at its own rate, reading the very pages the program writes.  That
is what a display controller does on a part that has one, and it is why
the viewer is DUMB BY CONTRACT: it mirrors a bitmap and knows nothing of
drawing, of commands, or of what the program means by any of it.

Attaching is BY NAME and never by a path: Linux happens to expose these
objects under /dev/shm but macOS exposes them nowhere, so shm_open is
the only portable door.  The name is short for the same reason - macOS
caps it at 31 characters.

The window follows whatever is publishing.  Every quarter of a second it
re-opens the name and compares the boot id in the header; a different
one means the segment was remade - another run, or another program at
another size - and the view remaps and resizes itself.  It has to be
done that way round: after an unlink and a recreate the old mapping
still points at the old object, which stays alive while it is
referenced, so no counter inside it could ever say anything had changed.

Usage:  brio view <name> [--zoom N]
"""

import argparse
import ctypes
import ctypes.util
import mmap
import os
import struct
import sys

MAGIC = b"BRGX"
PREFIX = "/brio-gfx-"
HEADER_BYTES = 1024
POLL_MS = 250
PAINT_MS = 16
# How fast a knob turned by the wheel walks through its states: slow
# enough that a program polling on a millisecond tick sees every one.
WALK_MS = 8

# magic, version, header_bytes, boot_id, w, h, stride, format, buffers,
# front, 3 reserved, frame, palette_used
_FIXED = struct.Struct("<4sHHQHHHBBB3sIH")


def _libc():
    """shm_open lives in libc on glibc 2.34 and later and on macOS, and
    in librt before that."""
    lib = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
    if not hasattr(lib, "shm_open"):
        lib = ctypes.CDLL(ctypes.util.find_library("rt"), use_errno=True)
    lib.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
    lib.shm_open.restype = ctypes.c_int
    return lib


def peek_boot_id(lib, name):
    """The boot id alone, without mapping anything: this runs four times
    a second and a framebuffer can be megabytes."""
    fd = lib.shm_open(name.encode(), os.O_RDONLY, 0)
    if fd < 0:
        return None
    try:
        head = os.pread(fd, _FIXED.size, 0)
    except OSError:
        return None
    finally:
        os.close(fd)
    if len(head) < _FIXED.size:
        return None
    magic, _version, header_bytes, boot_id = _FIXED.unpack(head)[:4]
    if magic != MAGIC or header_bytes != HEADER_BYTES:
        return None
    return boot_id


class Segment:
    """One attachment: the mapping, the header fields, and the pixels."""

    def __init__(self, lib, name):
        self.ok = False
        fd = lib.shm_open(name.encode(), os.O_RDONLY, 0)
        if fd < 0:
            return
        try:
            size = os.fstat(fd).st_size
            if size < HEADER_BYTES:
                return
            self.map = mmap.mmap(fd, size, prot=mmap.PROT_READ)
        finally:
            os.close(fd)
        fields = _FIXED.unpack_from(self.map, 0)
        (magic, self.version, self.header_bytes, self.boot_id, self.width,
         self.height, self.stride, self.format, self.buffers, self.front,
         _reserved, self.frame_at_open, self.palette_used) = fields
        if magic != MAGIC or self.header_bytes != HEADER_BYTES:
            self.map.close()
            return
        pal_at = _FIXED.size
        self.palette = [
            tuple(self.map[pal_at + 3 * i: pal_at + 3 * i + 3]) for i in range(256)
        ]
        # The pixels as a view the caller may hand to a QImage. Held here
        # so that close() can release it before unmapping - a mapping
        # with a live export refuses to close, and the reattach path is
        # exactly where that would bite.
        self.pixels = memoryview(self.map)[self.header_bytes:]
        self.ok = True

    @property
    def frame(self):
        return struct.unpack_from("<I", self.map, 28)[0]

    def close(self):
        view = getattr(self, "pixels", None)
        if view is not None:
            view.release()
            self.pixels = None
        if getattr(self, "map", None) is not None:
            try:
                self.map.close()
            except BufferError:
                # Something still holds a view of these pages; dropping
                # the reference lets the interpreter unmap them when it
                # goes, which is better than taking the viewer down.
                pass
            self.map = None


# The input panel, /brio-in-<name>: the program owns it and reads it, we
# write into it. Offsets mirror brio/host/sim_panel.hpp; they are checked
# against the magic and the size, and a mismatch means a stale viewer
# rather than a guess.
PANEL_PREFIX = "/brio-in-"
PANEL_BYTES = 512
PANEL_MAGIC = b"BRIP"
OFF_BOOT, OFF_BUTTONS, OFF_SHAFTS = 8, 16, 17
OFF_SEQ, OFF_PRESSED, OFF_SHAFT = 24, 28, 36
OFF_BNAME, OFF_SNAME, OFF_SWITCH, OFF_DETENT = 52, 180, 244, 248
NO_SWITCH = 0xFF
NAME_MAX = 16
MAX_BUTTONS, MAX_SHAFTS = 8, 4


def peek_panel_boot_id(lib, name):
    """The panel's boot id alone, the same cheap check the framebuffer
    gets and for the same reason."""
    fd = lib.shm_open(name.encode(), os.O_RDONLY, 0)
    if fd < 0:
        return None
    try:
        head = os.pread(fd, OFF_BOOT + 8, 0)
    except OSError:
        return None
    finally:
        os.close(fd)
    if len(head) < OFF_BOOT + 8 or head[0:4] != PANEL_MAGIC:
        return None
    return struct.unpack_from("<Q", head, OFF_BOOT)[0]


class PanelLink:
    """Attached read-write, or not attached at all - a program may publish
    pixels and have no panel, and then there is simply nothing to press."""

    def __init__(self, lib, name):
        self.ok = False
        self.map = None
        fd = lib.shm_open(name.encode(), os.O_RDWR, 0)
        if fd < 0:
            return
        try:
            size = os.fstat(fd).st_size
            if size < PANEL_BYTES:
                return
            self.map = mmap.mmap(fd, size)
        finally:
            os.close(fd)
        if bytes(self.map[0:4]) != PANEL_MAGIC:
            self.map.close()
            self.map = None
            return
        self.boot_id = struct.unpack_from("<Q", self.map, OFF_BOOT)[0]
        self.ok = True

    # -- what the program declared -----------------------------------
    def _name(self, base, i):
        raw = bytes(self.map[base + i * NAME_MAX: base + (i + 1) * NAME_MAX])
        return raw.rstrip(b"\x00").decode("ascii", "replace")

    def buttons(self):
        """A control EXISTS WHEN IT IS NAMED: the slots are fixed, so an
        unnamed one is a slot the program does not use."""
        return [(i, self._name(OFF_BNAME, i)) for i in range(MAX_BUTTONS)
                if self._name(OFF_BNAME, i)]

    def shafts(self):
        return [(i, self._name(OFF_SNAME, i)) for i in range(MAX_SHAFTS)
                if self._name(OFF_SNAME, i)]

    def shaft_detent(self, i):
        """Counts per detent, declared by the program: one notch of a
        wheel is one click of a knob."""
        return max(1, self.map[OFF_DETENT + i])

    def shaft_switch(self, i):
        """Which contact is under this knob, or None. DECLARED by the
        program: a viewer that decided for itself would be inventing a
        fact about the panel."""
        v = self.map[OFF_SWITCH + i]
        return None if v == NO_SWITCH else v

    # -- what the world is doing -------------------------------------
    def press(self, i, down):
        self.map[OFF_PRESSED + i] = 1 if down else 0
        self._bump()

    def turn(self, i, counts):
        at = OFF_SHAFT + 4 * i
        now = struct.unpack_from("<i", self.map, at)[0] + counts
        struct.pack_into("<i", self.map, at, now)
        self._bump()
        return now

    def shaft_count(self, i):
        return struct.unpack_from("<i", self.map, OFF_SHAFT + 4 * i)[0]

    def _bump(self):
        seq = struct.unpack_from("<I", self.map, OFF_SEQ)[0]
        struct.pack_into("<I", self.map, OFF_SEQ, (seq + 1) & 0xFFFFFFFF)

    def close(self):
        if self.map is not None:
            try:
                self.map.close()
            except BufferError:
                pass
            self.map = None


def main(argv):
    ap = argparse.ArgumentParser(prog=argv[0], description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="the publisher's name, without the prefix")
    ap.add_argument("--zoom", type=int, default=0,
                    help="whole-number magnification (default: fit a sensible window)")
    args = ap.parse_args(argv[1:])

    try:
        from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
        from PySide6.QtGui import QImage, QPainter, QColor, QPen, QFont
        from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout,
                                       QGridLayout, QLabel)
    except ImportError:
        print("brio view: PySide6 is not installed (pip install PySide6)",
              file=sys.stderr)
        return 2

    full = PREFIX + args.name
    if len(full) > 31:
        print("brio view: name too long (%d > 31 characters)" % len(full),
              file=sys.stderr)
        return 2
    lib = _libc()

    class ScreenView(QWidget):
        def __init__(self):
            super().__init__()
            self.seg = None
            self.image = None
            self.zoom = args.zoom
            self.last_frame = None
            self.on_attach = None
            self.setMinimumSize(160, 64)

            self.paint_timer = QTimer(self)
            self.paint_timer.timeout.connect(self.maybe_repaint)
            self.paint_timer.start(PAINT_MS)

            self.poll_timer = QTimer(self)
            self.poll_timer.timeout.connect(self.poll)
            self.poll_timer.start(POLL_MS)
            self.poll()

        # -- attaching -------------------------------------------------
        def poll(self):
            """Has the thing on the other side been remade?  The boot id
            is the only honest answer; see the module comment."""
            boot_id = peek_boot_id(lib, full)
            if boot_id is None:
                if self.seg is not None:
                    self.detach()
                return
            if self.seg is not None and boot_id == self.seg.boot_id:
                return
            self.attach()

        def attach(self):
            seg = Segment(lib, full)
            if not seg.ok:
                return
            self.detach()
            self.seg = seg
            fmt = QImage.Format.Format_Mono if seg.format == 1 \
                else QImage.Format.Format_Indexed8
            # Zero-copy: the QImage is a window onto the mapped pages and
            # is never rebuilt, so a write by the publisher is on screen
            # at the next repaint with nothing copied in between.
            self.image = QImage(seg.pixels, seg.width, seg.height, seg.stride,
                                fmt)
            self.image.setColorTable(
                [0xFF000000 | (r << 16) | (g << 8) | b
                 for (r, g, b) in seg.palette[:max(seg.palette_used, 2)]])
            if self.zoom <= 0:
                self.zoom = max(1, min(8, 512 // max(seg.width, 1)))
            self.setFixedSize(seg.width * self.zoom, seg.height * self.zoom)
            self.last_frame = None
            if self.on_attach is not None:
                self.on_attach(seg)
            self.update()

        def detach(self):
            # The QImage goes first: it is the thing holding a view of
            # the pages the mapping is about to give up.
            self.image = None
            if self.seg is not None:
                self.seg.close()
                self.seg = None
            self.update()

        # -- painting --------------------------------------------------
        def maybe_repaint(self):
            if self.seg is None:
                return
            f = self.seg.frame
            if f != self.last_frame:
                self.last_frame = f
                self.update()

        def paintEvent(self, _event):
            p = QPainter(self)
            if self.image is None:
                p.fillRect(self.rect(), Qt.GlobalColor.darkGray)
                return
            # Nearest neighbour, so a pixel looks like a pixel.
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)
            p.drawImage(self.rect(), self.image)


    # -- the controls --------------------------------------------------
    #
    # A control is a thing a hand reaches for, so the pointer is what
    # chooses it: the wheel goes to the knob UNDER it, with no selecting
    # first, which is the toolkit's own behaviour and also how a panel
    # works. Whichever control the pointer is on is drawn highlighted, so
    # it is never in doubt.

    HOVER = QColor(0xFF, 0xB0, 0x30)
    IDLE = QColor(0x70, 0x78, 0x84)
    FACE = QColor(0x20, 0x24, 0x2C)
    TEXT = QColor(0xE8, 0xF0, 0xF8)

    class Control(QWidget):
        def __init__(self, label, index, link):
            super().__init__()
            self.label = label
            self.index = index
            self.link = link
            self.hot = False
            self.down = False
            self.setMouseTracking(True)

        def enterEvent(self, _e):
            self.hot = True
            self.update()

        def leaveEvent(self, _e):
            self.hot = False
            self.update()

        def edge(self):
            return HOVER if self.hot else IDLE

    class ButtonControl(Control):
        """Left button presses it, letting go releases it."""

        def __init__(self, label, index, link):
            super().__init__(label, index, link)
            self.setFixedSize(92, 44)

        def mousePressEvent(self, e):
            if e.button() == Qt.MouseButton.LeftButton:
                self.down = True
                self.link.press(self.index, True)
                self.update()

        def mouseReleaseEvent(self, e):
            if e.button() == Qt.MouseButton.LeftButton:
                self.down = False
                self.link.press(self.index, False)
                self.update()

        def paintEvent(self, _e):
            p = QPainter(self)
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            r = QRectF(2, 2, self.width() - 4, self.height() - 4)
            p.setBrush(self.edge() if self.down else FACE)
            p.setPen(QPen(self.edge(), 2))
            p.drawRoundedRect(r, 8, 8)
            p.setPen(FACE if self.down else TEXT)
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, self.label)

    class KnobControl(Control):
        """The wheel turns it, one quadrature count a notch. The middle
        button presses its own switch - the right one too, so a machine
        without a middle button can still work it."""

        PRESS_BUTTONS = (Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton)
        DEGREES_PER_COUNT = 9.0

        def __init__(self, label, index, link, push_index):
            super().__init__(label, index, link)
            self.push_index = push_index
            self.per_detent = link.shaft_detent(index)
            self.pending = 0
            self.setFixedSize(92, 92)
            self.walker = QTimer(self)
            self.walker.timeout.connect(self.walk)
            self.walker.start(WALK_MS)

        def wheelEvent(self, e):
            notches = e.angleDelta().y() // 120
            if notches:
                # A NOTCH IS A DETENT, and a detent is several quadrature
                # counts - so it is queued and WALKED, one count per
                # tick, the way a shaft passes through every state on its
                # way round. Delivering them at once would be a turn no
                # hand could make, and a decoder sampling once a tick
                # would see a quarter of them or none.
                self.pending += int(notches) * self.per_detent
                self.update()

        def walk(self):
            if self.pending == 0:
                return
            step = 1 if self.pending > 0 else -1
            self.pending -= step
            self.link.turn(self.index, step)
            self.update()

        def mousePressEvent(self, e):
            if e.button() in self.PRESS_BUTTONS and self.push_index is not None:
                self.down = True
                self.link.press(self.push_index, True)
                self.update()

        def mouseReleaseEvent(self, e):
            if e.button() in self.PRESS_BUTTONS and self.push_index is not None:
                self.down = False
                self.link.press(self.push_index, False)
                self.update()

        def paintEvent(self, _e):
            p = QPainter(self)
            p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            side = min(self.width(), self.height()) - 24
            r = QRectF((self.width() - side) / 2, 4, side, side)
            p.setBrush(self.edge() if self.down else FACE)
            p.setPen(QPen(self.edge(), 3))
            p.drawEllipse(r)
            # A mark, so a turn is visible as a turn and not as a number.
            import math
            a = math.radians(self.link.shaft_count(self.index)
                             * self.DEGREES_PER_COUNT - 90.0)
            c = r.center()
            p.setPen(QPen(FACE if self.down else TEXT, 3))
            p.drawLine(c, QPointF(c.x() + math.cos(a) * side * 0.38,
                                  c.y() + math.sin(a) * side * 0.38))
            p.setPen(TEXT)
            p.drawText(QRectF(0, self.height() - 18, self.width(), 16),
                       Qt.AlignmentFlag.AlignCenter, self.label)

    class Window(QWidget):
        """The screen on top, the controls in a grid below it."""

        def __init__(self):
            super().__init__()
            self.link = None
            self.built_for = None
            self.setWindowTitle("brio view - waiting for " + args.name)
            self.setAutoFillBackground(True)
            pal = self.palette()
            pal.setColor(self.backgroundRole(), QColor(0x18, 0x1C, 0x24))
            self.setPalette(pal)

            self.screen = ScreenView()
            self.screen.on_attach = self.on_screen
            self.controls = QGridLayout()
            self.controls.setSpacing(10)

            box = QVBoxLayout(self)
            box.setContentsMargins(12, 12, 12, 12)
            box.setSpacing(12)
            box.addWidget(self.screen)
            box.addLayout(self.controls)

            self.panel_timer = QTimer(self)
            self.panel_timer.timeout.connect(self.poll_panel)
            self.panel_timer.start(POLL_MS)
            self.poll_panel()

        def on_screen(self, seg):
            self.setWindowTitle("brio view - %s (%dx%d)"
                                % (args.name, seg.width, seg.height))
            self.adjustSize()

        def poll_panel(self):
            """The panel has its own boot id, for the same reason the
            framebuffer does."""
            name = PANEL_PREFIX + args.name
            probe = peek_panel_boot_id(lib, name)
            if probe is None:
                if self.link is not None:
                    self.clear_controls()
                return
            if self.link is not None and probe == self.link.boot_id:
                return
            link = PanelLink(lib, name)
            if not link.ok:
                return
            self.clear_controls()
            self.link = link
            self.build_controls()

        def clear_controls(self):
            while self.controls.count():
                item = self.controls.takeAt(0)
                w = item.widget()
                if w is not None:
                    w.setParent(None)
            if self.link is not None:
                self.link.close()
                self.link = None

        def build_controls(self):
            shafts = self.link.shafts()
            buttons = self.link.buttons()
            # A contact that is a knob's own switch is drawn as part of
            # that knob and not again on its own.
            under_a_knob = {self.link.shaft_switch(i) for i, _ in shafts}
            col = 0
            for i, label in shafts:
                self.controls.addWidget(
                    KnobControl(label, i, self.link, self.link.shaft_switch(i)),
                    0, col)
                col += 1
            for i, label in buttons:
                if i in under_a_knob:
                    continue
                self.controls.addWidget(ButtonControl(label, i, self.link), 0, col)
                col += 1
            self.controls.setColumnStretch(col, 1)
            self.adjustSize()

    app = QApplication([argv[0]])
    window = Window()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
