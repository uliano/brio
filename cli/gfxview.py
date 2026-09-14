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


def main(argv):
    ap = argparse.ArgumentParser(prog=argv[0], description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="the publisher's name, without the prefix")
    ap.add_argument("--zoom", type=int, default=0,
                    help="whole-number magnification (default: fit a sensible window)")
    args = ap.parse_args(argv[1:])

    try:
        from PySide6.QtCore import Qt, QTimer
        from PySide6.QtGui import QImage, QPainter
        from PySide6.QtWidgets import QApplication, QWidget
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

    class View(QWidget):
        def __init__(self):
            super().__init__()
            self.seg = None
            self.image = None
            self.zoom = args.zoom
            self.last_frame = None
            self.setWindowTitle("brio view - waiting for " + args.name)

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
            self.setWindowTitle("brio view - %s (%dx%d)"
                                % (args.name, seg.width, seg.height))
            self.last_frame = None
            self.update()

        def detach(self):
            # The QImage goes first: it is the thing holding a view of
            # the pages the mapping is about to give up.
            self.image = None
            if self.seg is not None:
                self.seg.close()
                self.seg = None
            self.setWindowTitle("brio view - waiting for " + args.name)
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

    app = QApplication([argv[0]])
    view = View()
    view.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
