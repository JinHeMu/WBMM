"""X11 events targeted only at the WBMM virtual force test window."""
import ctypes as C

class KeyEvent(C.Structure):
    _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int), ('display', C.c_void_p), ('window', C.c_ulong), ('root', C.c_ulong), ('subwindow', C.c_ulong), ('time', C.c_ulong), ('x', C.c_int), ('y', C.c_int), ('x_root', C.c_int), ('y_root', C.c_int), ('state', C.c_uint), ('keycode', C.c_uint), ('same_screen', C.c_int)]

class FocusEvent(C.Structure):
    _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int), ('display', C.c_void_p), ('window', C.c_ulong), ('mode', C.c_int), ('detail', C.c_int)]

class Event(C.Union):
    _fields_ = [('key', KeyEvent), ('focus', FocusEvent), ('pad', C.c_long * 24)]
x = C.CDLL('libX11.so.6')
x.XOpenDisplay.argtypes = [C.c_char_p]
x.XOpenDisplay.restype = C.c_void_p
x.XDefaultRootWindow.argtypes = [C.c_void_p]
x.XDefaultRootWindow.restype = C.c_ulong
x.XQueryTree.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.POINTER(C.c_ulong)), C.POINTER(C.c_uint)]
x.XFetchName.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_void_p)]
x.XFree.argtypes = [C.c_void_p]
x.XStringToKeysym.argtypes = [C.c_char_p]
x.XStringToKeysym.restype = C.c_ulong
x.XKeysymToKeycode.argtypes = [C.c_void_p, C.c_ulong]
x.XKeysymToKeycode.restype = C.c_uint
x.XSendEvent.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.POINTER(Event)]
x.XFlush.argtypes = [C.c_void_p]
display = x.XOpenDisplay(None)
assert display
root = x.XDefaultRootWindow(display)

def find_window(parent):
    name = C.c_void_p()
    if x.XFetchName(display, parent, C.byref(name)) and name:
        title = C.string_at(name).decode(errors='replace')
        x.XFree(name)
        if title.startswith('WBMM Virtual Force'):
            return parent
    rt = C.c_ulong()
    p = C.c_ulong()
    children = C.POINTER(C.c_ulong)()
    n = C.c_uint()
    if not x.XQueryTree(display, parent, C.byref(rt), C.byref(p), C.byref(children), C.byref(n)):
        return None
    values = [children[i] for i in range(n.value)]
    if children:
        x.XFree(children)
    for child in values:
        found = find_window(child)
        if found:
            return found
    return None

class Keyboard:

    def __init__(self):
        self.window = find_window(root)
        assert self.window
        self.stamp = 0

    def key(self, name, pressed):
        self.stamp += 1
        e = Event()
        e.key = KeyEvent(2 if pressed else 3, 0, 1, display, self.window, root, 0, self.stamp, 0, 0, 0, 0, 0, x.XKeysymToKeycode(display, x.XStringToKeysym(name.encode())), 1)
        assert x.XSendEvent(display, self.window, 0, 1 if pressed else 2, C.byref(e))
        x.XFlush(display)

    def focus(self):
        e = Event()
        e.focus = FocusEvent(9, 0, 1, display, self.window, 0, 3)
        assert x.XSendEvent(display, self.window, 0, 1 << 21, C.byref(e))
        x.XFlush(display)
