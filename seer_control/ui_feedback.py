"""Keep telemetry fresh and handle Tcl-owned combobox focus windows."""
import queue
import tkinter as tk


class FeedbackQueue(queue.Queue):
    latest = frozenset(('state', 'sensor1101', 'laser', 'operation_info'))

    def __init__(self):
        super().__init__()
        self.pending = {}

    def put(self, item, block=True, timeout=None):
        key = tuple(item[:2]) if item[0] in self.latest else None
        with self.not_full:
            if key is not None and key in self.pending:
                self.pending[key][0] = item
                return
            slot = [item, key]
            self._put(slot)
            if key is not None:
                self.pending[key] = slot
            self.unfinished_tasks += 1
            self.not_empty.notify()

    def _get(self):
        item, key = super()._get()
        if key is not None:
            self.pending.pop(key, None)
        return item


def focus_widget(widget):
    try:
        return widget.focus_get()
    except (KeyError, tk.TclError):
        # A ttk combobox popdown is a Tcl window without a Python Widget.
        return None


def display_has_focus(widget):
    try:
        return bool(widget.tk.call('focus', '-displayof', widget._w))
    except tk.TclError:
        return False
