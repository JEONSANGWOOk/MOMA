import queue
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from seer_control.ui_feedback import FeedbackQueue, focus_widget, display_has_focus


class FeedbackTests(unittest.TestCase):
    def test_slow_gui_keeps_latest_telemetry_and_every_command(self):
        events=FeedbackQueue()
        events.put(('state',1,'old'))
        events.put(('command',1,'ack1'))
        for i in range(1000):
            events.put(('state',1,i))
            events.put(('sensor1101',1,i))
        events.put(('command',1,'ack2'))
        self.assertEqual(events.qsize(),4)
        self.assertEqual([events.get_nowait() for _ in range(4)],
                         [('state',1,999),('command',1,'ack1'),('sensor1101',1,999),('command',1,'ack2')])
        for _ in range(4):events.task_done()
        self.assertEqual(events.unfinished_tasks,0)
        self.assertFalse(events.pending)
        with self.assertRaises(queue.Empty):events.get_nowait()

    def test_reconnect_generations_are_separate(self):
        events=FeedbackQueue()
        events.put(('state',1,'old connection'))
        events.put(('state',2,'new connection'))
        self.assertEqual(events.qsize(),2)

    def test_combobox_popdown_has_focus_without_python_widget(self):
        widget=NS(focus_get=Mock(side_effect=KeyError('popdown')),
                  tk=NS(call=Mock(return_value='.combo.popdown.f.l')), _w='.')
        self.assertIsNone(focus_widget(widget))
        self.assertTrue(display_has_focus(widget))
        widget.tk.call.return_value=''
        self.assertFalse(display_has_focus(widget))


if __name__=='__main__':unittest.main()
