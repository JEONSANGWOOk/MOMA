import unittest
from types import SimpleNamespace
from seer_control.app import Console


class LoopPathValidationTests(unittest.TestCase):
    def test_legacy_json_edges_are_bidirectional(self):
        console=SimpleNamespace(map=SimpleNamespace(edges=[['A','B']],path_records=[]))
        self.assertTrue(Console._directed_path_exists(console,'A','B'))
        self.assertTrue(Console._directed_path_exists(console,'B','A'))
        self.assertFalse(Console._directed_path_exists(console,'A','C'))

    def test_directional_records_override_legacy_edges(self):
        console=SimpleNamespace(map=SimpleNamespace(edges=[['A','B']],path_records=[dict(a='A',b='B')]))
        self.assertTrue(Console._directed_path_exists(console,'A','B'))
        self.assertFalse(Console._directed_path_exists(console,'B','A'))
