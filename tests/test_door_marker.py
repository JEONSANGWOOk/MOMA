import unittest
from types import SimpleNamespace as NS
import numpy as np
import cv2
from seer_control.door_marker import marker_cells,marker_geometry

class DoorMarkerTests(unittest.TestCase):
    def test_pattern_detects_as_id_zero_without_mirroring(self):
        bitmap=np.asarray(marker_cells(),dtype=np.uint8)
        image=cv2.resize(bitmap,(180,180),interpolation=cv2.INTER_NEAREST)
        image=np.pad(image,30,constant_values=255)
        detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        _,ids,_=detector.detectMarkers(image)
        self.assertEqual(ids.tolist(),[[0]])
    def test_left_of_handle_same_height_with_white_margin(self):
        face=((-0.04,-.84,.37),(-.01,-.84,.46),(-.04,-.84,.46))
        asset=NS(faces=[face],face_materials=[36],transparent_faces=[])
        rows=marker_geometry({'asset':asset})
        paper=rows[0]
        self.assertAlmostEqual(paper[2][0][3],-.025-.055)
        self.assertAlmostEqual(paper[2][2][3],(.37+.46)/2)
        self.assertEqual(paper[3],'#ffffff')
        self.assertTrue(all(row[3]=='#080808' for row in rows[1:]))
