import unittest
import numpy as np
from seer_control.smooth_renderer import packed_triangles

class PackedRenderingTests(unittest.TestCase):
    def test_full_geometry_and_normals(self):
        faces=[((0,0,0),(1,0,0),(0,1,0)),((2,0,0),(3,0,0),(2,1,0))]
        data=packed_triangles(faces)
        self.assertEqual(data.dtype,np.float32)
        self.assertTrue(data.flags.c_contiguous)
        np.testing.assert_array_equal(data[:,3:],np.asarray(faces).reshape(-1,3))
        np.testing.assert_allclose(data[:,:3],np.tile([0,0,1],(6,1)))

    def test_polygon_triangulation_and_degenerate_faces(self):
        data=packed_triangles([((0,0,0),(1,0,0),(1,1,0),(0,1,0))])
        self.assertEqual(len(data),6)
        self.assertTrue(np.isfinite(data).all())
        self.assertTrue(np.isfinite(packed_triangles([((0,0,0),)*3])).all())
        self.assertEqual(packed_triangles([]).shape,(0,6))
