import unittest
from seer_control.smap import load_smap_data

class SmapCurveTests(unittest.TestCase):
    def base(self, curve):
        return {'header':{'mapName':'curve_test'},'normalPosList':[], 'advancedPointList':[], 'advancedCurveList':[curve]}

    def test_nested_control_positions_are_curved(self):
        data=self.base({'className':'BezierPath','startPos':{'pos':{'x':0,'y':0}},'endPos':{'pos':{'x':2,'y':0}},
                        'controlPos1':{'pos':{'x':0,'y':1}},'controlPos2':{'pos':{'x':2,'y':1}}})
        m=load_smap_data(data)
        self.assertGreater(len(m.curves[0]),2)
        self.assertGreater(max(y for x,y in m.curves[0]),0.5)

    def test_control_point_aliases_are_curved(self):
        data=self.base({'className':'BezierCurve','startPoint':{'x':0,'y':0},'endPoint':{'x':2,'y':0},
                        'controlPoint1':{'x':0,'y':1},'controlPoint2':{'x':2,'y':1}})
        m=load_smap_data(data)
        self.assertIn('curved=1',m.curve_diagnostics)
        self.assertGreater(max(y for x,y in m.curves[0]),0.5)

if __name__=='__main__':unittest.main()
