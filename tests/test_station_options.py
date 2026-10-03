import unittest
from seer_control.app import Console
from seer_control.model import MapModel
from seer_control.smap import load_smap_data


class StationOptionTests(unittest.TestCase):
    def test_model_preserves_station_options(self):
        m=MapModel({'format':'amr-console-map-v1','name':'t','nodes':[{
            'id':'LM1','x':1.0,'y':2.0,'r':1.2,'spin':True,'reachDist':0.03,'desc':'dock'
        }],'edges':[],'walls':[]})
        self.assertEqual(m.nodes['LM1']['r'],1.2)
        self.assertTrue(m.nodes['LM1']['spin'])
        self.assertEqual(m.nodes['LM1']['reachDist'],0.03)
        self.assertEqual(m.nodes['LM1']['desc'],'dock')

    def test_smap_dir_and_properties_preserved(self):
        data={'header':{'mapName':'m'},'normalPosList':[], 'advancedCurveList':[],
              'advancedPointList':[{
                  'className':'LandMark','instanceName':'LM7','pos':{'x':3.0,'y':4.0},'dir':1.5708,
                  'property':[{'key':'spin','type':'int32','int32Value':1},
                              {'key':'reachDist','type':'double','doubleValue':0.025}]
              }]}
        m=load_smap_data(data)
        n=m.nodes['LM7']
        self.assertAlmostEqual(n['r'],1.5708)
        self.assertEqual(n['spin'],1)
        self.assertAlmostEqual(n['reachDist'],0.025)

    def test_3051_payload_maps_r_and_station_options(self):
        c=Console.__new__(Console)
        p=c._station_nav_payload({
            'id':'LM7','x':1,'y':2,'r':1.57,'spin':'true','maxSpeed':0.4,
            'reachDist':0.03,'reachAngle':0.05,'desc':'not-sent','type':'LocationMark'
        })
        self.assertEqual(p['id'],'LM7')
        self.assertAlmostEqual(p['angle'],1.57)
        self.assertTrue(p['spin'])
        self.assertEqual(p['maxSpeed'],0.4)
        self.assertEqual(p['reachDist'],0.03)
        self.assertEqual(p['reachAngle'],0.05)
        self.assertNotIn('x',p)
        self.assertNotIn('desc',p)
        self.assertNotIn('type',p)

if __name__=='__main__':
    unittest.main()
