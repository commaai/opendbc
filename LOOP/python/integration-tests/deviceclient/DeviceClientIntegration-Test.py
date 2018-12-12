import os
import unittest
import platform
import time

from loop.deviceclient.DeviceClient import DeviceClient
from loop.loopserver.LoopServer import LoopServer

dbc_name_list = [
    'acura_ilx_2016_can_generated',
    'cadillac_ct6_chassis',
    'chrysler_pacifica_2017_hybrid',
    'ford_cgea1_2_bodycan_2011',
    'gm_global_a_chassis',
    'honda_accord_lx15t_2018_can_generated',
    # 'hyundai_2015_ccan',
    'kia_sorento_2018',
    'lexus_is_2018_pt_generated',
    'mercedes_benz_e350_2010',
    'tesla_can',
    'subaru_outback_2016_eyesight',
    'toyota_avalon_2017_pt_generated',
    # 'vw_golf_mk4',
]

class DeviceClientIntegration(unittest.TestCase):

    def setUp(self):
        serverIPaddr = os.getenv('LOOPSERVERIPADDR', default='localhost')
        serverUrl = "http://" + serverIPaddr

        self.loop_server = LoopServer(serverUrl)
        self.loop_server.reset_soft()

        self.device_client = DeviceClient(serverUrl)
        self.device_client.catalogs.delete_all()
        self.device_client.catalogs.reload()

        self.test_catalog = None

    def tearDown(self):
        self.device_client.catalogs.delete_all()
        self.device_client.catalogs.load_default()

    def test_verify_loop_can_load_dbc_files(self):
        for dbc_name in dbc_name_list:
            with self.subTest(dbc_file=dbc_name):
                dbc_file_name = dbc_name + '.dbc'
                print('Checking ' + dbc_file_name + '...')

                self.device_client.catalogs.delete_all()
                self.device_client.catalogs.reload()
                self.test_catalog = None

                full_file_name = os.path.join(os.path.dirname(__file__), '../../../../' + dbc_file_name)
                with open(full_file_name, 'r') as file:
                    self.test_catalog = file.read().replace('\n', '').replace(' ', '')
                self.assertIsNotNone(self.test_catalog)

                self.device_client.catalogs.add('dbc', dbc_name, self.test_catalog)
                self.device_client.catalogs.reload()

                loaded_catalogs = [ x['name'] for x in self.device_client.catalogs.catalog_info()['loadStatus']['loaded'] ]
                self.assertIn(dbc_name, loaded_catalogs, dbc_name + ' is loaded')

if __name__ == '__main__':
    unittest.main()
