import os
import unittest
import platform
import time
import re
import inspect

from loop.deviceclient.DeviceClient import DeviceClient
from loop.loopserver.LoopServer import LoopServer

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

    #
    # We're using discrete tests instead of subTest() so that the output is
    # more meaningful when a particular test fails.
    #

    @unittest.skip("contains unparsable VAL_ entry")
    def test_acura_ilx_2016_can_generated(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_cadillac_ct6_chassis(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_chrysler_pacifica_2017_hybrid(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_ford_cgea1_2_bodycan_2011(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_gm_global_a_chassis(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    @unittest.skip("contains unparsable VAL_ entry")
    def test_honda_accord_lx15t_2018_can_generated(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    @unittest.skip("contains unreadable characters")
    def test_hyundai_2015_ccan(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_kia_sorento_2018(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_lexus_is_2018_pt_generated(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_mercedes_benz_e350_2010(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_tesla_can(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_subaru_outback_2016_eyesight(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def test_toyota_avalon_2017_pt_generated(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    @unittest.skip("contains incorrect data that generates overlapping fields")
    def test_vw_golf_mk4(self):
        self.load_and_verify_dbc(re.search('^test_(.+)', self.get_my_method_name()).group(1))

    def get_my_method_name(self):
        frame = inspect.currentframe().f_back
        return inspect.getframeinfo(frame).function

    def load_and_verify_dbc(self, dbc_name):
        dbc_file_name = dbc_name + '.dbc'
        full_file_name = os.path.join(os.path.dirname(__file__), '../../../../' + dbc_file_name)
        with open(full_file_name, 'r') as file:
            #self.test_catalog = file.read().replace('\n', '').replace(' ', '')
            self.test_catalog = file.read()
        self.assertIsNotNone(self.test_catalog)

        self.device_client.catalogs.add('dbc', dbc_name, self.test_catalog)
        self.device_client.catalogs.reload()

        loaded_catalogs = [ x['name'] for x in self.device_client.catalogs.catalog_info()['loadStatus']['loaded'] ]
        self.assertIn(dbc_name, loaded_catalogs, dbc_name + ' is loaded')

if __name__ == '__main__':
    unittest.main()
