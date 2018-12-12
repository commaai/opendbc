import os
import unittest
import platform
import time

from loop.deviceclient.DeviceClient import DeviceClient
from loop.loopserver.LoopServer import LoopServer

dbc_files = [
    'gm_global_a_chassis.dbc',
    'tesla_can.dbc',
    'subaru_outback_2016_eyesight.dbc'
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

    def test_tesla_can_dbc_catalog(self):
        catalog_fullname = os.path.join(os.path.dirname(__file__), '../../../../tesla_can.dbc')
        with open(catalog_fullname, 'r') as file:
            self.test_catalog = file.read().replace('\n', '').replace(' ', '')
        self.assertIsNotNone(self.test_catalog)

        self.device_client.catalogs.add('dbc', "TestCatalog", self.test_catalog)
        self.device_client.catalogs.reload()
        loaded_catalogs = [ x['name'] for x in self.device_client.catalogs.catalog_info()['loadStatus']['loaded'] ]
        self.assertIn("TestCatalog", loaded_catalogs, "TestCatalog is loaded")

if __name__ == '__main__':
    unittest.main()
