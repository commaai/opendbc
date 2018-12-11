import os
import unittest
import platform
import time

from loop.deviceclient.DeviceClient import DeviceClient
from loop.loopserver.LoopServer import LoopServer


class DeviceClientIntegration(unittest.TestCase):

    def setUp(self):
        serverIPaddr = os.getenv('LOOPSERVERIPADDR', default='localhost')
        serverUrl = "http://" + serverIPaddr
        self.device_client = DeviceClient(serverUrl)
        self.loop_server = LoopServer(serverUrl)
        self.loop_server.reset_soft()

        catalog_fullname = os.path.join(os.path.dirname(__file__), '../../catalogs/IntegrationTestCatalog.json')
        with open(catalog_fullname, 'r') as file:
            self.TestCatalog = file.read().replace('\n', '').replace(' ', '')

        self.device_client.catalogs.delete_all()
        self.device_client.catalogs.put("TestCatalog",self.TestCatalog)
        self.device_client.catalogs.reload()

    def tearDown(self):
        self.device_client.catalogs.load_default()

    def test_catalog_is_setup(self):
        # this silly test is so there will be some test that runs on MacOS
        loaded_catalogs = [ x['name'] for x in self.device_client.catalogs.catalog_info()['loadStatus']['loaded'] ]
        self.assertIn("TestCatalog", loaded_catalogs, "TestCatalog is loaded")

if __name__ == '__main__':
    unittest.main()
