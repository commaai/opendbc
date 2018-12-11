import os
import unittest
import platform
import time

from utilities.can_adapter import CanAdapter

from loop.deviceclient.DeviceClient import DeviceClient
from loop.loopserver.LoopServer import LoopServer


class DeviceClientIntegration(unittest.TestCase):

    @classmethod
    def setupClass(cls):
        cls.CAN_able_os = ['Linux', 'Windows']  # which leaves 'Darwin' right out
        cls.os_can_CAN = platform.system() in cls.CAN_able_os

    def setUp(self):
        if DeviceClientIntegration.os_can_CAN:
            bus_name = os.getenv('TESTCANADAPTER', default='vcan0')
            self.can_adapter = CanAdapter(bus_name)
            self.assertTrue(self.can_adapter.status())
            self.can_adapter.start()

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
        self.set_default_adapter()

    def tearDown(self):
        if DeviceClientIntegration.os_can_CAN:
            self.can_adapter.stop()
        self.device_client.catalogs.load_default()

    def set_default_adapter(self):
        adapter_data = self.loop_server.adapters.get_info()
        for adapter_info in adapter_data["Entries"]:
            name = adapter_info['Name']
            if name == "pcan1" or name == "can0":
                if adapter_info["State"] == "UP" and adapter_info["Protocol"] == "CanBus":
                    self.loop_server.adapters.set_default(name)
                    return
        self.fail("Invalid default can device")

    def test_catalog_is_setup(self):
        # this silly test is so there will be some test that runs on MacOS
        loaded_catalogs = [ x['name'] for x in self.device_client.catalogs.catalog_info()['loadStatus']['loaded'] ]
        self.assertIn("TestCatalog", loaded_catalogs, "TestCatalog is loaded")

    def test_sends_message_over_bus(self):
        if not DeviceClientIntegration.os_can_CAN:
            self.skipTest("Cannot yet test CAN traffic on MacOS")

        device = self.device_client.create_device("TestDevice1")
        device.set_property("Property1", 0x1122334455667788)
        device.send_message("Message1")

        self.assertEquals(device.message_counts("Message1")["sendCount"], 1)

        message = self.can_adapter.receive(True, 1.0)
        self.assertIsNotNone(message)

        self.assertEquals(message.arbitration_id, 100)
        self.assertEqual(message.data, bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]))
        self.assertEqual(message.dlc, 8)

    def test_receives_message_over_bus(self):
        if not DeviceClientIntegration.os_can_CAN:
            self.skipTest("Cannot yet test CAN traffic on MacOS")

        device = self.device_client.create_device("TestDevice1")
        device.set_property("Property2", 0x00)

        self.assertEquals(device.message_counts("Message2")["receiveCount"], 0)

        self.can_adapter.send(200, bytes([0x11, 0x88, 0x22, 0x77, 0x33, 0x66, 0x44, 0x55]))
        time.sleep(1)

        self.assertEquals(device.message_counts("Message2")["receiveCount"], 1)
        self.assertEquals(int(device.get_property("Property2")), 0x1188227733664455)

if __name__ == '__main__':
    unittest.main()
