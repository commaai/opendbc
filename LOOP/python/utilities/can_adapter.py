import can
import threading
import queue
import re
import subprocess

from can.interfaces.interface import Bus
from can import Message

class CanAdapter(object):

    def __init__(self, can_interface):
        can.rc['interface'] = 'socketcan_ctypes'
        if can_interface is None:
            can_interface = 'can0'
        self._can_interface = can_interface
        if not self.status():
            raise IOError("CAN Bus not available")
        self._bus = Bus(self._can_interface)
        self._bitrate = 500000
        if self._bus is None:
            raise IOError("Can't find CAN bus interface")
        self._rcv_thread_quit = False
        self._rcv_thread = None
        self._rcv_queue = queue.Queue(maxsize = 1024)

        can.set_logging_level("Info")

    def status(self):
        ip_link_cmd = "ip link | grep " + self._can_interface
        can0_link = subprocess.check_output(ip_link_cmd, shell = True).decode("utf-8")
        states = re.sub('[!<>]', '', can0_link.split()[2]).split(',')
        return 'UP' in states

    def start(self, callback = None):
        if not self._rcv_thread:
            self._callback = callback
            self._rcv_thread = threading.Thread(target=self._process_receive)
            self._rcv_thread_quit = False
            self._rcv_thread.start()

    def stop(self):
        if self._rcv_thread:
            self._rcv_thread_quit = True
            self._rcv_thread.join(5.0)
            self.join_failed = self._rcv_thread.is_alive()
            self._rcv_thread = None

    def send(self, id, data):
        Message.extended_id = False
        Message.id_type = Message.extended_id 
        Message.is_remote_frame = False
        Message.is_error_frame = False
        Message.arbitration_id = id
        Message.dlc = len(data)
        Message.data = data
        try:
            self._bus.send(Message)
            return True
        except:
            return False

    def receive(self, block = False, timeout = None):
        try:
            msg = self._rcv_queue.get(block, timeout)
        except:
            msg = None
        return msg

    def flush(self):
        while self.receive() is not None:
            continue

    def _process_receive(self):
        while not self._rcv_thread_quit:
            msg = self._bus.recv(0.25)  # seconds
            if msg:
                try:
                    if self._callback is not None:
                        self._callback(msg)
                    self._rcv_queue.put(msg, False)
                except:
                    pass
