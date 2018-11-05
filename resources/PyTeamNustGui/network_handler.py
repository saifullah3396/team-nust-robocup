#!/usr/bin/python

import rospy
import socket as sc
from memory_data_publisher import MemoryDataPublisher
from comm_msg_types import CommMessageTypes

RECEIVE_BUFFER = 8096


class Receiver:

    def __init__(self, socket):
        self._socket = socket
        self.last_heart_beat_at = rospy.Time.now()
        self.memory_data_publisher = MemoryDataPublisher()

    def update(self):
        data = self._socket.recv(RECEIVE_BUFFER)
        self.parse_data(data)

    def parse_data(self, data):
        data_temp = data.split('\n')
        for ds in data_temp:
            if not ds:
                continue
            data_temp2 = ds.split('@')
            msg_type = int(data_temp2[0])
            try:
                if msg_type == CommMessageTypes.HEART_BEAT:
                    self.handle_heart_beat_msg()
                elif msg_type == CommMessageTypes.MEMORY_HEADER:
                    self.memory_data_publisher.handle_memory_header_msg(data_temp2[1])
                elif msg_type == CommMessageTypes.MEMORY_DATA:
                    self.memory_data_publisher.handle_memory_data_msg(data_temp2[1])
                    self.memory_data_publisher.publish()
            except AssertionError as e:
                print(e)
            #elif msg_type == CommMessageTypes.THREADS_HEADER:
            #    self.handle_threads_header_msg(data_split[1])
            #elif msg_type == CommMessageTypes.LOG_TEXT:
            #    self.handle_log_text_msg(data_split[1])
            #elif msg_type == CommMessageTypes.IMAGE:
            #    self.handle_image_msg(data_split[1])
            #elif msg_type == CommMessageTypes.FOOTSTEPS:
            #    self.handle_footstep_msg(data_split[1])
            #elif msg_type == CommMessageTypes.PF_STATES:
            #    self.handle_pf_state_msg(data_split[1])

    def handle_heart_beat_msg(self):
        self.last_heart_beat_at = rospy.Time.now()


class Sender:
    def __init__(self, socket):
        self._socket = socket


class NetworkHandler:
    def __init__(self, address, port, timeout, retry_interval):
        self._address = address
        self._port = port
        self._timeout = timeout
        self._retry_interval = retry_interval
        self.connected = False
        # To be defined in connect()
        self._socket = None
        # Sender and receiver are to be defined once the connection is made
        self._receiver = None
        self._sender = None
        pass

    def init(self):
        rospy.init_node('NetworkHandler', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        try:
            if not self.connected:
                self.connect()
                self._receiver = Receiver(self._socket)
                self._sender = Sender(self._socket)
            else:
                self._receiver.update()
                #self._sender.update()
        except IOError as e:
            self.connected = False
            print(e)

    def connect(self):
        self._socket = sc.socket(sc.AF_INET, sc.SOCK_STREAM)
        self._socket.connect((self._address, self._port))
        self.connected = True

    def cleanup(self):
        self._socket.close()


if __name__ == '__main__':
    try:
        nh = NetworkHandler('127.0.0.1', 20015, 9999, 15)
        nh.init()
    except rospy.ROSInterruptException:
        nh.cleanup()
        pass
