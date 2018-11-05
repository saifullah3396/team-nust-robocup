#!/usr/bin/python
import socket as sc

TNRS_ADDRESS = '127.0.0.1'
TNRS_PORT = 20015
BUFFER_SIZE = 8096

def receive():
  socket = sc.socket(sc.AF_INET, sc.SOCK_STREAM)
  socket.connect((TNRS_ADDRESS, TNRS_PORT))
  while(True):
    print('start')
    data = socket.recv(BUFFER_SIZE)
    print(data)
    print('end')

receive()
