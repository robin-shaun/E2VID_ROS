#client.py
#coding=utf-8  

import zmq

context = zmq.Context()
print('connect to hello world server')
socket =  context.socket(zmq.REQ)
socket.connect('tcp://localhost:5555')

for request in range(1,10):
    print('send ',request,'...')
    socket.send_string('hello')
    message = socket.recv()
    print('received reply ',request,'[',message,']')
