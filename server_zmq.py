import zmq

context = zmq.Context()            #创建上下文
socket = context.socket(zmq.REP)   #创建Response服务端socket
socket.bind("tcp://*:5555")        #socket绑定，*表示本机ip，端口号为5555，采用tcp协议通信

while True:
    message = socket.recv()
    print(type(message))          #接收到的消息也会bytes类型(字节)
    print("收到消息：{}".format(message))
    socket.send(b"new message")   #发送消息，字节码消息