import socket
import sys
address = ('127.0.0.1', 5006)  # 服务端地址和端口
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.connect(address)  # 尝试连接服务端
except Exception:
    print('[!] Server not found ot not open')
    sys.exit()
while True:
    trigger = input('Input: ')
    s.sendall(trigger.encode())
    data = s.recv(1024)
    data = data.decode()
    print('[Recieved]', data)
    if trigger == '###':  # 自定义结束字符串
        break
s.close()
