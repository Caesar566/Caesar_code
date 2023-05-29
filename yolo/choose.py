import socket
import os
import signal
import detect_fire
import detect_person
import test1

import multiprocessing
from threading import Thread

host = "192.168.1.112"
port = 1234

print('开始连接服务器, 设置的ip为:', host, '端口为:', port)
server = socket.socket()
server.connect((host, port))
print('服务器连接成功')
# fire_data = ''
# person_data = ''
# th_person = Thread(name="person_detect", target=detect_person.person, args=(person_data,))
# th_fire = Thread(name="fire_detect", target=detect_fire.fire, args=(fire_data,))
th_person = multiprocessing.Process(name="person_detect", target=detect_person.person, args=())
th_fire = multiprocessing.Process(name="fire_detect", target=detect_fire.fire, args=())

def handle_func(*args):
    cpid, status = os.waitpid(-1, os.WNOHANG)

# th_fire = Thread(name="num_detect", target=test1.num_set, args=(fire_data,))
# person_data_list = ['', '']
# fire_data_list = ['', '']
key = 0
print('开始接收...')
signal.signal(signal.SIGCHLD, handle_func)
while True:
    recv_a = server.makefile().readline()
    print('接受的数据为:', recv_a)
    f = open('data_123.txt', 'a')
    f.write(recv_a)
    #recv_a = 'F'
    if 'FB' in recv_a:
        key = 1
        th_fire.start()
        print('火焰识别已启动')
        f.write('火焰识别开始')
        
    elif 'MB' in recv_a:
        key = 2
        th_person.start()
        print('人体识别已启动')
        f.write('人体识别开始')
        
    elif 'FS' in recv_a and key == 1:
        key = 0
        th_fire.terminate()
        del th_fire
        th_fire = multiprocessing.Process(name="fire_detect", target=detect_fire.fire, args=())
        print('火焰识别已关闭')
        f.write('火焰识别已关闭')
        
    elif 'MS' in recv_a and key == 2:
        key = 0
        th_person.terminate()
        del th_person
        th_person = multiprocessing.Process(name="person_detect", target=detect_person.person, args=())
        print('人体识别已关闭')
        f.write('人体识别已关闭')
    # if
    #  key == 1:
    #     print('list', person_data_list, fire_data_list)
    #     person_data_list[1] = person_data
    #     fire_data_list[1] = fire_data
    #     if person_data_list[0] != person_data_list[1]:
    #         server.send(person_data_list[1])
    #         person_data_list[0] = person_data_list[1]
    #     if fire_data_list[0] != fire_data_list[1]:
    #         server.send(fire_data_list[1])
    #         fire_data_list[0] = fire_data_list[1]
