# -*- coding: utf-8 -*-

import socket
import time
import json
import serial
import multiprocessing
import inv_kinematics
import pwm
# import  handservo1
import RPi.GPIO as gpio
import math
import subprocess
MaxBytes=1024*1024
host ='192.168.3.33' #ip 地址
port = 1234 #端口
send_enable = True
global x ,y ,z
x, y, z = 0.0, 0.0, 0.0
client = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #AF_INET用于IPv4寻址
client.connect((host,port))#连接服务器
#ser = serial.Serial('/dev/ttyUSB0', 115200)#连接串口
# pi = pigpio.pi()#初始化 pigpio库
hands = pwm.hand()
serialHandle = serial.Serial("/dev/ttyAMA0", 115200)#初始化串口， 波特率为115200
gpio.setwarnings(False)
gpio.setmode(gpio.BCM)
gpio.setup(17,gpio.OUT)
gpio.setup(17,gpio.OUT,initial = gpio.LOW)
gpio.setup(18,gpio.OUT)
gpio.setup(18,gpio.OUT,initial = gpio.LOW)



def receive(q):# 接收服务器发送的信息
    while True:
        recvData = client.recv(MaxBytes)#接收
        localTime = time.asctime(time.localtime(time.time()))
        recvData = recvData.decode(encoding='utf-8')#接收服务�
        print(recvData)
        print('接收服务器数据成功')
        ca_main(q,recvData)
        #processing(recvData)
        #raspberry_receive(recvDat)

def processing(recvData):#数据处理函数
    # recvData = {'Mode': 'Manul', 'State': 'Front', 'Speed': 'Low','armCoordinate':[0,0,0]}
    # 自动模式 {'Mode':'Auto','Distance': '0','Angle': '45'}
    #try:#判断接收到的消息的格式，如果格式错误，便不能转化为json格式，会进行报错
        #recvDat = json.loads(recvData)
    #except json.JSONDecodeError:#如果报错便返回，将错误的数据丢掉
        #print("shu ju bei diu qi")
    
    # print(type(recvDat['Mode']))
   # recvDat = recvData#接收服务器的消息
    #if recvData=='E':
    #    print("yes")
           #jixiebi()
    #     # zuobiao_x = recvData['armCoordinate'][0]
    #     # zuobiao_y = recvData['armCoordinate'][1]
    #     # zuobiao_z = recvData['armCoordinate'][2]
    #     del recvDat['armCoordinate']
    #     # zuobiao_x = recvData['armCoordinate'][0]
    #     # zuobiao_y = recvData['armCoordinate'][1]
    #     # zuobiao_z = recvData['armCoordinate'][2]
    #     # del recvData['armCoordinate']
    #     print(recvDat)
    #     # zuobiao_x.value = zuobiao_x
    #     # zuobiao_y.value = zuobiao_y
    #     # zuobiao_z.value = zuobiao_z
    #     # print(zuobiao_x)
    #     # print(zuobiao_y)
    #     # print(zuobiao_z)
   # else:
       # send_Data = str(recvData)#将数据转化为字符串的�
        #if recvData.find(';') != -1:
            #if recvData.find(';') != -1 or len(recvData) == 1:
            print('global',recvData)
            ca_main(recvData)
        #     # while True:
    #     #     len1 = len(send_Data)#在数据的两边加上_,以方便给32进行处理
    #     #     if len1 != 60:
    #     #         diference = 60-len1
    #     #         tmp = ""
    #     #         for i in range(0,diference//2):
    #     #             tmp = tmp+"_"
    #     #         for i in range(0, diference - diference//2):
    #     #             send_Data = str(send_Data) + '_'
    #     #         send_Data = tmp + send_Data
    #     #         break
    #     # print(len(send_Data))#判断字符串的字长，发送给32固定的60位的字长
    #     print(send_Data):
        #raspberry_receive(send_Data)
    #     # return zuobiao_x, zuobiao_y, zuobiao_z
    # elif recvDat['Mode'] == 'Auto':
    #     fruit = recvDat['fruit']
    #     auto(fruit)
    

def auto(fruit):
    client.send('a'.encode())
    
    recvData = [0 for j in range(3)]
    while True:
        recvData = client.recv(MaxBytes)#接收
        recvData = recvData.decode(encoding='utf-8')#接收服务器的消息
        # print(localTime, ' 接收到数据字节数:',len(recvData))
        print(recvData)
        a = str(recvData)
        a = a.replace('[', '', 8)
        a = a.replace(']', '', 8)
        while '\'' in a:
            a = a.replace('\'', '')
        while ' ' in a:
            a = a.replace(' ', '')
        list1 = a.split(',')
        print(list1)
        if list1 == 'orange' or 'peach' or 'pomegranate' or 'basket' or 'big' or 'small':
            list1_len = len(list1)
            print(list1_len)
            object_loction = list(0 for i in range(list1_len//7))
            one_object = list(0 for i in range(7))
            for i in range(list1_len//7):
                for j in range(7):
                    one_object[j] = list1[i + ((list1_len//7)*j)]
                object_loction[i] = one_object
                one_object = list(0 for i in range(7))
            print(object_loction)
            panduan = 2
            for i in range(list1_len//7):
                if object_loction[i][0] == 'big':
                    panduan = 1
                    break
                elif object_loction[i][0] == 'small':
                    panduan = 1
                    break
                elif object_loction[i][0] == None:
                    panduan = 0
                    break
                else:
                    pass
            if panduan == 2:
                print('根据水果计算距离')
                for i in range(list1_len//7):
                    if object_loction[i] == fruit:
                        if object_loction[i] == 'orange':
                            print(object_loction[i])
                            distance, angle = getSearchInfo(0,0,object_loction[i])
                            break
                        elif object_loction[i] == 'peach':
                            print(object_loction[i])
                            distance, angle = getSearchInfo(0,1,object_loction[i])
                            break
            elif panduan == 1:
                print('根据地盘计算距离')
                for i in range(list1_len//7):
                    if 'orange' == fruit:
                        if object_loction[i] == 'big':
                            print(object_loction[i])
                            distance, angle = getSearchInfo(1,0,object_loction[i])
                            break
                    elif 'peach' == fruit:
                        if object_loction[i] == 'small':
                            print(object_loction[i])
                            distance, angle = getSearchInfo(1,1,object_loction[i])
                            break
            
            send_Data = {'Mode':'Auto','Distance': distance,'Angle': angle}
            while True:
                len1 = len(send_Data)#在数据的两边加上_,以方便给32进行处理
                if len1 != 60:
                    diference = 60-len1
                    tmp = ""
                    for i in range(0,diference//2):
                        tmp = tmp+"_"
                    for i in range(0, diference - diference//2):
                        send_Data = str(send_Data) + '_'
                    send_Data = tmp + send_Data
                    break
            send_data = ser.write(send_Data.encode('utf-8'))
            time.sleep(20)
            print('自动运行执行成功')
                    
        elif list1 == [[], [], []]:
            send_Data = {'Mode':'Auto','Distance': '0','Angle': '20.0'}
            while True:
                len1 = len(send_Data)#在数据的两边加上_,以方便给32进行处理
                if len1 != 60:
                    diference = 60-len1
                    tmp = ""
                    for i in range(0,diference//2):
                        tmp = tmp+"_"
                    for i in range(0, diference - diference//2):
                        send_Data = str(send_Data) + '_'
                    send_Data = tmp + send_Data
                    break
            send_data = ser.write(send_Data.encode('utf-8'))
            time.sleep(5)
            client.send('a'.encode())
            print('发送成功')
            continue
        else:
            continue   
    
def getSearchInfo(x,y,zuobiao):
    '''
    对于自动选择的水果
    传入底盘的左上右下4点值
    传入一个水果的四个值
    :param x: 是否识别到底盘,别到为1,否则为0
    :param y: 设定的水果种类,1为桃子,0为桔子
    '''
    # 底盘和水果的直径大小
    small=0.065
    big=0.105
    juzi_diameter=0.03
    taozi_diameter=0.035
    # 相机焦距
    # taozi_dipan=['small', '388.0', '161.5', '372', '154', '404', '169']
    # juzi_dipan=['big', '176.0', '351.5', '117', '328', '235', '375']
    # orange_juli=['orange', '47.0', '79.0', '34', '68', '60', '90']
    # peach_juli=['peach', '438.0', '24.0', '427', '13', '449', '35']
    F = 457 #焦距
    print(zuobiao[1])
    if x!=0 and y!=0:#桃子底盘
        if float(zuobiao[1])>=350 and float(zuobiao[1])<=450:
            print('向右旋转15度')
            angle = 15
        elif float(zuobiao[1])>450:
            print('向旋转右30度')
            angle = 30
        elif float(zuobiao[1])<=280 and float(zuobiao[1])>=180:
            print('向左旋转15度')
            angle = -15
        elif float(zuobiao[1]) < 180:
            print('向左30旋转')
            angle = -30
        else:
            print('原地')
        pixel=abs(float(zuobiao[3])-float(zuobiao[5]))
        print(pixel)
        distance=1.0*F*small/pixel
        print(distance)
        distance = int(distance * 10)
        return distance, angle 
    elif x != 0 and y == 0:#桔子底盘
        if float(zuobiao[1]) >= 350 and float(zuobiao[1]) <= 450:
            print('向右旋转15度')
            angle = 15
        elif float(zuobiao[1]) > 450:
            print('向旋转右30度')
            angle = 30
        elif float(zuobiao[1]) <= 280 and float(zuobiao[1]) >= 180:
            print('向左旋转15度')
            angle = -15
        elif float(zuobiao[1]) < 180:
            print('向左30旋转')
            angle = -30
        else:
            print('原地')
        pixel = abs(float(zuobiao[3]) - float(zuobiao[5]))
        print(pixel)
        distance = 1.0 * F * big / pixel
        print(distance)
        distance = int(distance * 10)
        return distance, angle
    elif x == 0 and y == 0:#桔子
        if float(zuobiao[1]) >= 350 and float(zuobiao[1]) <= 450:
            print('向右旋转15度')
            angle = 30
        elif float(zuobiao[1]) > 450:
            print('向旋转右30度')
            angle = 30
        elif float(zuobiao[1]) <= 280 and float(zuobiao[1]) >= 180:
            print('向左旋转15度')
            angle = -15
        elif float(zuobiao[1]) < 180:
            print('向左30旋转')
            angle = -30
        else:
            print('原地')
        pixel = abs(float(zuobiao[3]) - float(zuobiao[5]))
        print(pixel)
        distance = 1.0 * F * juzi_diameter / pixel
        print(distance)
        distance = int(distance * 10)
        return distance, angle
    else:
        if float(zuobiao[1]) >= 350 and float(zuobiao[1]) <= 450:
            print('向右旋转15度')
            angle=15
        elif float(zuobiao[1]) > 450:
            print('向旋转右30度')
            angle = 30
            print(angle)
        elif float(zuobiao[1]) <= 280 and float(zuobiao[1]) >= 180:
            print('向左旋转15度')
            angle = -15
            print(angle)
        elif float(zuobiao[1]) < 180:
            print('向左30旋转')
            angle = -30
        else:
            print('原地')
        pixel = abs(float(zuobiao[3]) - float(zuobiao[5]))
        print(pixel)
        distance = 1.0 * F * taozi_diameter / pixel
        print(distance)
        distance = int(distance * 10)
        if angle <= 0:
            angle = str(-angle) + str('-l')
        return distance, angle

def raspberry_receive(send_Data):
    #gpio.setup(17,gpio.OUT,initial = gpio.LOW)
    if(send_enable):
        gpio.setup(18,gpio.OUT,initial = gpio.LOW)#将数据编码发送给32
        send_data = serialHandle.write(send_Data.encode('utf-8'))#接收
        print(send_Data)
        print('发送给STM32成功') 

def raspberry_send():#树莓派接收32采集的信息
    # receive_data = str(ser.read(28))
    
    try:
        while True:
            #gpio.setup(17,gpio.OUT,initial = gpio.LOW)
            #gpio.setup(18,gpio.OUT,initial = gpio.LOW)
            receive_data = str(serialHandle.read(145))#接收固定格式的信息
            receive_data = receive_data.encode('utf-8')
            #print(receive_data)
            print('接收STM32成功')
        #send(receive_data)
    except serial.SerialException:
        return None

def send(Data): #将32采集的信息发给服务器
    try:
        # Data = {'light': 200.123, 'voltage': 12.3, 'mpu': [1.1, 1.01, 1.01]}
        # Data = json.dumps(Data)#发送
        # Data = dict(Data)
        Data = list(Data)#将数据中的补位码与校验码去除
        while '$' in Data:
            Data.remove('$')
        while '@' in Data:
            Data.remove('@')
        while ' ' in Data:
            Data.remove(' ')
        Data.remove('b')
        Data.remove('\'')
        Data.reverse()
        Data.remove('\'')
        Data.reverse()
        Data_str = ''.join(Data)
        # Data_str.replace('\'','\"')
        # print(Data_str)
        Data_str = str(Data_str)#检测32发送的消息是否存在错误或者被错位的情况
        try:
            Data_dict = eval(Data_str)
        except SyntaxError:
            print('shu ju yi dui qi')#如果错位即报错，并将错误的信息丢掉
            return
        try:
            Data_dict = json.loads(Data_str)
        except json.JSONDecodeError:
            print('shu ju yi dui qi')
            return
        Data_dict = str(Data_dict)
        print(Data_str)
        
        # Data_dict = bytes(Data_dict)
        client.send(Data_str.encode())#发送给服务器
        print('发送给服务器成功')
    except SyntaxError:
        print('shu ju yi dui qi')
        return

def servoWriteCmd(id, cmd, par1 = None, par2 = None):
    buf = bytearray(b'\x55\x55')
    #print("hander",buf)
    try:
        len = 3    #若命令是没有参数的话数据长度就是3
        buf1 = bytearray(b'')
        #print("buf1_hander",buf1)
        buf2=bytearray(b'')
        #print("buf2_hander",buf2)
#对参数进行处理
        if par1 is not None:
             
            len += 2  #数据长度加2  
            buf1.extend([(0xff & par1), (0xff & (par1 >> 8))])  #分低8位 高8位 放入缓存
        if par2 is not None:
            len += 2
            buf1.extend([(0xff & par2), (0xff & (par2 >> 8))])  #分低8位 高8位 放入缓存
        #print("turn_update",buf1)
        buf.extend([(0xff & id), (0xff & len), (0xff & cmd)])
        #print("extend_id len cmd later",buf)
        buf.extend(buf1) #追加参数
        #print("extendbuf1_later",buf)
        
##计算校验和
        sum = 0x00
        for b in buf:  #求和
            sum += b
        sum = sum - 0x55 - 0x55  #去掉命令开头的两个 0x55
        sum = ~sum  #取反
        buf.append(0xff & sum)  #取低8位追加进缓存
        serialHandle.write(buf) #发送
    except Exception as e:
        print(e)
def portInit():                     #配置用到的IO口
    # pi.set_mode(17, pigpio.OUTPUT)  #配置RX_CON即GPIO17为输出
    # pi.write(17, 0)
    # pi.set_mode(27, pigpio.OUTPUT)  #配置TX_CON即GPIO27为输出
    # pi.write(27, 1)
   # gpio.setup(17,gpio.OUT,initial = gpio.HIGH)
    gpio.setup(18,gpio.OUT,initial = gpio.HIGH)

def portWrite():                    #配置单线串口为输出
    # pi.write(27,1)                  #拉高TX_CON 即 GPIO27
    # pi.write(17,0)                  #拉低RX_CON 即 GPIO17
    #gpio.setup(17,gpio.OUT,initial = gpio.HIGH)
    gpio.setup(18,gpio.OUT,initial = gpio.HIGH) 

def portRead():                     #配置单线串口为输入
    # pi.write(27,0)                  #拉低TX_CON 即 GPIO27
    # pi.write(17,1)                  #拉高RX_CON 即 GPIO17
    #gpio.setup(17,gpio.OUT,initial = gpio.HIGH)
    gpio.setup(18,gpio.OUT,initial = gpio.HIGH)
    
    
def jixiebi(q):
    while True:
       recv =  q.get()
       if recv != None and recv.find('E')==1:
            print("recv",recv)
            recv = recv.split(',')
            x =float(recv[0])
            y = float(recv[1])
            z = float(recv[2])
            print('bb',x,y,z)
            if y != 0:
                #J0,J1,J2,J3=inv_kinematics.inverseKinematics(zuobiao_x,zuobiao_y,zuobiao_z)
                print('cc',x,y,z)
                send_enable = False
                J0,J1,J2,J3=inv_kinematics.inverseKinematics(x,y,z)
                servo3,servo4,servo5,servo6=inv_kinematics.transformAngelAdaptArm(J0,J1,J2,J3)
                print("servo3:",servo3,"servo4:",servo4,"servo5:",servo5,"servo6:",servo6)
                try:
                    #handservo1.handinit()
                    portWrite() #将单线串口配置为输出
                    #发送命令 参数1 舵机id=1, 参数2 命令 = 1, 参数3 位置 = 0, 参数4时间 = 1000ms10
                    servoWriteCmd(2,1,100,1200)
                    servoWriteCmd(3,1,servo3,1200)
                    servoWriteCmd(4,1,servo4,1200)
                    servoWriteCmd(5,1,servo5,1200)
                    servoWriteCmd(6,1,servo6,1200)
                    time.sleep(1)
                    hands.handopen()
                    time.sleep(2.1)
                    hands.handclench()
                    #handservo1.zero_posture()
                    time.sleep(1.1)
            
                    servoWriteCmd(2,1,100,1200)
                    servoWriteCmd(3,1,200,1200)
                    servoWriteCmd(4,1,700,1200)
                    servoWriteCmd(5,1,500,1200)
                    servoWriteCmd(6,1,800,1200)
                    time.sleep(1)
                    hands.handopen()
                    #time.sleep(1.1)
            
                    #handservo1.handinit()
            
                    servoWriteCmd(2,1,100,1200)
                    servoWriteCmd(3,1,50,1200)
                    servoWriteCmd(4,1,900,1200)
                    servoWriteCmd(5,1,850,1200)
                    servoWriteCmd(6,1,500,1200)
                    #handservo1.zero_posture()
                    hands.handclench()
                    time.sleep(1.1)
                    y = 0
                    send_enable = True
                except Exception as e:
                    print(e)

def ca_main(q,recvData):
    #while True
            print(recvData)
            recvData = recvData.split(' ')
            recvData[1] = 480 - int(recvData[1])
            Depth = subprocess.getoutput('echo $(./first '+ str(recvData[0]) +' ' + str(recvData[1]) + ')')
            Depth = Depth.split('done\n')
            Depth = Depth[-1]
            print('distance is: ', Depth)
            client.send(Depth.encode())
            print("send ok ",Depth)
            print(type(recvData[0]))

            x =float(recvData[0])
            y =float(recvData[1])
            z =float(Depth)
            q.put(str(-7)+','+str(25)+','+str(9))
            q.put(None)
            print('send xyz',x,y,z)
            
        #x = recvData[0]
        #y = recvData[1]
        #z = Depth
        #x = x - 320
        #y = y - 240
            
         
            
if __name__ == '__main__':
    #zuobiao_x = multiprocessing.Value("f",0.0)
    #zuobiao_y = multiprocessing.Value("f",0.0)
    #zuobiao_z = multiprocessing.Value("f",0.0)
    q=multiprocessing.Queue()
    #设置两个进程，接收与发送分别进行
    threading1 = multiprocessing.Process(name='server',target=receive,args=(q,))#将服务器消息发给32
    threading2 = multiprocessing.Process(name='STM32',target=raspberry_send)#将32的消息发给服务器
    threading3 = multiprocessing.Process(name='jixiebi',target=jixiebi,args=(q,))
   # threading4 = multiprocessing.Process(name='ca_main',target=ca_main,args=(q,)) 
    threading1.start()#进行进程1
    # threading1.join()
    print('1')
    threading2.start()#进行进程2
    print('2')
    # portInit()
    print('初始化成功')
   # threading4.start()
   # print('4')
    threading3.start()
    print('3')
    while True:#进行循环，使两个进程持续运行
        time.sleep(2)
        pass