import serial
import time
import socket
import json
import subprocess
import math
import multiprocessing

import RPi.GPIO as gpio

import inv_kinematics
import pwm


#define
MaxBytes=1024*1024
host ='192.168.3.33' #ip 地址
port = 1234 #端口

pic_xy = [640, 480]
init_number_fruit = 5
init_arrange_fruit = 2

client = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #AF_INET用于IPv4寻址
client.connect((host,port))#连接服务器

serialHandle = serial.Serial("/dev/ttyAMA0", 115200)#初始化串口， 波特率为115200

hands = pwm.hand()

#{'CatEN':'E/D','XY':[0.0,0.0]}


class Car:
    def __init__(self) -> None:
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.turn_angle = 0.0
    
    def walk_turn(self, walk_distance, turn_angle, left_speed = 1.0, right_speed = 1.0):#小车行走距离。参数：行驶的距离，（左右轮的速度）
        self.left_speed, self.right_speed = left_speed, right_speed
        if walk_distance != 0 and turn_angle == 0:
            serialHandle.write((str(self.left_speed) + ';' + str(self.right_speed) + ';' + str(turn_angle)).encode('utf-8'))
            time.sleep(walk_distance // left_speed)
            serialHandle.write(('0.0' + ';' + '0.0' + ';' + '0.0').encode('utf-8'))
        elif walk_distance == 0 and turn_angle != 0:
            if turn_angle > 0:
                serialHandle.write((str(self.left_speed) + ';' + str(0.0) + ';' + str(turn_angle)).encode('utf-8'))
            elif turn_angle < 0:
                serialHandle.write((str(0.0) + ';' + str(self.right_speed) + ';' + str(turn_angle)).encode('utf-8'))       
        elif walk_distance != 0 and turn_angle != 0:
            print('实现不了')
            exit()
        self.left_speed, self.right_speed = 0.0, 0.0
    
    def auto_walk(self, left_speed, right_speed, turn_angle = 0.0):
        serialHandle.write((str(left_speed) + ';' + str(right_speed) + ';' + str(turn_angle)).encode('utf-8'))
        
    def near(self):#小车平滑靠近时使用，参数由相机返回值获得
        pass
    
class Camera:# 2 - 12 右 - 左 上 - 下
    def __init__(self) -> None:
        self.LR_angle = 90
        self.HL_angle = 90
        
        makerobo_pins = [11, 12]  # PIN管脚字典  gpio.BOARD对应实际物理管脚  gpio.BCM对应BCM编码
        gpio.setmode(gpio.BCM)   #gpio.BOARD对应实际物理管脚  gpio.BCM对应BCM编码
        gpio.setwarnings(False)  # 去除gpio口警告
        
        gpio.setup(makerobo_pins, gpio.OUT)  # 设置Pin模式为输出模式
        gpio.output(makerobo_pins, gpio.LOW)  # 设置Pin管脚为低电平(0V)关闭LED
        
        self.LR_pin = gpio.PWM(makerobo_pins[0], 50)  # 设置频率为50Hz
        self.HL_pin = gpio.PWM(makerobo_pins[1], 50)  # 设置频率为50Hz
        
        self.LR_pin.start(10)
        self.HL_pin.start(10)
    
    def turn(self, LR_angle, HL_angle):#控制相机云台旋转。参数：左右旋转角度，上下旋转角度
        
        self.LR_pin.ChangeDutyCycle(0.555 * LR_angle + 5)  # 改变占空比
        self.LR_angle = 0.555 * LR_angle + 5
        self.HL_pin.ChangeDutyCycle(0.555 * HL_angle + 5)
        self.HR_angle = 0.555 * HL_angle + 5 
    
    def follow_fruit(self, recvData):#控制相机，使识别到的果子一直在相机的中心位置
        if recvData[0] > pic_xy[0]:   #pic_xy = [640, 480]
            self.turn(self.LR_angle - 1)
        elif recvData[0] < pic_xy[0]:
            self.turn(self.LR_angle + 1)
            
        if recvData[1] > pic_xy[1]:
            self.turn(self.HL_angle - 1)
        elif recvData[1] < pic_xy[1]:
            self.turn(self.HL_angle + 1)
            
    def get_angle(self):#相机角度获取。返回：相机当前的左右上下角度
        return self.LR_angle, self.HL_angle

class Mechanical_Arm:
    def __init__(self) -> None:
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        gpio.setup(17,gpio.OUT)
        gpio.setup(17,gpio.OUT,initial = gpio.LOW)
        gpio.setup(18,gpio.OUT)
        gpio.setup(18,gpio.OUT,initial = gpio.LOW)

    def portWrite(self):                    #配置单线串口为输出
        gpio.setup(18,gpio.OUT,initial = gpio.HIGH) 
    
    def servoWriteCmd(self, id, cmd, par1 = None, par2 = None):
        buf = bytearray(b'\x55\x55')
        try:
            len = 3                                                 #若命令是没有参数的话数据长度就是3
            buf1 = bytearray(b'')
            buf2=bytearray(b'')                                     #对参数进行处理
    
            if par1 is not None: 
                len += 2                                            #数据长度加2  
                buf1.extend([(0xff & par1), (0xff & (par1 >> 8))])  #分低8位 高8位 放入缓存
            if par2 is not None:
                len += 2
                buf1.extend([(0xff & par2), (0xff & (par2 >> 8))])  #分低8位 高8位 放入缓存
                
            buf.extend([(0xff & id), (0xff & len), (0xff & cmd)])
            buf.extend(buf1)                                        #追加参数
            
            sum = 0x00
            for b in buf:  #求和
                sum += b
            sum = sum - 0x55 - 0x55  #去掉命令开头的两个 0x55
            sum = ~sum  #取反
            buf.append(0xff & sum)  #取低8位追加进缓存
            serialHandle.write(buf) #发送
            
        except Exception as e:
            print('机械臂处出现错误, servoWriteCmd', e)
            
    def grab_fruit(self, fruit_X, fruit_Y, fruit_Z):
        fruit_X =float(fruit_X)
        fruit_Y = float(fruit_Y)
        fruit_Z = float(fruit_Z)
        
        if fruit_Y != 0:
            J0,J1,J2,J3=inv_kinematics.inverseKinematics(fruit_X, fruit_Y, fruit_Z)
            servo3,servo4,servo5,servo6=inv_kinematics.transformAngelAdaptArm(J0,J1,J2,J3)
            print("servo3:",servo3,"servo4:",servo4,"servo5:",servo5,"servo6:",servo6)
            
            try:
                #handservo1.handinit()
                self.portWrite() #将单线串口配置为输出
                #发送命令 参数1 舵机id=1, 参数2 命令 = 1, 参数3 位置 = 0, 参数4时间 = 1000ms10
                self.servoWriteCmd(2,1,100,1200)
                self.servoWriteCmd(3,1,servo3,1200)
                self.servoWriteCmd(4,1,servo4,1200)
                self.servoWriteCmd(5,1,servo5,1200)
                self.servoWriteCmd(6,1,servo6,1200)
                time.sleep(1)
                hands.handopen()
                time.sleep(2.1)
                hands.handclench()
                time.sleep(1.1)
        
                self.servoWriteCmd(2,1,100,1200)
                self.servoWriteCmd(3,1,200,1200)
                self.servoWriteCmd(4,1,700,1200)
                self.servoWriteCmd(5,1,500,1200)
                self.servoWriteCmd(6,1,800,1200)
                time.sleep(1)
                hands.handopen()
                #time.sleep(1.1)
        
                self.servoWriteCmd(2,1,100,1200)
                self.servoWriteCmd(3,1,50,1200)
                self.servoWriteCmd(4,1,900,1200)
                self.servoWriteCmd(5,1,850,1200)
                self.servoWriteCmd(6,1,500,1200)
                hands.handclench()
                time.sleep(1.1)
                fruit_Y = 0
                
            except Exception as e:
                print('机械臂出现错误， jixiebi', e)

def hard_to_app():
    #需要判断格式
    receive_data = str(serialHandle.read(145)).encode('utf-8')
    client.send(receive_data)

def processing(recvData):#数据处理函数 recvData
    # app {'Vrl':'10.0;10.0'}
    # yolo {'CatEN':'E/D','XY':[0.0,0.0]}
    try:                                #判断接收到的消息的格式，如果格式错误，便不能转化为json格式，会进行报错
        recvData = json.loads(recvData)
    except json.JSONDecodeError:        #如果报错便返回，将错误的数据丢掉
        print("数据丢弃")
        
    if len(recvData)==2:                #如果接受的键值为2个
        x, y = recvData['X'], recvData['Y']#取yolo的 xy值
        return x, y
    
    elif len(recvData) == 1:
        try:                                 #若不是yolo发来的信息则是手机发送的消息此消息要经过树莓派转发给stm32
            Vrl = recvData['Vrl']
            return Vrl
            send_to_32(Vrl)
        except KeyError:
            if recvData['auto'] == 'Y':
                return 'auto_start'
                #auto(x,y)
                print(1)

           #1.1 - 3.1

def depth_get_process():
    Depth = subprocess.getoutput('echo $(./first '+ str(pic_xy[0] / 2) +' ' + str(pic_xy[1] / 2) + ')')
    Depth = Depth.split('done\n')
    Depth = Depth[-1]
    print('distance is: ', Depth)
    return Depth

def camera_LR_HL_move(my_camera):
    while True:
        recvData = client.recv(MaxBytes)#接收
        recvData = processing(recvData)    
        my_camera.follow_fruit(recvData)

def auto_grab_fruit():#18.9 - 16.9, 自动流程
    
    my_camera = Camera()#创建两个对象
    my_car = Car()
    
    fruit_arrange = init_arrange_fruit
    
    while fruit_arrange:
        
        my_camera.turn(135, 90)#相机转向，并且执行寻找水果
        time.sleep(1)
        my_car.auto_walk(1.0, 1.0)
        client.recv(MaxBytes)#接收
        my_car.auto_walk(0.0, 0.0)
        
        recvData = [0.0, 0.0]#果实追随
        while True:
            if recvData == [320, 240]:
                break
            recvData = client.recv(MaxBytes)#接收
            recvData = processing(recvData)    
            my_camera.follow_fruit(recvData)
        
        camera_angel = my_camera.get_angle() #计算距离
        depth = depth_get_process()
        car_distance_front = depth * math.sin(90 - camera_angel)
        car_distance_right = depth * math.cos(90 - camera_angel)
        
        fruit_number = init_number_fruit
        
        while fruit_number:
            camera_control = multiprocessing.Process(name='python_camera_move', target= camera_LR_HL_move, args=(my_camera,))#将服务器消息发给32
            
            camera_control.start()#靠近果实，并一直追随
            my_car.walk_turn(car_distance_front)#需要时延
            my_car.auto_walk(0, 90.0)
            time.sleep(5)
            my_car.walk_turn(car_distance_right)
            my_car.auto_walk(0, -90.0)
            time.sleep(5)
            camera_control.close()
            
            fruit_Z = depth_get_process()#获取三位坐标值
            fruit_X, fruit_Y = pic_xy[0] / 2, pic_xy[1] / 2
            
            #坐标系转换问题
            
            my_mechanical_arm = Mechanical_Arm()
            my_mechanical_arm.grab_fruit(fruit_X, fruit_Y, fruit_Z)
            
            fruit_number -= 1
        
        
        my_camera.turn(90, 90)
        if fruit_arrange % 2 == 0:
            my_car.walk_turn(0, -90.0)
        elif fruit_arrange % 2 != 0:
            my_car.walk_turn(0, 90.0)
        else:
            print('数据错误')
            exit()

def control_by_hand():
    while True:
        data_by_hand = client.recv(MaxBytes)
        if 'auto' in data_by_hand:
            break
        serialHandle.write(data_by_hand)#接收
    
    
if __name__ == '__main':
    camera_control = multiprocessing.Process(name='hard_send', target= hard_to_app, args=())#将服务器消息发给32
    while True:
        init_data = client.recv(MaxBytes)#接收
        if 'auto' in init_data:
            auto_grab_fruit()
        else:
            control_by_hand()
            auto_grab_fruit()
        
                

    
     
    

    
    
    
