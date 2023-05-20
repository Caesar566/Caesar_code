import time
import numpy as np
import math 
 


def inverseKinematics(x,y,z):
    array_j3=[];
    angle=np.zeros([200,4])
    l1=10.1
    l2=9.7
    l3=21.3
    row=0
    #new l1 
    #l1=10.5
    #l2=14.9
    #l3=17.0
    #print(l1,l2,l3)   
    i=0
    for j1 in range(-90,91):
        j0=math.atan2(y,x)
        a=x/math.cos(j0);
        if(x==0):
            a=y
        b=z;
        #print('j1', j1)
        j1=math.radians(j1)
        #print("j1rad:",j1)
       
        temp=(math.pow(a,2)+math.pow(b,2)+math.pow(l1,2)-math.pow(l2,2)-pow(l3,2)-2*a*l1*(math.sin(j1))-2*b*l1*math.cos(j1))/(2*l2*l3);
        #print("temp:",temp)
        if temp<=1 and temp>=-1:
            
            j3=math.acos(temp)
            #print("j3:",j3)
       # else:
       #    print("error j3")
            m=l2*math.sin(j1)+l3*math.sin(j1)*math.cos(j3)+l3*math.cos(j1)*math.sin(j3)
            n=l2*math.cos(j1)+l3*math.cos(j1)*math.cos(j3)-l3*math.sin(j1)*math.sin(j3)    
            t=a-l1*math.sin(j1)
            p=pow(pow(n,2)+pow(m,2),0.5)
            q=math.asin(m/p)
            temp_tp=t/p
            if temp_tp<=1 and temp_tp>=-1:
                j2=math.asin(temp_tp)-q
                #print("j2:",j2)
       # else:
        #   print("error j2")
                x1=(l1*math.sin(j1)+l2*math.sin(j1+j2)+l3*math.sin(j1+j2+j3))*math.cos(j0)
                y1=(l1*math.sin(j1)+l2*math.sin(j1+j2)+l3*math.sin(j1+j2+j3))*math.sin(j0)
                z1=l1*math.cos(j1)+l2*math.cos(j1+j2)+l3*math.cos(j1+j2+j3)
                j0=math.degrees(j0)
                j1=math.degrees(j1)
                j2=math.degrees(j2)
                j3=math.degrees(j3)
                #print('first j0 j1 j2 j3 :', j0, j1, j2, j3)
                #print('first xyz :', x, y, z)
                if x1<(x+1)and x1>(x-1)and y1<(y+1)and y1>(y-1)and z1<(z+1)and z1>(z-1):
                     if j3>=0 and j3<=110 and j2>-135 and j2<=135:
                        i=1 
                        #print("j0:",j0,"j1:",j1,"j2:",j2,"j3:",j3)
                        #print("x1:",x1,"y1:",y1,"z1:",z1)
                #return math.degrees(j0),j1,j2,j3
                        angle[row][0]=j0
                        angle[row][1]=j1
                        angle[row][2]=j2
                        angle[row][3]=j3
                        row+=1
    # print(row)
    
    for j1 in range(-90,91):
        j0=math.atan2(y,x)
        a=x/math.cos(j0);
        if(x==0):
            a=y
        b=z;
        # print(j1)
        j1=math.radians(j1)
        # print("j1:",j1)
       
        temp=(math.pow(a,2)+math.pow(b,2)+math.pow(l1,2)-math.pow(l2,2)-pow(l3,2)-2*a*l1*(math.sin(j1))-2*b*l1*math.cos(j1))/(2*l2*l3);
        # print("temp:",temp)
        if temp<=1 and temp>=-1:
            
            j3=math.acos(temp)#
            # print("j3:",j3)
       # else:
       #    print("error j3")
            m=l2*math.sin(j1)+l3*math.sin(j1)*math.cos(j3)+l3*math.cos(j1)*math.sin(j3)
            n=l2*math.cos(j1)+l3*math.cos(j1)*math.cos(j3)-l3*math.sin(j1)*math.sin(j3)    
            t=a-l1*math.sin(j1)
            p=pow(pow(n,2)+pow(m,2),0.5)
            q=math.asin(m/p)
            temp_tp=t/p
            if temp_tp<=1 and temp_tp>=-1:
                
                j2=-(math.asin(temp_tp)-q)
         #  print("j2:",j2)
       # else:
        #   print("error j2")
                x1=(l1*math.sin(j1)+l2*math.sin(j1+j2)+l3*math.sin(j1+j2+j3))*math.cos(j0)
                y1=(l1*math.sin(j1)+l2*math.sin(j1+j2)+l3*math.sin(j1+j2+j3))*math.sin(j0)
                z1=l1*math.cos(j1)+l2*math.cos(j1+j2)+l3*math.cos(j1+j2+j3)
                j1=math.degrees(j1)
                j2=math.degrees(j2)
                j3=math.degrees(j3)
                j0=math.degrees(j0)
                #print('second j :', j0, j1, j2, j3)
                #print('second xyz :', x, y, z) #根据传入的xyz三维坐标来计算出多个解
                if x1<(x+1)and x1>(x-1)and y1<(y+1)and y1>(y-1)and z1<(z+1)and z1>(z-1):
                     if j3>=0 and j3<=110 and j2>-135 and j2<=135:
                         i=1 
                         #print("j0:",math.degrees(j0),"j1:",j1,"j2:",j2,"j3:",j3)
                         #print("x1:",x1,"y1:",y1,"z1:",z1)
                         angle[row][0]=j0
                         angle[row][1]=j1
                         angle[row][2]=j2
                         angle[row][3]=j3
                         row+=1
    #print(row)
    '''                    
    for angle_1 in range(0,angle.shape[0]):
        for angle_2 in range(0,angle.shape[1]):
            print (angle[angle_1][angle_2],end="  ")
        print()
    '''  
    for angle_1 in range(0,angle.shape[0]):
        for angle_2 in range(2,3):
            array_j3.append(angle[angle_1][angle_2])
            #print (angle[angle_1][angle_2],end="  ")
        #print()
    #print('j3array:', array_j3)
    
    array_j3max=array_j3[0]
    index_j3 = 0
    for i_arr in range(0,len(array_j3)):
        if array_j3max<array_j3[i_arr]:
             array_j3max=array_j3[i_arr]
             index_j3=i_arr
             #print('index_j3', index_j3)
         
       
            
    #print('index_j3:', index_j3)
    #print('angle is', angle)
    #print(angle[index_j3])
    return(angle[index_j3])
    
    if i==0:
        print("no answer")
    
   
            
            
def transformAngelAdaptArm(j0, j1, j2, j3):
    
    if j0<=180 and j0>=0:
        if j0>=90:
            servo6=int(500+(350/90.0)*(j0-90))
        else:
            servo6=int(500-(400/90.0)*(90-j0))
    if j1<=90 and j1>=-90:
        if j1>=0:
            servo5=int(500-(400/87.0)*j1)
        else:
            servo5=int(500+(450/90.0)*(-j1))
    if j3>-135 and j2<=135:
        servo4=int(500+(500/120.0)*j2)
    if j3>=0 and j3<=110:
        servo3=int(500-((500-115)/90.0)*j3)
    
    #将逆运动学算出的角度转换为舵机对应的脉宽值
    return servo3, servo4,  servo5, servo6
 

if __name__ == "__main__":
    #inverseKinematics(10,5,10) W0
    #可用值（-7, 25, 6）
    for i in range(-40, 40):
        for j in range(-40, 40):
            for k in range(-40, 40):
                J0,J1,J2,J3=inverseKinematics(i, j, k)
    #inverseKinematics(-4,30,0)
    #J0,J1,J2,J3=inverseKinematics(0,25,0)
                if (J0 or J1 or J2 or J3) == 0:
                    #print('数值错误')
                    pass
                else:
                    print(i, j, k)
                    with open('1.txt', "a", encoding="utf-8") as f:
                        f.write(str(i)+' '+str(j)+' '+str(k)+'\n'+str(J0)+' '+str(J1)+' '+str(J2)+' '+str(J3)+'\n')  # 括号内需要传递一个字符串
                    print('j0 j1 j2 j3 得出的值:', J0,J1,J2,J3)
    #servo3,servo4,servo5,servo6=transformAngelAdaptArm(J0,J1,J2,J3)
    #print("servo3:",servo3,"servo4:",servo4,"servo5:",servo5,"servo6:",servo6) 
    
    
    
         
     
