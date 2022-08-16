

覃俊佳
import subprocess

flag_fpga = 1
def control(pwm1,pwm2,pwm3,angle1,angle2,fpga):
    # 全是int,fpga是bool,1(True)触发
    global flag_fpga
    massage = '"{pwm1：'
    massage += str(pwm1)
    massage += ','
    massage += 'pwm2：'
    massage += str(pwm2)
    massage += ','
    massage += 'pwm3：'
    massage += str(pwm3)
    massage += ','
    massage += 'angle1：'
    massage += str(angle1)
    massage += ','
    massage += 'angle2：'
    massage += str(angle2)
    massage += ','
    massage += 'fpga：'
    if fpga == 1:
        massage += str(flag_fpga+1)
        flag_fpga += 1
    else :
        massage += str(flag_fpga)
    massage += '}"'
    print(massage)
    subprocess.call(['rosservice','call','/servo_ctrl',massage])

control(1,2,3,4,5,1)
更多
import sys,os
import time
import socket
from numpy import *
import subprocess

pwm1 = 0
pwm2 = 0
pwm3 = 0
angle1 = 0
angle2 = 0
flag_fpga = 0

def control():
    global pwm1,pwm2,pwm3,angle1,angle2,flag_fpga
    massage = '"{pwm1：' + str(pwm1) + ',' + 'pwm2：' + str(pwm2) + ',' + 'pwm3：'+ str(pwm3) + ','
    massage += 'angle1：' + str(angle1) + ',' + 'angle2：' + str(angle2) + ','
    massage += 'fpga：'
    if flag_fpga == 1:
        massage += str(flag_fpga+1)
        flag_fpga += 1
    else:
        massage += str(flag_fpga)
    massage += '}"'
    print(massage)
    subprocess.call(['rosservice','call','/servo_ctrl',massage])

def start():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_addr = ("127.0.0.1", 8080)
    udp_socket.bind(local_addr)
    while True:
        recv_data = udp_socket.recvfrom(1024)
        print(recv_data.decode("utf-8"))

if __name__ == "__main__":
    start()
​​