#!/usr/bin/python
# -*- coding: UTF-8 -*-

import re,socket               # 导入 socket 模块
import json
import sys,time
sys.path.append("/home/smalllogo/tracking_ws/src/uav_follow_robot/scripts/1209/PyIT2FLS-master/PyIT2FLS-master/examples")
from new_v1 import IT2FL_v1_fun
from new_v2 import IT2FL_v2_fun

    
def tcpServer():
    # TCP服务
    # with socket.socket() as s:
    ip_port=('127.0.0.1',8082)
    BUFSIZE=1024
    s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 绑定服务器地址和端口
    s.bind(ip_port)
    # 启动服务监听                                                                                                                                                                                                                                                                                                                                            
    s.listen(5)
    print('Waitting for connnecting')
    while True:
        # 等待客户端连接请求,获取connSock
        conn, addr = s.accept()
        print('{} Connected!'.format(addr))
        # print("conn:",conn)
        while True:
            # 接收请求信息
            json_data = conn.recv(BUFSIZE)
            # print ("ini_rev:",json_data)
            # print("json_data type:",type(json_data))
            rev_data=json.loads(str(json_data, encoding="utf-8"))
            # print('rev_data:',rev_data)
            print("rev_center:",rev_data)
            # time.sleep(1)

            # w=752
            # h=480
            w=480
            h=300
            y=(rev_data[1]-h/2)/(h/2)
            v1=IT2FL_v1_fun(y)

            x=(int(rev_data[0])-w/2)/(w/2)
            yaw_des=IT2FL_v2_fun(x)
            # if x > -0.3 and  x < 0.3: 
            #      yaw_des=0
            # else: 
            #     yaw_des=IT2FL_v2_fun(x)
            #     print("v2_input:",yaw_des)
            #     # print("Published:",v1,yaw_des)

            send_data=[v1,yaw_des]
            json_send_data=json.dumps(send_data)
            conn.send(json_send_data.encode('utf-8'))
            print("send_con:",send_data)
            # time.sleep(1)

                       
                       
    s.close()

def main():
    print("start")
    tcpServer()

if __name__ == '__main__':
    # rospy.init_node("riseq_rotors_waypoint_publisher", anonymous = True)
    main()
 
