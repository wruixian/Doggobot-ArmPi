#!/usr/bin/python3
# coding=utf8
# Date:2021/04/20
# Author:Aiden
import sys
import cv2
import math
import rospy
import threading
import numpy as np
from threading import Timer

from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image

from warehouse.srv import *
from warehouse.msg import Grasp
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, JointState

from kinematics import ik_transform
from armpi_fpv import bus_servo_control

# 转仓
# 如未声明，使用的长度，距离单位均为m

# 初始化
__target_data = ()
__isRunning = False
org_image_sub_ed = False
lock = threading.RLock()
ik = ik_transform.ArmIK()

# 初始位置
def initMove():
    with lock:
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    rospy.sleep(2)

# 变量重置
def reset():
    global __target_data
    
    __target_data = ()   

# app初始化调用
def init():
    rospy.loginfo("exchange Init")
    initMove()
    reset()

def pick(grasps):
    position = grasps.grasp_pos.position
    rotation = grasps.grasp_pos.rotation
    approach = grasps.grasp_approach
    retreat = grasps.grasp_retreat
    
    # 计算是否能够到达目标位置，如果不能够到达，返回False
    target1 = ik.setPitchRanges((position.x + approach.x, position.y + approach.y, position.z + approach.z), rotation.r, -180, 0)
    target2 = ik.setPitchRanges((position.x, position.y, position.z), rotation.r, -180, 0)
    target3 = ik.setPitchRanges((position.x, position.y, position.z + grasps.up), rotation.r, -180, 0)
    target4 = ik.setPitchRanges((position.x + retreat.x, position.y + retreat.y, position.z + retreat.z), rotation.r, -180, 0)

    if not __isRunning:
        return False 
    if target1 and target2 and target3 and target4:
        # 第一步：云台转到朝向目标方向，夹持器打开
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 800, ((1, grasps.pre_grasp_posture), (2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data['servo6'])))
        rospy.sleep(0.8)
        if not __isRunning:
            return False
        
        # 第二步：移到接近点
        bus_servo_control.set_servos(joints_pub, 500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
        rospy.sleep(0.5)
        if not __isRunning:
            return False
        
        # 第三步：移到目标点
        servo_data = target2[1]
        bus_servo_control.set_servos(joints_pub, 500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
        rospy.sleep(1)
        if not __isRunning:
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
            rospy.sleep(1)            
            return False
        
        # 第四步：夹取
        bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.grasp_posture), ))               
        rospy.sleep(1)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture), ))               
            rospy.sleep(0.5)            
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
            rospy.sleep(1)             
            return False
        
        # 第五步：抬升
        servo_data = target3[1]
        if grasps.up != 0:
            bus_servo_control.set_servos(joints_pub, 400, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(0.5)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture), ))               
            rospy.sleep(0.5)             
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
            rospy.sleep(1)              
            return False
        
        # 第六步：移到撤离点
        servo_data = target4[1]
        if servo_data != target3[1]:            
            bus_servo_control.set_servos(joints_pub, 500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))        
            rospy.sleep(0.5)
            if not __isRunning:
                bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture), ))               
                rospy.sleep(0.5)                 
                return False
            
        # 第七步：移到稳定点
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 500, ((2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data['servo6'])))
        rospy.sleep(0.5)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture), ))               
            rospy.sleep(0.5)             
            return False
        
        return True
    else:
        rospy.loginfo('pick failed')
        return False

def place(places):
    position = places.grasp_pos.position
    rotation = places.grasp_pos.rotation
    approach = places.grasp_approach
    retreat = places.grasp_retreat
    
    # 计算是否能够到达目标位置，如果不能够到达，返回False
    target1 = ik.setPitchRanges((position.x + approach.x, position.y + approach.y, position.z + approach.z), rotation.r, -180, 0)
    target2 = ik.setPitchRanges((position.x, position.y, position.z), rotation.r, -180, 0)
    target3 = ik.setPitchRanges((position.x, position.y, position.z + places.up), rotation.r, -180, 0)
    target4 = ik.setPitchRanges((position.x + retreat.x, position.y + retreat.y, position.z + retreat.z), rotation.r, -180, 0)

    if not __isRunning:
        return False
    if target1 and target2 and target3 and target4:
        # 第一步：云台转到朝向目标方向
        servo_data = target1[1]
        if __target_data[0][0] != __target_data[1][0]:
            bus_servo_control.set_servos(joints_pub, 2000, ((1, places.pre_grasp_posture), (2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data['servo6'])))
            rospy.sleep(2)
        rospy.sleep(0.5)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))               
            rospy.sleep(0.5)            
            return False
        
        # 第二步：移到接近点
        bus_servo_control.set_servos(joints_pub, 500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))      
        rospy.sleep(0.5)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))               
            rospy.sleep(0.5)            
            return False
        
        # 第三步：移到目标点
        servo_data = target2[1]
        if servo_data != target1[1]:        
            bus_servo_control.set_servos(joints_pub, 500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6']))) 
            rospy.sleep(1)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))               
            rospy.sleep(0.5)            
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
            rospy.sleep(1)              
            return False
        
        # 第四步：放置
        bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))         
        rospy.sleep(1)
        if not __isRunning:
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
            rospy.sleep(1)              
            return False
        
        # 第五步：抬升
        servo_data = target3[1]
        if places.up != 0:
            bus_servo_control.set_servos(joints_pub, 400, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(0.5)
        if not __isRunning:
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
            rospy.sleep(1)              
            return False
        
        # 第六步：移到撤离点
        servo_data = target4[1]
        if servo_data != target3[1]:
            bus_servo_control.set_servos(joints_pub, 500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(0.5)
            if not __isRunning:
                return False
            
        # 第七步：移到稳定点
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 500, ((2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data['servo6'])))
        rospy.sleep(0.5)
        if not __isRunning:
            return False
        
        return True
    else:
        rospy.loginfo('place failed')
        return False

##########################################
## 货架每层位置x, y, z(m)
shelf_position = {'R1':[0.277, 0, 0.02],
                  'R2':[0.277, 0, 0.12],
                  'R3':[0.277, 0, 0.21],
                  'L1':[-0.277, 0, 0.02],
                  'L2':[-0.277, 0, 0.12],
                  'L3':[-0.277, 0, 0.21]}
##########################################

# 每层放置时的俯仰角
roll_dict = {'R1': -130,
             'R2': -120,
             'R3': -90,
             'L1': -130,
             'L2': -120,
             'L3': -90}

def move():
    global __target_data
    
    while True:
        if __isRunning:
            try:
                if len(__target_data) == 2:                                    
                    move_out = __target_data[0]
                    if shelf_position[move_out][0] > 0:
                        approach_x = -0.07
                    else:
                        approach_x = 0.07
                    grasps = Grasp()                    
                    grasps.grasp_pos.position.x = shelf_position[move_out][0]
                    grasps.grasp_pos.position.y = shelf_position[move_out][1]
                    grasps.grasp_pos.position.z = shelf_position[move_out][2]
                    grasps.grasp_pos.rotation.r = roll_dict[move_out]
                    grasps.up = 0
                    grasps.grasp_approach.x = approach_x
                    grasps.grasp_retreat.x = approach_x
                    grasps.grasp_retreat.z = 0.02
                    grasps.grasp_posture = 450
                    grasps.pre_grasp_posture = 75 
                    buzzer_pub.publish(0.1)
                    result = pick(grasps)                          
                    if result:
                        move_in = __target_data[1]
                        if shelf_position[move_in][0] > 0:
                            approach_x = -0.07
                        else:
                            approach_x = 0.07
                        places = Grasp()                    
                        places.grasp_pos.position.x = shelf_position[move_in][0]
                        places.grasp_pos.position.y = shelf_position[move_in][1]
                        places.grasp_pos.position.z = shelf_position[move_in][2]
                        places.grasp_pos.rotation.r = roll_dict[move_in]
                        places.up = 0.0
                        places.grasp_approach.x = approach_x
                        places.grasp_approach.z = 0.04
                        places.grasp_retreat.x = approach_x
                        places.grasp_retreat.z = 0.02
                        places.grasp_posture = 75
                        places.pre_grasp_posture = 450                        
                        place(places)                    
                            
                    __target_data = []
                    initMove()
                else:
                    rospy.sleep(0.01)
            except BaseException as e:
                rospy.loginfo("pick and place error:%s", e)
        else:
            rospy.sleep(0.01)
            
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 将ros发布的图像转化成opencv能够处理的格式，并且将处理后的图像发布出去
def image_callback(ros_image):  
    global lock
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    frame = cv2_img.copy()
    frame_result = frame
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)
            
def enter_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed
    
    rospy.loginfo("enter exchange")
    with lock:
        init()
        if not org_image_sub_ed:
            org_image_sub_ed = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
        
    return [True, 'enter']

heartbeat_timer = None
def exit_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed
    
    rospy.loginfo("exit exchange")
    with lock:
        __isRunning = False
        try:
            if org_image_sub_ed:
                org_image_sub_ed = False
                heartbeat_timer.cancel()
                image_sub.unregister()
        except:
            pass
        
    return [True, 'exit']

def start_running():
    global lock
    global __isRunning
    
    rospy.loginfo("start running exchange")
    with lock:
        __isRunning = True

def stop_running():
    global lock
    global __isRunning
    
    rospy.loginfo("stop running exchange")
    with lock:
        __isRunning = False
        reset()

def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()
        
    return [True, 'set_running']

def set_target(msg):
    global lock
    global __target_data
    
    rospy.loginfo('%s', msg)
    with lock:
        __target_data = msg.position

    return [True, 'set_target']

def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/exchange/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('exchange', log_level=rospy.DEBUG)
    
    # 舵机发布
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    
    # 图像发布
    image_pub = rospy.Publisher('/exchange/image_result', Image, queue_size=1)
    
    # app通信服务
    enter_srv = rospy.Service('/exchange/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/exchange/exit', Trigger, exit_func)
    running_srv = rospy.Service('/exchange/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/exchange/set_target', SetExchangeTarget, set_target)
    heartbeat_srv = rospy.Service('/exchange/heartbeat', SetBool, heartbeat_srv_cb)
     
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    rospy.sleep(0.2) # pub之后必须延时才能生效
    
    initMove()
    buzzer_pub.publish(0.1)
    
    debug = False
    if debug:
        enter_func(1)
        
        msg = SetExchangeTarget()
        msg.position = ['R1', 'R3']
        set_target(msg)
        
        start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
