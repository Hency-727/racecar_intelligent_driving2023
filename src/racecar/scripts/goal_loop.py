#coding:utf-8
from geometry_msgs.msg import PoseStamped, Pose
#!/usr/bin/env python
import rospy
import string
import math
import time
import sys
import csv
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

#repeat the route use for test 
#功能:循环导航 
#csv要求 第一个点是起点下一个点，终点是起点 1230
class MultiGoals:
    def __init__(self, goalListX, goalListY, retry, map_frame):
        self.retry = 1
        if(self.retry == 1):
            #self.sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=1)
            self.pub = rospy.Publisher(
                'move_base_simple/goal', PoseStamped, queue_size=10)
            #self.pose_ekf = rospy.Subscriber('/odometry/filtered',Odometry,self.getPose_ekf,queue_size=10)
            self.pose_amcl = rospy.Subscriber(
                "/amcl_pose", PoseWithCovarianceStamped, self.getPose_amcl, queue_size=10)
            # params & variables
            self.pub_final = rospy.Publisher('/arrfinal', Float64, queue_size=1)
            self.goalListX = goalListX
            self.goalListY = goalListY
            self.goalListW = goalListW
            self.goalListZ = goalListZ
            self.kx = 0
            self.ky = 0
            self.gx = 0
            self.gy = 0
            self.flag = 1
            self.MIN_DISTANCE = 5  # min distance of the judge between the goal and odometrypose
            self.LONG = len(self.goalListX)
            self.goalId = 0
            self.count = 0
            self.start_time = 0
            self.pubfinal = False
            self.isGetImg = False
            
            self.goalMsg = PoseStamped()
            self.goalMsg.header.frame_id = map_frame

            time.sleep(1)
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.pub.publish(self.goalMsg)
            # print(self.goalMsg)
            self.start_time = rospy.get_time()
            rospy.loginfo(
                "Initial goal published! Goal ID is: %d", self.goalId)

            self.goalId = self.goalId + 1

    def statusCB(self):
        #判断是否到达终点
        if self.pubfinal == True:
            self.pub_final.publish(1.0)
            print("the final goal")

        self.gx = self.goalListX[self.goalId-1] if(self.goalId != 0) else self.goalListX[self.goalId]
        self.gy = self.goalListY[self.goalId-1] if(self.goalId != 0) else self.goalListY[self.goalId]
        
        #计算机器人此时位姿与目标点的距离关系
        self.dist = self.distance(self.kx, self.ky, self.gx, self.gy)
        #rospy.loginfo(
        #        "-------------------------------------------------------------------- is: %d", self.dist)

        #距离小于一个阈值时进入        
        if self.dist < self.MIN_DISTANCE and :
            
            #计算一个点到另外一个点经过的时间/间隔时间
            finish_time = rospy.get_time()
            interval = finish_time - self.start_time
            print(interval)

            self.isGetImg = True
            if not self.istarget and  self.isGetImg:
                print('imgupdate')
                # cv.imwrite('/home/ucar/ucar_ws/src/yolo/images/'+str(self.goal_num)+'.jpg', undistort(self.img))
                cv.imwrite('/home/ucar/ucar_ws_zx/src/photos/images/'+str(self.goal_num)+'.jpg', self.img)
                print("goal is reaching and image "+str(self.goal_num)+" is downloading")
                self.goal_num+=1
                self.isGetImg=False

            #如果goalid迭代到数组长度，设置为0？ 起点？
            if self.goalId == self.LONG:
                self.goalId = 0
            
            #更新此时位姿和时间戳
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]

            #
            if self.flag == 1:
                self.pub.publish(self.goalMsg)
                time.sleep(interval)


            rospy.loginfo(
                "Initial goal published! Goal ID is: %d", self.goalId)
            #rospy.loginfo(
            #    "-----------------------woshishabi222222222222222222222222--------------------")                
            rospy.loginfo("intostatusCB")
            
            #计数
            self.count = self.count+1
            print(self.count)

            if self.goalId < (len(self.goalListX)) and self.goalId %3==0 :
                self.goalId = self.goalId + 1

            else:
                self.goalId = (len(self.goalListX)):
                self.flag = 0
                self.istarget = True 
                print("final")


    def getPose_ekf(self, data):
        self.kx = data.pose.pose.position.x
        self.ky = data.pose.pose.position.y
        self.statusCB()

    def getPose_amcl(self, data):
        self.kx = data.pose.pose.position.x
        self.ky = data.pose.pose.position.y
        self.statusCB()

    def distance(self, kx, ky, gx, gy):
        try:
            return math.sqrt((kx-gx)**2+(ky-gy)**2)
        except:
            return None



if __name__ == "__main__":
    try:
        # ROS Init
        rospy.init_node('multi_goals', anonymous=True)
        retry = 1
        goalList = []
        goalListX=[]
        goalListY=[]
        goalListZ=[]
        goalListW=[]
        map_frame = rospy.get_param('~map_frame', 'map' )

        with open('test.csv', 'r') as f:
            reader = csv.reader(f)

            for cols in reader:
                goalList.append([float(value) for value in cols])

            goalList = np.array(goalList)
            print("read suc!!")
            goalListX = goalList[:,0]
            goalListY = goalList[:,1]
            goalListZ = goalList[:,2]
            goalListW = goalList[:,3]


        if len(goalListX) == len(goalListY) & len(goalListY) >= 1:
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalListX, goalListY, retry, map_frame)
            rospy.spin()
        else:
            rospy.loginfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")


