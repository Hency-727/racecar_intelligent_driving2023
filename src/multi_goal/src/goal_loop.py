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
import math
import tf.transformations as tf

#repeat the route use for test 
#功能:循环导航 
#csv要求 第一个点是起点下一个点，终点是起点 1230
class MultiGoals:
    def __init__(self, goalListX, goalListY, retry, map_frame, ListD):
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
            self.ListD = ListD
            self.kx = 0
            self.ky = 0
            self.gx = 0
            self.gy = 0
            
            self.flag = 1
            self.MIN_DISTANCE = 0.65  # min distance of the judge between the goal and odometrypose
            self.MIN_ANGLE = 0.5

            self.LONG = len(self.goalListX)
            self.goalId = 0
            self.count = 0
            self.start_time = 0
            self.pubfinal = False
            self.goalMsg = PoseStamped()
            self.goalMsg.header.frame_id = map_frame

            
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
        if self.pubfinal == True:
            self.pub_final.publish(1.0)
            print("the final goal")

        self.gx = self.goalListX[self.goalId-1] if(self.goalId != 0) else self.goalListX[self.goalId]
        self.gy = self.goalListY[self.goalId-1] if(self.goalId != 0) else self.goalListY[self.goalId]

        self.dist = self.distance(self.kx, self.ky, self.gx, self.gy)
        self.angle_diff = self.get_angle(self.kx, self.ky, self.gx, self.gy)

        if self.dist < self.MIN_DISTANCE and self.angel_diff < self.MIN_ANGLE:

            finish_time = rospy.get_time()
            interval = finish_time - self.start_time
            print(interval)
            if self.goalId == self.LONG:
                self.goalId = 0
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            
                
            rospy.loginfo(
                "Initial goal published! Goal ID is: %d", self.goalId)
            rospy.loginfo("intostatusCB")
            self.count = self.count+1
            print(self.count)

            if self.count < (len(self.goalListX)):
                self.goalId = self.goalId + 1
                
                #目标点
                if self.ListD == 0.0:
                    #publish
                    self.pub.publish(self.goalMsg)
                    #这里要插入函数，执行的功能为停止并拍照
                    self.take_photo()
                    time.sleep(1)
                #过渡点
                elif:
                    self.ListD == 1:

                #停车点
                else:

                    time.sleep(1)

            else:
                self.goalId = len(self.goalListX)

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

    def get_angle(self, kx, ky, gx, gy):
        # 将四元数转换为欧拉角
        _, _, yaw = tf.euler_from_quaternion([0, 0, z, w])

        # 计算目标点与车辆当前位姿之间的夹角
        angle = math.atan2(kx - gx, ky - gy) - yaw

        # 将夹角转换为[-pi, pi]范围内的值
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi

        return angle



    #识别点函数
    def takephoto(self)
        if  self.ListD == 0
            return True
        else
            return False

    #停车函数
    def




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
        List=[]
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

        
        with open('dection.csv', 'r') as f:
            reader = csv.reader(f)
            for cols in reader:
                List.append([int(value) for value in cols])
            List = np.array(List)
            print("read suc!!")
            ListD = List[:,0]


        if len(goalListX) == len(goalListY) & len(goalListY) >= 1:
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalListX, goalListY, retry, map_frame, goalListD)
            rospy.spin()
        else:
            rospy.loginfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")


