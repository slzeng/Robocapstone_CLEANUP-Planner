#!/usr/bin/env python
import roslib;
roslib.load_manifest('planner')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import tf
import math
import Queue
import actionlib
import random
# from planner.msg import gen_trajAction, gen_trajGoal
import planner.msg
from actionlib_msgs.msg import GoalID


class Task(object):
    
    def __init__(self,goal):
        self.priority = 1000;
        self.goal = goal
    def action(self):
        pass


class Follow_Trajectory_Task(Task):
    """docstring for Follow_Trajectory_Task"""
    def __init__(self,goal,client):
        super(Follow_Trajectory_Task, self).__init__(goal)
        self.priority = 4;
        self.goal = goal
        self.planner_client = client

    def action(self):
        self.planner_client.send_goal(self.goal)
            

class Task_Manager(object):
    """docstring for Sim"""
    def __init__(self):
        rospy.init_node('task_managers', anonymous=True)
        rospy.Subscriber("/planner/result", planner.msg.gen_trajActionResult, self.planner_complete_callback)
        # rospy.Subscriber("/drill_cycle", bool, self.drill_callback)
        rospy.Subscriber("/boundary", Int32, self.boundary_callback)
        rospy.Subscriber("/weed_location", Pose2D, self.detection_callback)
        # rospy.Subscriber("/collision", bool, self.bumper_callback)
        
        self.boundary_priority = 2;
        self.weed_priority = 3;

        self.boundary_triggered = 0;
        self.current_priority = 4;
        self.current_task = -1;

        self.weed_target = None;
        self.task_queue = Queue.PriorityQueue();

        self.planner_client = actionlib.SimpleActionClient('planner', planner.msg.gen_trajAction)
        print("Waiting for planner server")
        self.planner_client.wait_for_server();


        #start task is follow trajectory
        start = planner.msg.gen_trajActionResult();
        start.result.success = 1;
        self.planner_complete_callback(start);

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print(self.task_queue.qsize())
            if(not self.task_queue.empty()):
                (self.current_priority,obj) = self.task_queue.get_nowait()
                obj.action()
            

            rate.sleep()
            
            
            # print("waiting")
            # print(client.get_result());
            # print("DONE!")



    def planner_complete_callback(self, data):
        if(data.result.success == 1):
            print("REQUESTING NEW PLANNER RANDOM ACTION")
            goal = planner.msg.gen_trajGoal()
            goal.x = random.uniform(.5,5)*((1)**random.randint(0,1))
            goal.y = random.uniform(.1,3)*((1)**random.randint(0,1))
            goal.theta = 0
            goal.frame = "robot"
            t = Follow_Trajectory_Task(goal,self.planner_client)
            self.task_queue.put((t.priority,t))
            self.boundary_triggered = 0;
            self.weed_target = None
        else:
            print("FAILED GOAL")

    def drill_callback(self,data):
        print("DRILLING")

    def boundary_callback(self,data):
        if(data.data == 1 and self.boundary_triggered==0):
            self.planner_client.cancel_all_goals()
            self.boundary_triggered = 1;
            print("CANCELING AND REQUESTING PLANNER REVERSE ACTION")
            goal = planner.msg.gen_trajGoal()
            goal.x = random.uniform(1,3)*((-1))
            goal.y = random.uniform(.5,2)*((-1))
            goal.theta = 0
            goal.frame = "robot"
            t = Follow_Trajectory_Task(goal,self.planner_client)
            self.task_queue.put_nowait((self.boundary_priority,t))
        if(self.boundary_triggered and data.data == 0):
            print("Boundary Reset")
            self.boundary_triggered = 0;


    def detection_callback(self,data):
        print(data)
        if(self.weed_target == None):
            self.weed_target = data;
            self.planner_client.cancel_all_goals()
            goal = planner.msg.gen_trajGoal()
            goal.x = self.weed_target.x
            goal.y = self.weed_target.y
            goal.theta = 0
            goal.frame = "world"
            t = Follow_Trajectory_Task(goal,self.planner_client)
            self.task_queue.put_nowait((self.weed_priority,t))

if __name__ == '__main__':
    tm = Task_Manager();