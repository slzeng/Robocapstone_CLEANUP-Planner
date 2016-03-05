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
        # rospy.Subscriber("/collision", bool, self.bumper_callback)
        
        self.boundary_priority = 3;

        self.current_task = -1;
        
        self.task_queue = Queue.PriorityQueue();

        self.planner_client = actionlib.SimpleActionClient('planner', planner.msg.gen_trajAction)
        print("Waiting for planner server")
        self.planner_client.wait_for_server();


        #start task is follow trajectory
        self.planner_complete_callback(None);

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if(not self.task_queue.empty()):
                (self.current_priority,obj) = self.task_queue.get_nowait()
                obj.action()
            

            rate.sleep()
            
            
            # print("waiting")
            # print(client.get_result());
            # print("DONE!")



    def planner_complete_callback(self, data):
        print("REQUESTING NEW PLANNER RANDOM ACTION")
        goal = planner.msg.gen_trajGoal()
        goal.x = random.uniform(.5,3)*((-1)**random.randint(0,1))
        goal.y = random.uniform(.5,3)*((-1)**random.randint(0,1))
        goal.theta = 0
        t = Follow_Trajectory_Task(goal,self.planner_client)
        self.task_queue.put((t.priority,t))

    def drill_callback(self,data):
        # drill complete
        pass

    def boundary_callback(self,data):
        print(self.boundary_priority,self.current_priority)
        print("BOUNDARY CALLBACK")
        if(self.boundary_priority<self.current_priority):
            cancel_msg = GoalID()
            self.planner_client.publish(cancel_msg)
            
            print("CANCELING AND REQUESTING PLANNER REVERSE ACTION")
            goal = planner.msg.gen_trajGoal()
            goal.x = random.uniform(.5,3)*((-1))
            goal.y = random.uniform(.5,3)*((-1))
            goal.theta = 0
            t = Follow_Trajectory_Task(goal,self.planner_client)
            self.task_queue.put((t.priority,t))


    def bumper_callback(self,data):
        # stop: collision
        pass

    def detection_callback(self,data):
        pass

if __name__ == '__main__':
    tm = Task_Manager();