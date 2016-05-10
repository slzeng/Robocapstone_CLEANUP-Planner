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
import eliminator.msg

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

class Drill_Task(Task):
    """docstring for Follow_Trajectory_Task"""
    def __init__(self,client):
        self.priority = 3;
        self.eliminator_client = client

    def action(self):
        goal = eliminator.msg.eliminationGoal()
        goal.start = True;
        self.eliminator_client.send_goal(goal)
            
            

class Task_Manager(object):
    """docstring for Sim"""
    def __init__(self):
        rospy.init_node('task_managers', anonymous=True)
        
        # rospy.Subscriber("/collision", bool, self.bumper_callback)
        
        self.boundary_priority = 1;
        self.weed_priority = 2;

        self.boundary_triggered = 0;
        self.current_priority = 4;
        self.current_task = -1;

        self.weed_target = None;
        self.low_res_target = None;
        self.task_queue = Queue.PriorityQueue();
        self.action_in_proccess = False;
        self.planner_client = actionlib.SimpleActionClient('planner', planner.msg.gen_trajAction)
        self.eliminator_client = actionlib.SimpleActionClient("eliminate", eliminator.msg.eliminationAction)
        print("Waiting for planner server")
        self.planner_client.wait_for_server();
        print("Waiting for eliminator server")
        self.eliminator_client.wait_for_server();

        self.cmd_pub = rospy.Publisher("/cmd",Pose2D,queue_size=0);
        self.detection_level_pub = rospy.Publisher("/detection_level", String,queue_size=0);
        self.elim_pub = rospy.Publisher("/eliminate",String,queue_size = 0)
        rospy.Subscriber("/planner/result", planner.msg.gen_trajActionResult, self.planner_complete_callback)
        rospy.Subscriber("/eliminate/result", eliminator.msg.eliminationActionResult, self.drill_callback)
        rospy.Subscriber("/boundary", Int32, self.boundary_callback)
        rospy.Subscriber("/weed_detection", Pose2D, self.low_res_detection_callback)
        rospy.Subscriber("/weed_location",Pose2D,self.high_res_detection_callback)
        
        self.current_task = 'Finding weed'

        self.detection_level = "low";
        self.detection_level_pub.publish(self.detection_level)
        #start task is follow trajectory
        start = planner.msg.gen_trajActionResult();
        start.result.success = 1;
        self.planner_complete_callback(start);

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.detection_level_pub.publish(self.detection_level)
            if(not self.task_queue.empty() and not self.action_in_proccess):
                self.action_in_proccess = True;
                (self.current_priority,obj) = self.task_queue.get_nowait()
                obj.action()
            
            print(self.current_task)
            rate.sleep()
            
            
            # print("waiting")
            # print(client.get_result());
            # print("DONE!")



    def planner_complete_callback(self, data):
        self.action_in_proccess = False

        if(data.result.success == 1):
            # goal = planner.msg.gen_trajGoal()
            # goal.x = random.uniform(.5,5)
            # goal.y = random.uniform(.1,3)*((-1)**random.randint(0,1))
            # goal.theta = 0
            # goal.frame = "robot"
            # t = Follow_Trajectory_Task(goal,self.planner_client)
            # print("REQUESTING NEW PLANNER RANDOM ACTION: Plan to %f,%f" % (goal.x, goal.y))
            # self.task_queue.put((t.priority,t))
            self.boundary_triggered = 0;

            self.detection_level = 'high'
            self.detection_level_pub.publish(self.detection_level)
            d = rospy.Duration(5)
            rospy.sleep(d)
        else:
            print("FAILED GOAL")
        self.low_res_target = None
        
        self.detection_level = 'low'
        self.detection_level_pub.publish(self.detection_level)
        go_msg = Pose2D(.1,0,0);
        self.cmd_pub.publish(go_msg)
        print('planner complete')
        self.current_task = 'Finding weed'


    def drill_callback(self,data):
        self.current_task = 'Drilling'

        print("DRILLING")
        rospy.sleep(2)
        print("DONE")
        self.elim_pub.publish("DRILL HERE")
        self.weed_target = None
        self.detection_level = 'low'
        self.action_in_proccess = False
        self.low_res_target = None



    def boundary_callback(self,data):
        self.current_task = 'Boundary Callback???!?!?!?'
        if(data.data == 1 and self.boundary_triggered == 0 and self.weed_target == None):
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
            self.weed_target = None

    def low_res_detection_callback(self,data):
        
        if(self.weed_target == None):
            self.current_task = 'Low Res Scaning'
    
        #     self.low_res_target = data;
        #     self.planner_client.cancel_all_goals()

        #     goal = planner.msg.gen_trajGoal()
        #     goal.x = self.low_res_target.x
        #     goal.y = self.low_res_target.y
        #     goal.theta = 0
        #     goal.frame = "robot"
        #     print("ADDING Reposition TASK: Plan to %f,%f" % (goal.x, goal.y))
        #     t = Follow_Trajectory_Task(goal,self.planner_client)
        #     self.task_queue.put_nowait((self.weed_priority,t))

            self.detection_level = 'high'
            self.detection_level_pub.publish(self.detection_level)
      

    def high_res_detection_callback(self,data):
        if(self.weed_target == None):
            self.current_task = 'High Res Scaning'

            self.weed_target = data;
            self.planner_client.cancel_all_goals()

            goal = planner.msg.gen_trajGoal()
            goal.x = self.weed_target.x
            goal.y = self.weed_target.y
            goal.theta = 0
            goal.frame = "robot"
            print("ADDING WEED TASK: Plan to %f,%f" % (goal.x, goal.y))
            t = Follow_Trajectory_Task(goal,self.planner_client)
            self.task_queue.put_nowait((self.weed_priority,t))
            self.detection_level = 'wait'
            self.detection_level_pub.publish(self.detection_level)
      

            t = Drill_Task(self.eliminator_client)
            self.task_queue.put_nowait((t.priority,t))

if __name__ == '__main__':
    tm = Task_Manager();