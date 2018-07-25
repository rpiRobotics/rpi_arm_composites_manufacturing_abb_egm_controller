# Copyright (c) 2017, Rensselaer Polytechnic Institute, Wason Technology LLC
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Rensselaer Polytechnic Institute, or Wason 
#       Technology LLC, nor the names of its contributors may be used to 
#       endorse or promote products derived from this software without 
#       specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import threading
import sys
import rospy
import numpy as np

import moveit_commander
import general_robotics_toolbox as rox
import general_robotics_toolbox.urdf as rox_urdf
import general_robotics_toolbox.ros_msg as rox_msg

from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    SetControllerMode, SetControllerModeRequest

from rpi_arm_composites_manufacturing_abb_egm_controller.msg import \
    ControllerMode, ControllerState

from geometry_msgs.msg import PoseStamped, Pose

class arm_composites_manufacturing_controller_commander(object):
    
    MODE_HALT = ControllerMode.MODE_HALT
    MODE_JOINT_TELEOP = ControllerMode.MODE_JOINT_TELEOP
    MODE_CARTESIAN_TELEOP = ControllerMode.MODE_CARTESIAN_TELEOP
    MODE_SHARED_TRAJECTORY = ControllerMode.MODE_SHARED_TRAJECTORY
    MODE_AUTO_TRAJECTORY = ControllerMode.MODE_AUTO_TRAJECTORY
    
    def __init__(self, arm_controller_ns = "", move_group = "move_group", goal_position_tolerance = 0.04, planning_time = 30, rox_robot = None ):
        
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.moveit_robot = moveit_commander.RobotCommander()
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        if isinstance(move_group, basestring):
            self.moveit_group = moveit_commander.MoveGroupCommander(move_group)
            self.moveit_group.set_goal_position_tolerance(goal_position_tolerance)
            self.moveit_group.allow_replanning(True)
            self.moveit_group.set_planning_time(planning_time)
        else:
            self.moveit_group = move_group
        
        if isinstance(rox_robot, rox.Robot):
            self.rox_robot = rox_robot
        elif isinstance(rox_robot, basestring):
            self.rox_robot = rox_urdf.robot_from_parameter_server(rox_robot)
        elif rox_robot is None:
            self.rox_robot = rox_urdf.robot_from_parameter_server()
        else:
            raise AssertionError("Invalid parameter type for rox_robot")
        
        set_controller_mode_name = rospy.names.ns_join(arm_controller_ns, "set_controller_mode")        
        self._set_controller_mode=rospy.ServiceProxy(set_controller_mode_name, SetControllerMode)
        self._arm_controller_ns = arm_controller_ns        
        
    def set_controller_mode(self, mode, speed_scalar=1.0, ft_threshold=[]):
        req=SetControllerModeRequest()
        req.mode.mode=mode
        req.speed_scalar=speed_scalar
        req.force_torque_stop_threshold=ft_threshold
        
        res=self._set_controller_mode(req)
        if (not res.success): raise Exception("Could not set controller mode")
        
    def subscribe_controller_state(self, cb):
        controller_state_name = rospy.names.ns_join(self.arm_controller_ns, "controller_state")
        return rospy.Subscriber(controller_state_name, ControllerState, cb)
    
    def compute_ik(self, pose, current_joint = None):
        
        if isinstance(pose, PoseStamped):
            pose=rox_msg.msg2transform(pose.pose)
        elif isinstance(pose, Pose):
            pose=rox_msg.msg2transform(pose)
               
        joint_targets=rox.robot6_sphericalwrist_invkin(self.rox_robot, pose)

        if current_joint is None:
            current_joint=self.moveit_group.get_current_joint_values()
            
        joint_target=None
        d_max=1e10
        for j in joint_targets:
            d=np.linalg.norm(j-current_joint)
            if d < d_max:
                d_max = d
                joint_target=np.copy(j)
        
        if (joint_target is None):
            raise Exception("Could not find target joint values")
        
        return joint_target
    
    def compute_fk(self, joint = None):
        
        if joint is None:
            joint=self.moveit_group.get_current_joint_values()
        
        return rox.fwdkin(self.rox_robot, joint)
    
    def get_current_joint_values(self):
        return self.moveit_group.get_current_joint_values()
    
    def get_current_pose_msg(self):
        return self.moveit_group.get_current_pose()
            
    def plan(self, pose_target):
        
        joint_target=self.compute_ik(pose_target)
        return self.plan_joint_target(joint_target)
    
    def plan_joint_target(self, joint_target):
        
        self.moveit_group.set_joint_value_target(joint_target)
        
        plan1 = self.moveit_group.plan()        
        cnt = 0
        while( (not plan1.joint_trajectory.points) and (cnt<3)):
            if rospy.is_shutdown():
                raise Exception("Node shutdown")            
            plan1 = self.moveit_group.plan()
            cnt = cnt+1
        
        if (not plan1.joint_trajectory.points):
            raise Exception("Planning failed")
        
        return plan1
    
    def execute(self, plan):
        if rospy.is_shutdown():
            raise Exception("Node shutdown")
        res = self.moveit_group.execute(plan)
        if not res:
            raise Exception("Path execution failed")
    
    def async_execute(self, plan, result_cb):
                
        self._async_execute_func(lambda: self.execute(plan), result_cb)   
    
    def plan_and_move(self, pose_target):
        plan1=self.plan(pose_target)
        self.execute(plan1)
    
    def plan_joint_target_and_move(self, joint_target):
        plan1=self.plan_joint_target(joint_target)
        self.execute(plan1)
    
    def async_plan_and_move(self, pose_target, result_cb):                
        
        self._async_execute_func(lambda: self.plan_and_move(pose_target), result_cb)             
    
    def async_plan_joint_target_and_move(self, joint_target, result_cb):                
        
        self._async_execute_func(lambda: self.plan_joint_target_and_move(joint_target), result_cb)             
    
    
    def compute_cartesian_path(self, pose_target, jump_threshold=0.01, eef_step=0.0, avoid_collisions=True):
        
        if isinstance(pose_target, PoseStamped):
            pose_target=pose_target.pose
        elif isinstance(pose_target,rox.Transform):
            pose_target=rox_msg.transform2pose_msg(pose_target)
            
        (path, fraction) = self.moveit_group.compute_cartesian_path([pose_target], jump_threshold,\
                                                                    eef_step, avoid_collisions)
        
        if (fraction < 0.9999):
            raise Exception("Could not compute cartesian path")
        
        return path
    
    def compute_cartesian_path_and_move(self, pose_target, jump_threshold=0.01, eef_step=0.0, avoid_collisions=True):
        plan=self.compute_cartesian_path(pose_target, jump_threshold, eef_step, avoid_collisions)
        self.execute(plan)
    
    def async_compute_cartesion_path_and_move(self, pose_target, result_cb, jump_threshold=0.01, eef_step=0.0, avoid_collisions=True):                
        
        self._async_execute_func(lambda: self.compute_cartesian_path_and_move(pose_target, jump_threshold, \
                                                                              eef_step, avoid_collisions), result_cb)             
        
    def _async_execute_func(self, f, result_cb):
        lock=threading.Lock()
        def t():
            with lock:
                exp=None                
                try:
                    f()
                except Exception as exp1:
                    exp=exp1
                    
                result_cb(exp)
        
        with lock:
            thread=threading.Thread(target = t)
            thread.daemon=True
            thread.start()
    
    def stop_move(self):
        self.moveit_group.stop()
