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

import rospy
import general_robotics_toolbox as rox
import numpy as np
from follow_joint_trajectory_action_adapter import FollowJointTrajectoryActionAdapter
from sensor_msgs.msg import JointState
import threading
from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    SetControllerMode, SetControllerModeRequest, SetControllerModeResponse
from rpi_arm_composites_manufacturing_abb_egm_controller.msg import ControllerState

def fill_joint_state_msg(joint_angles,now):
    js = JointState()        
    js.header.stamp = now
    js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']      
    js.position = joint_angles
    js.velocity = [0,0,0,0,0,0]
    js.effort = [0,0,0,0,0,0]
    return js


#TODO: implement safe teleoperation
class Controller(object):
    def __init__(self, robot, ts, max_joint_vel = None):        
        self._robot=robot
        self._ts=ts        
        self._command_joint_angles=None
        self._current_joint_angles=None
        self._error_msg=None
        self._mode=0
        self._speed_scalar=1
        self._ft_threshold=None
        self._ft=None
        if max_joint_vel is not None:
            self._max_joint_vel = max_joint_vel
        else:
            self._max_joint_vel = robot.joint_vel_limit
        
        self._lock=threading.Lock()
        self._joint_pub=rospy.Publisher('joint_states', JointState, queue_size=10)
        self._state_pub=rospy.Publisher('controller_state', ControllerState, queue_size=10)
        self._trajectory=FollowJointTrajectoryActionAdapter()
        self._mode_srv=rospy.Service('set_controller_mode', SetControllerMode, self._set_controller_mode_callback)
               
    def step(self, joint_angles, joint_cmd_vel, cmd_vel, trajectory_cmd_vel, cmd_halt, ft=None):
        with self._lock:
            now = rospy.Time.now()
            self._error_msg=None
            if self._current_joint_angles is None:
                self._current_joint_angles=np.copy(joint_angles)
            self._current_joint_angles[:]=joint_angles
            step_ts = self._ts * self._speed_scalar
            if self._command_joint_angles is None:
                self._command_joint_angles=np.copy(joint_angles)
            
            self._trajectory.current_joint_angles=self._current_joint_angles
            
            if cmd_halt:
                self._mode = 0
            
            if self._check_ft_threshold(ft):
                if self._mode == 0:
                    pass
                elif self._mode == 1:
                    if joint_cmd_vel is not None:
                        self._command_joint_angles += joint_cmd_vel.dot(step_ts)
                        self._clip_joint_angles()                    
                elif self._mode == 2:
                    if cmd_vel is not None:
                        J = rox.robotjacobian(self._robot, self._current_joint_angles)
                        joints_vel = np.linalg.pinv(J).dot(cmd_vel)
                        joints_vel = np.clip(joints_vel, -self._max_joint_vel, self._max_joint_vel)
                        self._command_joint_angles += joints_vel.dot(step_ts)
                        self._clip_joint_angles()
                elif self._mode == 3:
                    if trajectory_cmd_vel is not None:
                        res, command_joints1 = self._trajectory.increment_trajectory_time( \
                                                            step_ts * trajectory_cmd_vel)
                        if res: 
                            self._command_joint_angles = command_joints1
                elif self._mode == 4:
                    #TODO: Will this cause stuttering? Use absolute time increments?
                    res, command_joints1 = self._trajectory.increment_trajectory_time(step_ts)
                    if res:            
                        self._command_joint_angles = command_joints1
                else:
                    pass
            else:
                self._trajectory.abort_trajectory()
            
            
            joint_state_msg = fill_joint_state_msg(self._current_joint_angles,now)            
            self._joint_pub.publish(joint_state_msg)
            self._ft=ft
            
            self._publish_state(now)
            
            return self._command_joint_angles
    
    def error_step(self, error_msg):
        with self._lock:
            self._command_joint_angles=None
            self._error_msg=error_msg
            self._ft = None
            self._publish_state(rospy.Time.now())
    
    def _clip_joint_angles(self):
        if self._robot.joint_lower_limit is not None:
            self._command_joint_angles = np.clip(self._command_joint_angles, \
                self._robot.joint_lower_limit, self._robot.joint_upper_limit, \
                self._command_joint_angles)
    
    def _set_controller_mode_callback(self, req):
        with self._lock:
            ret=SetControllerModeResponse()
            ret.success=True
            if req.mode.mode < 0 or req.mode.mode > 4:
                ret.success=False
                return ret
            if req.speed_scalar < 0 or req.speed_scalar > 5:
                ret.success=False
                return ret
            ft = req.force_torque_stop_threshold
            if np.shape(ft) == (0,):
                pass
            elif np.shape(ft) != (6,):
                ret.success=False
                return ret
            else:
                if np.any(ft < 0):
                    ret.success=False
                    return ret
            self._mode = req.mode.mode
            self._speed_scalar=req.speed_scalar
            self._ft_threshold=np.array(req.force_torque_stop_threshold)
            return ret
        
    def _check_ft_threshold(self, ft):
        if ft is None:
            return True
        if self._ft_threshold is None:
            return True
        if np.shape(self._ft_threshold) != (6,):
            return True
        if np.all(self._ft_threshold < 1e-6):
            return True
        if np.any(np.logical_and((self._ft_threshold > 1e-6), (np.abs(ft) > self._ft_threshold))):
            return False        
        return True
    
    def _publish_state(self, now):
        s=ControllerState()
        s.header.stamp=now
        s.mode.mode=self._mode
        s.joint_name=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        s.joint_position=self._current_joint_angles if self._current_joint_angles is not None else []
        s.joint_command_position=self._command_joint_angles if self._command_joint_angles is not None else []
        if self._ft is not None:
            s.ft_wrench.torque.x=self._ft[0]
            s.ft_wrench.torque.y=self._ft[1]
            s.ft_wrench.torque.z=self._ft[2]
            s.ft_wrench.force.x=self._ft[3]
            s.ft_wrench.force.y=self._ft[4]
            s.ft_wrench.force.z=self._ft[5]
            s.ft_wrench_valid=True
        else:
            s.ft_wrench_valid=False
        s.trajectory_valid=self._trajectory.trajectory_valid
        s.trajectory_time=self._trajectory.trajectory_time
        s.trajectory_max_time=self._trajectory.trajectory_max_time
        s.error_msg = self._error_msg if self._error_msg is not None else ""
        self._state_pub.publish(s)

        