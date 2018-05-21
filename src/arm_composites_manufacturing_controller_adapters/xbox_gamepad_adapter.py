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

from sensor_msgs.msg import Joy
import rospy
import numpy as np
import threading

class XboxGamepadAdapter(object):
    
    def __init__(self):
        self._lock=threading.Lock()        
        self._joy_subscription=rospy.Subscriber("joy", Joy, self._joy_callback)
        
        self._cmd_vel=None
        self._cmd_vel_time=rospy.Time.from_sec(0)        
        self._joint_vel=None
        self._joint_vel_time=rospy.Time.from_sec(0)
        self._trajectory_vel=None
        self._trajectory_vel_time=rospy.Time.from_sec(0)
        
        self._joint_vel_gain=np.array([-0.5, 1, 1, 1, 1, -1])*np.deg2rad(5)
        self._cmd_vel_gain=np.array([-0.15, 0.15, 0.05, 0.087, 0.175, 0.087])
        self._cmd_halt=False
    
    def _joy_callback(self, data):
        with self._lock:            
            if any(data.buttons[0:4]):
                #Trajectory shared control
                if data.buttons[0] != 0:
                    self._trajectory_vel = 0.25 * data.axes[1]                
                elif data.buttons[1] != 0:
                    self._trajectory_vel = data.axes[1]                
                elif data.buttons[2] != 0:
                    self._trajectory_vel = 0.25                
                elif data.buttons[3] != 0:
                    self._trajectory_vel = 1
                self._trajectory_vel_time=rospy.Time.now()
                self._joint_vel=None
                self._cmd_vel=None                                
                               
            elif data.buttons[4] != 0:
                #Joint level teleop control
                self._joint_vel = np.array([data.axes[0], data.axes[1], data.axes[4], \
                                          data.axes[6], data.axes[7], data.axes[3]]) \
                                          *self._joint_vel_gain
                self._joint_vel_time=rospy.Time.now()
                self._trajectory_vel=None
                self._cmd_vel=None
            elif data.buttons[5] != 0:
                #Cartersian teleop control
                self._cmd_vel = np.array([data.axes[7], data.axes[6], data.axes[3], \
                                          data.axes[0], data.axes[1], data.axes[4]]) \
                                          *self._cmd_vel_gain
                self._cmd_vel_time=rospy.Time.now()
                self._trajectory_vel=None
                self._joint_vel=None
            else:
                self._joint_vel = None
                self._cmd_vel=None
                self._trajectory_vel=None
            
            self._cmd_halt = data.axes[2] < 0 or data.axes[5] < 0
    
    def current_command(self):
        with self._lock:
            #Clear stale command data
            now=rospy.Time.now()
            dt=rospy.Duration(0.5) #Half second timeout
            if (self._joint_vel is not None) and (now - self._joint_vel_time > dt):
                print (now-self._joint_vel_time).to_sec()
                self._joint_vel=None
            if (self._cmd_vel is not None) and (now - self._cmd_vel_time > dt):
                self._cmd_vel=None
            if (self._trajectory_vel is not None) and (now-self._trajectory_vel_time > dt):
                self._trajectory_vel=None            
            
            return self._joint_vel, self._cmd_vel, self._trajectory_vel, self._cmd_halt
            