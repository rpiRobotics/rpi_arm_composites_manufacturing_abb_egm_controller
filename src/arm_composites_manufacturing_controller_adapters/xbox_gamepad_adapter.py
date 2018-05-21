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
            