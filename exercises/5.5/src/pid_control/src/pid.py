import sys
import numpy as np
import rclpy
from moveit_msgs.msg import RobotTrajectory

class PID(rclpy.node.Node):
  def __init__(self, Kp, Ki, Kd, Kw):
    super().__init__('pid_controller_node')
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd
    self.Kw = Kw

    self.lastError = np.zeros(len(Kd))
    self.lastTime = 0
    self.intError = np.zeros(len(Ki))
    self.ring_buff = [0.0, 0.0, 0.0]

    self.path = RobotTrajectory()
    self.curIndex = 0
    self.maxIndex = 0

  def control(self, t):
    while (rclpy.ok() and self.curIndex < self.maxIndex and self.path.joint_trajectory[self.curIndex+1].time_from_start.to_sec() < t+0.001):
      self.curIndex += 1 

    current_position = 
    current_velocity = 

    target_position = np.array(self.path.joint_trajectory.points[self.curIndex].positions)
    target_velocity = np.array(self.path.joint_trajectory.points[self.curIndex].velocities)

    # Feed forward term
    u_ff = target_velocity
    # Error term
    error = target_position - current_position
    # Integral term
    self.intError = self.Kw * self.intError + error
    # Derivative term
    dt = t - self.lastTime
    # We use a moving average filter to smooth the derivative 
    self.ring_buff[2] = self.ring_buff[1]
    self.ring_buff[1] = self.ring_buff[0]
    self.ring_buff[0] = (error - self.lastError) / dt
    ed = np.mean(self.ring_buff)

    self.lastError = error 
    self.lastTime = t 

    u = u_ff + (self.Kp*error) + (self.Ki*self.intError) + (self.Kd*ed)

    return u 
