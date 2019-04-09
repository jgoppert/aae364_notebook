from grandprix import DiscreteStateSpace
import numpy as np
import control
import scipy

class Controller:
  """
  You must design a controller using a PID or lead-lag controller or
  a set or any combination of these controller to design the best
  control system possible that drives the race course without
  leaving the track.
  """
  
  def __init__(self, dt):
    """
    @param dt: the update rate of the controller
    """
    self.dt = dt

    ###
    ### WRITE YOUR OWN CONTROLLERS HERE
    ###
    s = control.tf([1, 0], [0, 1])
    
    H = 1 + 0/s + 0*s*10/(s+10)
    self.ex_steering = DiscreteStateSpace(H, dt)

    # don't change this
    self.desired_speed = 10
  
  def update(self, error, velocity):
    """
    current state:
    @param error: [etheta, ex, ey]
    @param velocity: how fast you are moving, m/s
    
    ouput
    @param throttle: the throttle to the motor, commands velocity m/s
    @param steering: the steering angle of the vehicle in radians
    """
    etheta, ex, ey = error
    
    ###
    ### WRITE YOUR CODE HERE TO CONTROL THE CAR
    ###
    steering = float(self.ex_steering.update(ex))

    # for the race, just leave the throttle at desired_speed
    throttle = self.desired_speed
    return throttle, steering