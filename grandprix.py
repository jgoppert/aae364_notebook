import numpy as np
import control
import scipy
import matplotlib.pyplot as plt


class SE2:
  """
  This is an implementation of the mathematical group SE2, that represents rigid
  body motions in the plane. We are using it as it allows us to turn the 
  non-linear control problem of driving a car on a plane into a linear control
  problem that you can solve with the methods learned in this class.
  
  @see http://ethaneade.com/lie.pdf
  @see https://www.youtube.com/watch?v=mJ8ZDdA10GY
  """
  
  def from_params(self, v):
    """`
    Create group form parameterization.
    v: [theta, x, y]
    """
    theta, x, y = v
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

  def to_params(self, G):
    """
    Get parameterization of group.
    v = [theta, x, y]
    """
    theta = np.arctan2(G[1, 0], G[0, 0])
    x = G[0, 2]
    y = G[1, 2]
    return np.array([theta, x, y])
  
  def wedge(self, v):
    """
    This function takes a vector in R^3 and transforms it into an element of
    the lie algebra using the wedge operator.
    
    @param v:
      v[0] : dtheta - rotational rate
      v[1] : ux - velocity in body x frame
      v[2] : uy - velocity in body y frame
    @return The 3x3 matrix in the lie algebra
    """
    dtheta, dx, dy = v
    return np.array([
        [0, -dtheta, dx],
        [dtheta, 0, dy],
        [0, 0, 0]
        ])
  
  def vee(self, Omega):
    """
    This takes an element of the lie algebra
    and transforms it to a vector in R^n using the vee operator.
    
    @param Omega: element of lie algebra
    @return vector in R^3
    """
    theta = Omega[1, 0]
    x = Omega[0, 2]
    y = Omega[1, 2]
    return np.array([theta, x, y])
 
  def exp(self, Omega):
    """
    This is the exponential map that transforms an element of the lie algebra
    se2 to the lie group SE2
    """
    theta = Omega[1, 0]
    u = np.array([Omega[0, 2], Omega[1, 2]])
    if np.abs(theta) < 1e-5:
      A = 1
      B = 0
    else:
      A = np.sin(theta)/theta
      B = (1 - np.cos(theta))/theta
    V = np.array([[A, -B], [B, A]])
    p = V.dot(u)
    return np.array([
        [np.cos(theta), -np.sin(theta), p[0]],
        [np.sin(theta), np.cos(theta), p[1]],
        [0, 0, 1]
    ])
  
  def log(self, G):
    """
    The is the log map that transforms an element in the lie group SE2 to the
    lie algebra se2
    """
    theta = np.arctan2(G[1, 0], G[0, 0])
    if np.abs(theta) < 1e-5:
      A = 1
      B = 0
    else:
      A = np.sin(theta)/theta
      B = (1 - np.cos(theta))/theta
    V_I = np.array([[A, B], [-B, A]])/(A**2 + B**2)
    p = np.array([G[0, 2], G[1, 2]])
    u = V_I.dot(p)
    return np.array([
        [0, -theta, u[0]],
        [theta, 0, u[1]],
        [0, 0, 0]
    ])


def test_SE2():
  """
  Make sure SE2 is working properly.
  """
  G = SE2()
  v = np.array([1, 2, 3])
  assert np.allclose(G.vee(G.wedge(v)), v)
  assert np.allclose(G.vee(G.log(G.exp(G.wedge(v)))), v)
  assert np.allclose(G.to_params(G.from_params(v)), v)
  
test_SE2()

class Sim:
  
  def __init__(self, Controller, dt=0.001, tf=0.1*2*np.pi, verbose=False):
    """
    Setup the sim and load the controller.
    """
    self.G = SE2()
    self.dt = dt
    self.tf = tf
    self.controller = Controller(dt)
    self.data = {
        't': [],
        'theta': [],
        'alpha': [],
        'x': [],
        'y': [],
        'theta_r': [],
        'x_r': [],
        'y_r': [],
        'throttle': [],
        'velocity': [],
        'steering': [],
        'wheel': [],
        'e_theta': [],
        'e_x': [],
        'e_y': []
    }
    
    # you can turn on/off noise and disturbance here
    self.enable_noise = 1 # turn on noise (0 or 1)
    self.enable_disturbance = 1 # turn on disturbance (0 or 1)
    
    # reference trajectory parameters
    self.verbose = verbose
    self.radius = 1
    self.width = 0.05
    self.l_cm = 0.01 # distance from center of mass to front wheel
    self.disturbance_mag_xy = 2e-1 # disturbance due to unmodelled effects
    self.disturbance_mag_theta = 1e-1 # magnitude of theta disturbance
    self.noise_mag = 1e-2 # magnitude of noise for error signal
    
    # actuators
    s = control.tf([1, 0], [0, 1])

    if self.verbose:
      print('sim initialized')

  def run(self):
    if self.verbose:
      print('sim started')
    
    # randomize noise and disturbance phase
    phi_dist = 0.1*np.pi*np.random.randn()
    phi_noise = 0.1*np.pi*np.random.randn()
    
    # put the car at the starting line, facing the right direction
    theta0 = 0
    x0 = self.radius + self.width/2
    y0 = 0
    X = self.G.from_params([theta0, x0, y0])
    Xr = self.G.from_params([theta0, self.radius, 0])
    
    # start reference position as starting line
    alpha_last = np.arctan2(y0, x0)
    distance = 0
    velocity = 0
    
    for t in np.arange(0, self.tf, self.dt):
      
      # compute error and control
      theta_r, x_r, y_r = self.G.to_params(Xr)
      theta, x, y = self.G.to_params(X)
      Xr = self.G.from_params([theta_r, x_r, y_r])
      error = self.G.vee(self.G.log(np.linalg.inv(Xr).dot(X)))
      
      # add noise
      error += self.enable_noise*self.noise_mag*(np.sin(30*2*np.pi*t + phi_noise))*velocity
      
      # check if you ran off the track
      if np.abs(np.sqrt(x**2 + y**2) - self.radius) > self.width:
        print('CRASH at time {:10.4f} seconds'.format(t))
        return 0
        
      # call the controller
      throttle, steering = self.controller.update(error, velocity)

      # update actuators
      if throttle < 0:
        throttle = 0
      elif throttle > 10:
        throttle = 10
      if steering > 1:
        steering = 1
      elif steering < -1:
        steering = -1
      wheel = steering
      velocity = throttle
        
      # simulate disturbance in body frame
      dist = self.enable_disturbance*(0.2 + np.sin(20*t*2*np.pi + phi_dist + np.random.randn()))*velocity
      disturbance_x = dist*self.disturbance_mag_xy
      disturbance_theta = dist*self.disturbance_mag_theta
      
      # integrate trajectory
      dtheta = velocity*np.tan(wheel)/self.l_cm + disturbance_theta
      dx = disturbance_x
      dy = velocity
      u = np.array([dtheta, dx, dy])
      dX = self.G.exp(self.G.wedge(u*self.dt))
      X = X.dot(dX)
      
      # reference trajectory, the race course
      u_r = np.array([self.controller.desired_speed/self.radius, 0, self.controller.desired_speed])
      dXr = self.G.exp(self.G.wedge(u_r*self.dt))
      Xr = Xr.dot(dXr)

      # track distance travelled
      alpha = np.arctan2(y, x)
      delta_alpha = alpha - alpha_last
      if delta_alpha > np.pi:
        delta_alpha -= 2*np.pi
      elif delta_alpha < -np.pi:
        delta_alpha += 2*np.pi
      distance += delta_alpha*self.radius
      alpha_last = alpha

      # store data
      self.data['t'].append(t)
      self.data['alpha'].append(alpha)
      self.data['theta'].append(theta)
      self.data['x'].append(x)
      self.data['y'].append(y)
      self.data['theta_r'].append(theta_r)
      self.data['x_r'].append(x_r)
      self.data['y_r'].append(y_r)
      self.data['throttle'].append(throttle)
      self.data['steering'].append(steering)
      self.data['velocity'].append(velocity)
      self.data['wheel'].append(wheel)
      self.data['e_theta'].append(error[0])
      self.data['e_x'].append(error[1])
      self.data['e_y'].append(error[2])

    # convert lists to numpy array for faster plotting
    for k in self.data.keys():
      self.data[k] = np.array(self.data[k])
    
    if self.verbose:
      print('sim complete')
      print('you made it: {:10.4f} m'.format(distance))
    
    return distance
 
  def plot(self):
    theta = np.linspace(0, 2*np.pi, 1000)
    plt.figure(figsize=(10, 10))
    plt.plot(
        (self.radius + self.width)*np.cos(theta),
        (self.radius + self.width)*np.sin(theta), 'r--')
    plt.plot(
        (self.radius - self.width)*np.cos(theta),
        (self.radius - self.width)*np.sin(theta),
        'r--', label='track')
    plt.plot(self.data['x'], self.data['y'], 'b', label='vehicle')
    plt.plot(self.data['x_r'], self.data['y_r'], 'r--', label='reference')
    plt.legend()
    plt.axis('equal')
    plt.grid()

    plt.figure(figsize=(10, 30))
    n = 3
    plt.subplot(n, 1, 1)
    plt.plot(self.data['t'], self.data['e_x'], label='e_x')
    plt.xlabel('t, sec')
    plt.ylabel('m')
    plt.legend()
    plt.title('cross track error')
    plt.grid()

    plt.subplot(n, 1, 2)
    plt.plot(self.data['t'], self.data['e_y'], label='e_y')
    plt.legend()
    plt.xlabel('t, sec')
    plt.ylabel('m')
    plt.title('along track error')
    plt.grid()

    plt.subplot(n, 1, 3)
    plt.plot(self.data['t'], np.rad2deg(self.data['e_theta']), label='e_theta')
    plt.legend()
    plt.xlabel('t, sec')
    plt.ylabel('deg')
    plt.title('angle error')
    plt.grid()

    plt.figure(figsize=(10, 20))
    n = 2
    plt.subplot(n, 1, 1)
    plt.plot(self.data['t'], self.data['throttle'], label='command')
    plt.plot(self.data['t'], self.data['velocity'], label='velocity')
    plt.legend()
    plt.xlabel('t, sec')
    plt.ylabel('velocity, m/s')
    plt.title('velocity')
    plt.grid()

    plt.subplot(n, 1, 2)
    plt.plot(self.data['t'], np.rad2deg(self.data['steering']), label='command')
    plt.plot(self.data['t'], np.rad2deg(self.data['wheel']), label='wheel')
    plt.legend()
    plt.xlabel('t, sec')
    plt.ylabel('angle, deg')
    plt.title('steering')
    plt.grid()


class DiscreteStateSpace:
  """
  Use this class to implement any controller you need.
  It takes a continuous time transfer function.
  """
  
  def __init__(self, H, dt):
    sys = control.tf2ss(control.c2d(H, dt))
    self.x = np.zeros((sys.A.shape[0], 1))
    self.A = sys.A
    self.B = sys.B
    self.C = sys.C
    self.D = sys.D
    self.dt = sys.dt

  def update(self, u):
    self.x = self.A.dot(self.x) + self.B.dot(u)
    return self.C.dot(self.x) + self.D.dot(u)
 
  def __repr__(self):
    return repr(self.__dict__)