from collections import namedtuple
from copy import deepcopy
from random import random, gauss
from math import pi, sin, cos, sqrt, exp
import numpy as np

##################################################################
# SETTINGS
##################################################################

Landmark = namedtuple('landmark', 'x y')
WORLD_SIZE = 100
LANDMARKS = [Landmark(x = 0.2 * WORLD_SIZE, y = 0.2 * WORLD_SIZE), 
             Landmark(x = 0.2 * WORLD_SIZE, y = 0.8 * WORLD_SIZE), 
             Landmark(x = 0.8 * WORLD_SIZE, y = 0.2 * WORLD_SIZE), 
             Landmark(x = 0.8 * WORLD_SIZE, y = 0.8 * WORLD_SIZE)]

FORWARD_NOISE = 0.05
TURN_NOISE = 0.05
SENSE_NOISE = 5.

NUMBER_OF_PARTICLES = 1000
NUMBER_OF_STEPS = 15
FORWARD_DISTANCE = 2.
TURN_DISTANCE = pi / 6. 

ROBOT_INITIAL_POSITION = [50., 50., pi / 2.]
#ROBOT_INITIAL_POSITION = [random() * WORLD_SIZE, random() * WORLD_SIZE, random() * 2. * pi]


##################################################################
# HELPERS
##################################################################

def gaussian_prob(mu, sigma, x): 
    # calculate the probability of x for 1-dim Gaussian with mean mu and var sigma 
    return exp( -((mu - x) ** 2) / (sigma ** 2) / 2.) / sqrt( 2. * pi * (sigma ** 2))    

def get_weight(my_measurements, ground_measurements, noise):   

  w = 1.
  for my_distance, ground_distance in zip(my_measurements, ground_measurements):
    w *= gaussian_prob(mu = my_distance, sigma = noise, x = ground_distance)
  
  return w + 1.e-300 # avoid round-off to zero

def get_random_position(world_size):
    
  x = random() * world_size
  y = random() * world_size
  heading =  random() * 2. * pi 

  return [x, y, heading]

def distance(ax, ay, bx, by):
  return sqrt((ax - bx)**2 + (ay - by)**2)

def evaluate(robot, particles):
  # EVALUATE PARTICLE FILTER'S PERFORMANCE
    
  rx, ry, _ = robot.get_current_position()

  s = 0.
  for particle in particles:    
    px, py, _ = particle.get_current_position()
    s += sqrt((px - rx) ** 2 + (py - ry) ** 2)
  
  return s / len(particles)


##################################################################
# PARTICLE
##################################################################

class Particle:
    
  def __init__(self, position = [0., 0., 0.], noises = [0., 0., 0.]): 
    
    forward_noise, turn_noise, sense_noise = noises
    self.set_noise(forward_noise, turn_noise, sense_noise)
    
    x, y, heading = position
    self.set_position(x, y, heading)
      
  def set_position(self, x, y, heading): 
    heading %= (2. * pi)        
    self.x, self.y, self.heading = x, y, heading
    
  def set_noise(self, forward_noise, turn_noise, sense_noise):
    self.forward_noise = forward_noise
    self.turn_noise = turn_noise
    self.sense_noise = sense_noise

  def forward(self, d):
    distance = d + gauss(0., self.forward_noise)
    self.x = (self.x + distance * cos(self.heading)) 
    self.y = (self.y + distance * sin(self.heading))
    
  def turn(self, angle):
    self.heading += (angle + gauss(0., self.turn_noise))
    self.heading %= (2 * pi)
               
  def move(self, angle, distance):
    self.turn(angle)
    self.forward(distance)
  
  def sense(self, landmarks, with_noise = True):
        
    distances = []
    
    for landmark in landmarks:
      d = distance(self.x, self.y, landmark.x, landmark.y)
      if with_noise: d += gauss(0., self.sense_noise)
      distances.append(d)
    
    return distances

  def get_current_position(self):
    return [self.x, self.y, self.heading]
    

##################################################################
# PARTICLE FILTER UPDATE FUNCTIONS
##################################################################

def move(robot, particles, turn_distance, forward_distance, sense_noise):
  #------------------------------------------------------------------------
  # move the robot and sense surroundings
  #------------------------------------------------------------------------
  robot.move(angle = turn_distance, distance = forward_distance)
  ground_measurements = robot.sense(LANDMARKS)

  #------------------------------------------------------------------------
  # move each particle and sense surroundings
  # get each particle's weight based 
  # on how much the particle and the robot's measurements are alike
  #------------------------------------------------------------------------
  weights = []

  for particle in particles:        
    particle.move(angle = turn_distance, distance = forward_distance)
    particle_measurements = particle.sense(LANDMARKS, with_noise = False)

    w = get_weight(particle_measurements, ground_measurements, sense_noise)
    weights.append(w)
  
  return robot, particles, weights


def resample(particles, weights):
  #------------------------------------------------------------------------    
  # calculate probability and "window" of particle getting picked in "resampling stage"
  # IE get normalized cumulative weights
  #------------------------------------------------------------------------
  normalized_prob = np.array(weights) / sum(weights)
  cumulative_prob = []
  current = 0.
  number_of_particles = len(particles)
    
  for i in range(number_of_particles):
    cumulative_prob.append(current)
    current += normalized_prob[i]
  
  #------------------------------------------------------------------------
  # get the new set of particles by "resampling"
  #------------------------------------------------------------------------
  resampled_particles = []
    
  for i in range(number_of_particles):
    
    current = random()
    j = number_of_particles - 1
    
    while cumulative_prob[j] > current:
      j -= 1
    
    resampled_particles.append(deepcopy(particles[j]))
    
  return resampled_particles

##################################################################
# PARTICLE FILTER IS PERFORMED BY RUNNING THIS
##################################################################

def run():
  #------------------------------------------------------------------------
  # INITIALIZE ROBOT AND PARTICLES AT RANDOM POSITIONS + HEADING
  #------------------------------------------------------------------------  
  robot_history, particles_history = [], []
    
  myRobot = Particle(position = ROBOT_INITIAL_POSITION)
  myParticles = []

  for i in range(NUMBER_OF_PARTICLES):
    p = Particle(position = get_random_position(WORLD_SIZE), noises = [FORWARD_NOISE, TURN_NOISE, SENSE_NOISE])
    myParticles.append(p)

  robot_history.append(deepcopy(myRobot))
  particles_history.append(deepcopy(myParticles))

  #------------------------------------------------------------------------
  # PERFORM PARTICLE FILTER UPDATES
  #------------------------------------------------------------------------
  for i in range(NUMBER_OF_STEPS):
  
    ### MOVE STEP
    myRobot, myParticles, w = move(robot = myRobot, particles = myParticles, sense_noise = SENSE_NOISE,
                                   turn_distance = TURN_DISTANCE, forward_distance = FORWARD_DISTANCE)

    robot_history.append(deepcopy(myRobot))
    particles_history.append(deepcopy(myParticles))

    ### RESAMPLE STEP
    myParticles = resample(particles = myParticles, weights = w)
    
    robot_history.append(deepcopy(myRobot))
    particles_history.append(deepcopy(myParticles))
    
  return robot_history, particles_history

