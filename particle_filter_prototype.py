from collections import namedtuple
from copy import deepcopy
from random import random, gauss
from math import pi, sin, cos, sqrt, exp
import numpy as np
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
    

