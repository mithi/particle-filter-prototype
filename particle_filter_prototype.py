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
