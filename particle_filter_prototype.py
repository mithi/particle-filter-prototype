from collections import namedtuple
from copy import deepcopy
from random import random, gauss
from math import pi, sin, cos, sqrt, exp
import numpy as np
def get_random_position(world_size):
    
  x = random() * world_size
  y = random() * world_size
  heading =  random() * 2. * pi 

  return [x, y, heading]

def distance(ax, ay, bx, by):
  return sqrt((ax - bx)**2 + (ay - by)**2)

