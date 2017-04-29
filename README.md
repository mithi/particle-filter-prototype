# A particle-filter visualization in Python based on [Udacity's free A.I. for Robotics course](https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373) using [Bokeh](bokeh.pydata.org)
- For visualization, if you check 
[this jupyter notebook](https://github.com/mithi/particle-filter-prototype/blob/master/visualization_only.ipynb)
you'll see that you get the particle filter history by running the following:

```python
from particle_filter_prototype import *
robot_history, particles_history = run()
```
- Tune the global variables in ```lines 11-27``` in 
[this file](https://github.com/mithi/particle-filter-prototype/blob/master/particle_filter_prototype.py)
to suit your needs

```python
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
```

![Animation 1](https://github.com/mithi/particle-filter-prototype/blob/master/docs/animation1.gif)
![Animation 2](https://github.com/mithi/particle-filter-prototype/blob/master/docs/animation2.gif)
