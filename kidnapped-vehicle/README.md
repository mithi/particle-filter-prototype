

## Project Introduction
My robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I have implemented a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.
- http://robots.stanford.edu/papers/thrun.pf-in-robotics-uai02.pdf

For this project, only [this source file](https://github.com/mithi/kidnapped-vehicle/blob/master/src/particle_filter.cpp) has been written, the rest including the data are cloned from the following repository:
- https://github.com/udacity/CarND-Kidnapped-Vehicle-Project

## Running the Code
```
> ./clean.sh
> ./build.sh
> ./run.sh
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   control_data.txt
|   |   gt_data.txt
|   |   map_data.txt
|   |
|   |___observation
|       |   observations_000001.txt
|       |   ...
|       |   observations_002444.txt
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

#### Control Data
`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

#### Observation Data
The `observation` directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes place.

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE.
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.
