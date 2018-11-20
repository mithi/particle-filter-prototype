#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include "particle_filter.h"

using namespace std;

const int NUMBER_OF_PARTICLES = 300;
const double INITIAL_WEIGHT = 1.0;

/*
 * NOTE(s):
 * GAUSSIAN NOISE
 *   - http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
 *   - http://www.cplusplus.com/reference/random/default_random_engine/
 * MULTIVARIATE NORMAL DISTRIBUTIONS
 *   - https://en.wikipedia.org/wiki/Multivariate_normal_distribution
 * TRANSFORMING FROM MAP TO VEHICLE COORDINATE
 *  - https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
 * The map's y-axis actually points downwards.
 *  - http://planning.cs.uiuc.edu/node99.html
 * DISCRETE_DISTRIBUTION
 *  - http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
 * A  Mersenne Twister pseudo-random generator of 32-bit numbers with a state size of 19937 bits.
 * - http://www.cplusplus.com/reference/random/default_random_engine/
 */


/***************************************************************
 * Set the number of particles. Initialize all particles to first position
 * (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
 * random gaussian noise is added to each particle
 ***************************************************************/
void ParticleFilter::init(double x, double y, double theta, double std[]) {

  this->num_particles = NUMBER_OF_PARTICLES;
  random_device random_device;
  mt19937 gen(random_device());
  normal_distribution<> particle_x(x, std[0]);
  normal_distribution<> particle_y(y, std[1]);
  normal_distribution<> particle_theta(theta, std[2]);

  for (int i = 0; i < NUMBER_OF_PARTICLES; i++) {

    Particle p = {
      i,
      particle_x(gen),
      particle_y(gen),
      particle_theta(gen),
      INITIAL_WEIGHT
    };

    this->weights.push_back(INITIAL_WEIGHT);
    this->particles.push_back(p);
  }

  this->is_initialized = true;
}


/***************************************************************
 *  Add measurements to each particle and add random Gaussian noise.
 ***************************************************************/
void ParticleFilter::prediction(double delta_t, double std[], double velocity, double yaw_rate) {

  const double THRESH = 0.001;
  
  random_device random_device;
  mt19937 gen(random_device());
  normal_distribution<> noise_x(0.0, std[0]);
  normal_distribution<> noise_y(0.0, std[1]);
  normal_distribution<> noise_theta(0.0, std[2]);
    
  for (int i = 0;  i < NUMBER_OF_PARTICLES; i++) {

    double d = velocity * delta_t;
    double theta = this->particles[i].theta;
    
    if (fabs(yaw_rate) < THRESH) { //moving straight

      this->particles[i].x += d * cos(theta) + noise_x(gen);
      this->particles[i].y += d * sin(theta) + noise_y(gen);
      this->particles[i].theta += noise_theta(gen);

    } else {

      double delta_theta = yaw_rate * delta_t;
      double phi = theta + delta_theta;
      double k = velocity / yaw_rate;

      this->particles[i].x += k * (sin(phi) - sin(theta)) + noise_x(gen);
      this->particles[i].y += k * (cos(theta) - cos(phi)) + noise_y(gen);
      this->particles[i].theta = phi + noise_theta(gen);
    }
  }
}


/**************************************************************
 * Find the predicted measurement that is closest to each observed measurement
 * and assign the observed measurement to this particular landmark.
 ***************************************************************/
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations){

  const double BIG_NUMBER = 1.0e99;

  for (int i = 0; i < observations.size(); i++) {

    int current_j;
    double current_smallest_error = BIG_NUMBER;

    for (int j = 0; j < observations.size(); j++) {

      double dx = predicted[j].x - observations[i].x;
      double dy = predicted[j].y - observations[i].y;

      double error = dx * dx + dy * dy;

      if (error < current_smallest_error) {
        current_j = j;
        current_smallest_error = error;
      }
    }
    observations[i].id = current_j;
  }
}


/***************************************************************
 *  Update the weights of each particle using a mult-variate Gaussian distribution.
 *  NOTE: The observations are given in the VEHICLE'S coordinate system. Particles are located
 *        according to the MAP'S coordinate system. So transformation is done.
 * For each particle:
 *   1. transform observations from vehicle to map coordinates assuming it's the particle observing
 *   2. find landmarks within the particle's range
 *   3. find which landmark is likely being observed based on nearest neighbor method
 *   4. determine the weights based difference particle's observation and actual observation
 ***************************************************************/
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {

  for (int  i = 0; i < NUMBER_OF_PARTICLES; i++) {

    double px = this->particles[i].x;
    double py = this->particles[i].y;
    double ptheta = this->particles[i].theta;

    vector<LandmarkObs> landmarks_in_range;
    vector<LandmarkObs> map_observations;

   /**************************************************************
    * STEP 1:
    * transform each observations to map coordinates
    * assume observations are made in the particle's perspective
    **************************************************************/
    for (int j = 0; j < observations.size(); j++){

      int oid = observations[j].id;
      double ox = observations[j].x;
      double oy = observations[j].y;

      double transformed_x = px + ox * cos(ptheta) - oy * sin(ptheta);
      double transformed_y = py + oy * cos(ptheta) + ox * sin(ptheta);

      LandmarkObs observation = {
        oid,
        transformed_x,
        transformed_y
      };

      map_observations.push_back(observation);
    }

   /**************************************************************
    * STEP 2:
    * Find map landmarks within the sensor range
    **************************************************************/
    for (int j = 0;  j < map_landmarks.landmark_list.size(); j++) {

      int mid = map_landmarks.landmark_list[j].id_i;
      double mx = map_landmarks.landmark_list[j].x_f;
      double my = map_landmarks.landmark_list[j].y_f;

      double dx = mx - px;
      double dy = my - py;
      double error = sqrt(dx * dx + dy * dy);

      if (error < sensor_range) {

        LandmarkObs landmark_in_range = {
          mid,
          mx,
          my
         };

        landmarks_in_range.push_back(landmark_in_range);
      }
    }

  /**************************************************************
   * STEP 3:
   * Associate landmark in range (id) to landmark observations
   * this function modifies std::vector<LandmarkObs> observations
   * NOTE: - all landmarks are in map coordinates
   *       - all observations are in map coordinates
   **************************************************************/
    dataAssociation(landmarks_in_range, map_observations);

   /**************************************************************
    * STEP 4:
    * Compare each observation (by actual vehicle) to corresponding
    * observation by the particle (landmark_in_range)
    * update the particle weight based on this
    **************************************************************/
    double w = INITIAL_WEIGHT;

    for (int j = 0; j < map_observations.size(); j++){

      int oid = map_observations[j].id;
      double ox = map_observations[j].x;
      double oy = map_observations[j].y;

      double predicted_x = landmarks_in_range[oid].x;
      double predicted_y = landmarks_in_range[oid].y;

      double dx = ox - predicted_x;
      double dy = oy - predicted_y;

      double varx = std_landmark[0];
      double vary = std_landmark[1];

      double a = dx * dx / (2.0 * varx * varx);
      double b = dy * dy / (2.0 * vary * vary);
      double d = sqrt( 2.0 * M_PI * varx * vary);
      double r = exp(-(a + b)) / d;
      // There are elements of this calculation (such as the denominator) 
      // that don't depend on the particle, landmark, or observation. 
      // I suggest calculating them separately outside of the particle loop and reusing these values.
      w *= r;
    }

    this->particles[i].weight = w;
    this->weights[i] = w;
  }
}


/**************************************************************
 * Resample particles with replacement with probability proportional to their weight.
 ***************************************************************/
void ParticleFilter::resample(){

  vector<Particle> resampled_particles;
	random_device random_device;
  mt19937 gen(random_device());
  discrete_distribution<int> index(this->weights.begin(), this->weights.end());

  for (int c = 0; c < NUMBER_OF_PARTICLES; c++) {

    int i = index(gen);

    Particle p {
      i,
      this->particles[i].x,
      this->particles[i].y,
      this->particles[i].theta,
      INITIAL_WEIGHT
    };

    resampled_particles.push_back(p);
  }

  this->particles = resampled_particles
}


void ParticleFilter::write(std::string filename) {
  // You don't need to modify this file.
  std::ofstream dataFile;
  dataFile.open(filename, std::ios::app);

  for (int i = 0; i < this->num_particles; ++i) {
    dataFile << this->particles[i].x << " " << this->particles[i].y << " " << this->particles[i].theta << "\n";
  }

  dataFile.close();
}

    
