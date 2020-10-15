/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  
  //std::cout << "Init:" << std::endl;
  
  std::default_random_engine gen;
  double std_x, std_y, std_theta;  // Standard deviations for x, y, and theta
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];
  
  // TODO: Set standard deviations for x, y, and theta
  // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  // TODO: Create normal distributions for y and theta

  //std::cout << "Before Loop :" << std::endl;
  for (int i = 0; i < this->num_particles ; ++i) {
    
    
    Particle sample;
    
    sample.x = dist_x(gen);
    sample.y = dist_y(gen);
    sample.theta = dist_theta(gen);
    sample.weight = 1;
     
    //std::cout << "Before particles :" << std::endl;
    
    this->particles.push_back(sample);
    
    
    /*
    std::cout << "Sample " << i + 1 << " " << this->particles[i].x << " " << this->particles[i].y << " " 
              << this->particles[i].theta << " " << this->particles[i].weight << std::endl;
              */
    
  }
  
  this->is_initialized = true;

  //std::cout << "End Init :" << std::endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  double std_x, std_y, std_theta;  // Standard deviations for x, y, and theta
  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];
  
  // TODO: Set standard deviations for x, y, and theta
  // This line creates a normal (Gaussian) distribution for x
  
  
  double x, y, theta; 
  
  for (int i = 0; i < num_particles ; ++i) {
    

    x = this->particles[i].x;
    y = this->particles[i].y;
    theta = this->particles[i].theta;
    
    x = x + (velocity/yaw_rate)*(sin(theta + yaw_rate*delta_t) - sin(theta));
    y = y + (velocity/yaw_rate)*(sin(theta + yaw_rate*delta_t) - sin(theta));
    theta = theta + (velocity/yaw_rate)*(sin(theta + yaw_rate*delta_t) - sin(theta));
    
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);
    
    this->particles[i].x = dist_x(gen);
    this->particles[i].y = dist_y(gen);
    this->particles[i].theta = dist_theta(gen);
    
    
  }
  

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     const Map &map_landmarks) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  		
  	
    int nPredicted = predicted.size();
    std::cout<<"In Data assoc : "<< nPredicted <<std::endl;
  
    //Para cada observacion:
    for (int p = 0; p < nPredicted ; p++){
        //Busque el landmark mas cercano:
        double distanciaMinima = 1000000, distancia;
      
        int nObservations = map_landmarks.landmark_list.size();
      
        for(int l = 0 ; l < nObservations ; l++){

            distancia = dist(predicted[p].x,predicted[p].y,map_landmarks.landmark_list[l].x_f,map_landmarks.landmark_list[l].y_f);

            if(distancia < distanciaMinima){
              predicted[p].id = map_landmarks.landmark_list[l].id_i;
            }
        }
        std::cout<<"Observation : "<< predicted[p].id <<std::endl;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  std::cout<<"updateWeights"<<std::endl;
  std::cout<<"num_particles: "<<num_particles<<std::endl;
  double theta = -M_PI/2; 
  
  for(int p = 0; p < num_particles; p++){
    std::cout<<"Particle  : "<<p<<std::endl;
    
    //Transformo:
    vector<LandmarkObs> obsTransformed;
    int nObservations = observations.size();
    std::cout<<"Particle : "<<p<<std::endl;
    std::cout<<"Observations : "<<nObservations<<std::endl;
    for(int o = 0; o < nObservations; o++){
      double x_obs, y_obs;
      LandmarkObs obs;
      
      x_obs = observations[o].x;
      y_obs = observations[o].y;
      
      obs.x = particles[p].x + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      obs.y = particles[p].y + (sin(theta) * x_obs) + (cos(theta) * y_obs);
      
      obsTransformed.push_back(obs);    
    
    }
    
    
    
    
    
    //Asocio::
   // std::vector<single_landmark_s> mLandmark_list;
    
    //mLandmark_list = Map->landmark_list;
    std::cout<<"dataAssociation : "<<std::endl;
    this->dataAssociation( obsTransformed , map_landmarks );
    
  
    //Saco el peso de cada una:
    for(int o = 0; o < nObservations; o++){
    	//LandmarkObs aLandmark;
    	//Buscar su landmark:
      	int nLandmarks = map_landmarks.landmark_list.size();
        for(int l = 0; l < nLandmarks; l++){
			if(map_landmarks.landmark_list[l].id_i == observations[o].id){
              //Multiplique la probabilidad de su peso:
              particles[p].weight *= multiv_prob(std_landmark[0],std_landmark[1],observations[o].x,observations[o].y,map_landmarks.landmark_list[l].x_f,map_landmarks.landmark_list[l].y_f);
              
            }
        }   
    }
    std::cout<<"Particle Weight : "<<particles[p].weight<<std::endl;
    
  }
  std::cout<<"End Updating... "<<std::endl;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::cout<<"Init Resampling... "<< weights.size() << std::endl;
  std::default_random_engine generator;
  weights.clear();
  
  double totalW = 0;
  //Calculo la suma:
  for(int p = 0; p < num_particles; p++){
    totalW += particles[p].weight;
  }
  //Normalizo:
  for(int p = 0; p < num_particles; p++){
    weights.push_back( particles[p].weight / totalW );
  }
  
  std::cout<<"Before Distributions...Num Particles "<< num_particles << std::endl;
  //std::discrete_distribution<int> distribution;
  std::discrete_distribution<int> distribution(weights.begin(), weights.end());
  std::vector<Particle> newParticles;
  
  int number = 0;
  std::cout<<"Before Loop, Weights Size: "<<  weights.size()  <<std::endl;
  for(int p = 0; p < num_particles; p++){
    
    std::cout<<"Loop... "<<p<<std::endl;
    number = distribution(generator);
    std::cout<<"Mid Loop... "<<number<<std::endl;
    newParticles.push_back(particles[number]);
        
  }
  
  std::cout<<"Before New Particle... "<< newParticles.size()<<std::endl;
  
  particles = newParticles;
  
  std::cout<<"End Resampling... "<<std::endl;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
