/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 10; //100 //maybe 1000?

	default_random_engine gen;

	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for( int i = 0; i < num_particles; i++) {
		double sample_x = dist_x(gen);
		double sample_y = dist_y(gen);
		double sample_theta = dist_theta(gen);

		particles.push_back(Particle());
		particles[i].id = i;
		particles[i].x =  sample_x;
		particles[i].y = sample_y;
		particles[i].theta = sample_theta;
		particles[i].weight = 1.0;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
	double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	for( int i = 0; i < num_particles; i++) {

		//Equations from Lesson 14 Section 7
		double theta_0 = particles[i].theta;
		double theta_delta = yaw_rate * delta_t;
		double theta_f = theta_0 + theta_delta;

		double y_0 = particles[i].y;
		double x_0 = particles[i].x;

		double y_delta, x_delta;
		if( yaw_rate == 0 ) {
			y_delta = velocity * delta_t * sin(theta_0);
			x_delta = velocity * delta_t * cos(theta_0);
		} else {
			y_delta = (velocity/yaw_rate)*(cos(theta_0)-cos(theta_f));
			x_delta = (velocity/yaw_rate)*(sin(theta_f)-sin(theta_0));
		}

		double y_f = y_0 + y_delta;
		double x_f = x_0 + x_delta;

		//Add noise to final pos values
		normal_distribution<double> dist_x(x_f, std_x);
		normal_distribution<double> dist_y(y_f, std_y);
		normal_distribution<double> dist_theta(theta_f, std_theta);

		double sample_x = dist_x(gen);
		double sample_y = dist_y(gen);
		double sample_theta = dist_theta(gen);

		particles[i].x =  sample_x;
		particles[i].y = sample_y;
		particles[i].theta = sample_theta;
	}

}

//Nearest Neighbor
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, 
	std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	cout << "Begin updateWeights" << endl;

	//Parse the noise parameter
	double std_x = std_landmark[0];
	double std_y = std_landmark[1];

	//Initialize a sum for normalizing weights
	double W = 0.0;

	//For each particle
	for( int i = 0; i<num_particles; i++ ) {

		Particle particle = particles[i];

		std::vector<LandmarkObs> predictions;
		predictions.clear();
		predictions.reserve(observations.size());

		//For every observation,
		for( int j = 0 ; j < observations.size() ; j++ ) {

			//Transform observation to map coordinate system
			LandmarkObs observation = observations[j];
			observation.x = particle.x + observation.x*cos(particle.theta) - observation.y*sin(particle.theta);
			observation.y = particle.y + observation.x*sin(particle.theta) + observation.y*cos(particle.theta);
			
			//Declare variables to hold the closest observation and its distance
			LandmarkObs min_observation;
			double min_dist = sensor_range;

			//Compare each observation to each landmark
			for( Map::single_landmark_s landmark : map_landmarks.landmark_list ) {

				//Get the distance
				double observation_distance = dist( landmark.x_f, landmark.y_f, observation.x, observation.y);
				
				//Test to see if this landmark is the new closest and associate it if so.
				if( observation_distance < min_dist ) {

					//This is the new min
					min_dist = observation_distance;

					//Associate the prediction and save the min observation
					observation.id = landmark.id_i;
					min_observation = observation;

					//Not sure if this matters
					observations[i].id = landmark.id_i;

				}

			}
			
			//Add the closest observation to the predictions
			predictions.push_back( min_observation );
			
		}


		//Initialize Particle's weight with the multiplicative identity
		particles[i].weight = 1.0;

		//Iterate through each predicted observation / landmark pair
		for( int j = 0; j < predictions.size() ; j++) {

			//Get the observation (already in map coords)
			LandmarkObs prediction = predictions[j];

			//And its associated landmark
			//Technically I should iterate and esnure prediciton.id == landmark.id_i
			//But this works.
			Map::single_landmark_s landmark = map_landmarks.landmark_list[prediction.id-1];

			
			//Calculate the weight.
			double delta_x = prediction.x - landmark.x_f; 
			double delta_y = prediction.y - landmark.y_f; 
			////cout << "Delta\tX:" << delta_x << "\tY: " << delta_y << endl;
			double exponent_term_x = pow(delta_x,2)/(2*pow(std_x,2));
			double exponent_term_y = pow(delta_y,2)/(2*pow(std_y,2));
			////cout << "exponent_term\tX:" << exponent_term_x << "\tY: " << exponent_term_y << endl;
			double exponent_term = -1.0*(exponent_term_x+exponent_term_y);
			////cout << "exponent_term " << exponent_term << endl;
			double exponent = exp(exponent_term);
			////cout << "exponent " << exponent << endl;
			double constant = 1/(2*M_PI*std_x*std_y);
			////cout << "constant " << constant << endl;
			double observation_weight = constant * exponent;
			////cout << "observation_weight " << observation_weight << endl;

			//Multiply it into the particle's total weight.
			particles[i].weight *= observation_weight;

		}

		//Add the particles total weight to our running sum.
		W+=particles[i].weight;
		
	}

	//Clear old weights
	weights.clear();
	weights.reserve(num_particles);

	//Normalize Weights and repopulate weights array	
	for( int i = 0; i < num_particles; i++ ) {
		/*if( W > 0.001 ) { // must need more zeros?
			particles[i].weight/=W;
		} else { 
			////cout << "Error: all weights were 0" << endl;
			particles[i].weight=1.0/num_particles;
		}*/
		particles[i].weight/=W;
		weights.push_back(particles[i].weight);
	}

	cout << "End updateWeights" << endl;

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	cout << "Begin resample" << endl;

	//cout << num_particles << endl;
	for( int i = 0; i < num_particles; i++ ) {
		//cout << "weights[" << i << "] = " << weights[i] << endl;
	}

	//Initialize std::discrete_distribution
	default_random_engine gen;
	std::discrete_distribution<int> d(weights.begin(), weights.end());

	//Resamping the particles
	std::vector<Particle> resampled_particles;
	for( int i = 0; i < num_particles; i++ ) {
		Particle random_particle = particles[d(gen)];
		resampled_particles.push_back(random_particle); 
	}
	particles = resampled_particles;

	cout << "End resample" << endl;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
