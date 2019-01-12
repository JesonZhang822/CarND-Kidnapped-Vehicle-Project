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
#include <map>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	//random engine 
	default_random_engine gen;

	// initialize the particle filter
	if(!initialized()){

		// normal distribution for x,y and theta 
		normal_distribution<double> dist_x(x,std[0]);
		normal_distribution<double> dist_y(y,std[1]);
		normal_distribution<double> dist_theta(theta,std[2]);

		// sample particles
		Particle sample_particle;

		// the number of particles
		num_particles = 200;

		cout << "Initialize the particles............."<< endl;

		for (int i=0;i< num_particles;i++){

			// Add random Gassian noise to each particle
			sample_particle.x  = dist_x(gen);
			sample_particle.y  = dist_y(gen);
			sample_particle.theta = dist_theta(gen);

			//set all weight to 1.0
			sample_particle.weight = 1.0;
			sample_particle.id = i;

			particles.push_back(sample_particle);
/*
			cout << "particle ID :" << i << endl;
			cout << "Init particle x :" << sample_particle.x << endl;
			cout << "Init particle y :" << sample_particle.y << endl;
			cout << "Init particle theta :" << sample_particle.theta << endl;
			cout << "Init particle weight :" << sample_particle.weight << endl;
*/

		}

		is_initialized = true;
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	double pred_x,pred_y,pred_theta;

	// random engine
	default_random_engine gen;

	for (int i=0;i< num_particles; i++){

		// normal distribution for x,y,theta
		normal_distribution<double> x_noise(0.0,std_pos[0]);
		normal_distribution<double> y_noise(0.0,std_pos[1]);
		normal_distribution<double> theta_noise(0.0,std_pos[2]);
		
		// avoid the yaw rate to be zero
		if (fabs(yaw_rate) > 0.0001)
		{
			pred_x = particles[i].x + (velocity / yaw_rate) *(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			pred_y = particles[i].y + (velocity / yaw_rate) *(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			pred_theta = particles[i].theta + yaw_rate * delta_t;
				
		} else {

			pred_x = particles[i].x + velocity *cos(particles[i].theta)*delta_t;
			pred_y = particles[i].y + velocity *sin(particles[i].theta)*delta_t;
			pred_theta = particles[i].theta;
		}

		// add the noise
		particles[i].x = pred_x + x_noise(gen);
		particles[i].y = pred_y + y_noise(gen);
		particles[i].theta = pred_theta + theta_noise(gen);
	}


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	std::vector<LandmarkObs> obs_landmark_vector = observations;
	std::vector<LandmarkObs> pred_landmark_vector = predicted;
	std::vector<LandmarkObs> landmark_vector;
	LandmarkObs landmark;
	double dist_landmark,dist_landmark_min,index_min;

	// use a nearest-neighbors data association
	if (obs_landmark_vector.size() <= pred_landmark_vector.size()) {

		for (int k=0;k < obs_landmark_vector.size();k++){


			for (int m=0;m <pred_landmark_vector.size();m++){

				dist_landmark = dist(obs_landmark_vector[k].x,obs_landmark_vector[k].y,pred_landmark_vector[m].x,pred_landmark_vector[m].y);
				
				// get the association id of the map landmark
				if (m ==0){

					dist_landmark_min = dist_landmark;
					index_min = pred_landmark_vector[m].id;
				}else {
					if (dist_landmark <= dist_landmark_min){

						dist_landmark_min = dist_landmark;
						index_min = pred_landmark_vector[m].id;
					}

				}

			}

			landmark.id = index_min;
			landmark.x  = obs_landmark_vector[k].x;
			landmark.y  = obs_landmark_vector[k].y;
			landmark_vector.push_back(landmark);

		}

	} else {

		for (int k = 0;k < pred_landmark_vector.size(); k++){
			
			for (int m =0;m < obs_landmark_vector.size();m++){

				dist_landmark = dist(pred_landmark_vector[k].x,pred_landmark_vector[k].y,obs_landmark_vector[m].x,obs_landmark_vector[m].y);
				// get the association id of the map landmark
				if (m ==0){

					dist_landmark_min = dist_landmark;
					index_min = pred_landmark_vector[k].id;
				}else {
					if (dist_landmark <= dist_landmark_min){

						dist_landmark_min = dist_landmark;
						index_min = pred_landmark_vector[k].id;
					}

				}					


			}

			landmark.id = index_min;
			landmark.x  = obs_landmark_vector[k].x;
			landmark.y  = obs_landmark_vector[k].y;
			landmark_vector.push_back(landmark);

		}
	}
	// update the observations data
	observations = landmark_vector;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

	
	LandmarkObs obs_landmark,pred_landmark;

	double x_map,y_map,distance;

	std::vector<LandmarkObs> pred_landmark_vector;
	std::vector<LandmarkObs> obs_landmark_vector;

	for (int j=0;j < num_particles ; j++){

		// clear the vectors
		pred_landmark_vector.clear();
		obs_landmark_vector.clear();

		particles[j].associations.clear();
		particles[j].sense_x.clear();
		particles[j].sense_y.clear();

		//convert the vechile's coordinate system to the map's coordinate system
		for (int i=0;i < observations.size();i++){

			x_map = particles[j].x + (cos(particles[j].theta) * observations[i].x) - (sin(particles[j].theta) * observations[i].y);
			y_map = particles[j].y + (sin(particles[j].theta) * observations[i].x) + (cos(particles[j].theta) * observations[i].y);

			obs_landmark.id = observations[i].id + i;
			obs_landmark.x  = x_map;
			obs_landmark.y  = y_map;

			obs_landmark_vector.push_back(obs_landmark);

		}

		// get the landmarks in the range of sensor 
		for (int n=0;n < map_landmarks.landmark_list.size();n++){

			// calculate the distance between the particle and the map landmarks
			distance = dist(particles[j].x,particles[j].y,map_landmarks.landmark_list[n].x_f,map_landmarks.landmark_list[n].y_f);

			if (distance <= sensor_range) {

				pred_landmark.id = map_landmarks.landmark_list[n].id_i;
				pred_landmark.x  = map_landmarks.landmark_list[n].x_f;
				pred_landmark.y  = map_landmarks.landmark_list[n].y_f;

				pred_landmark_vector.push_back(pred_landmark);

			}

		}

		// data association
		// the obs_landmark_vector value will be changed 
		dataAssociation(pred_landmark_vector,obs_landmark_vector);

		// get association data
		std::vector<int> associations;
		std::vector<double> x_a,y_a;
		for (int n_l = 0;n_l < obs_landmark_vector.size();n_l++){
			
			associations.push_back(obs_landmark_vector[n_l].id);
			x_a.push_back(obs_landmark_vector[n_l].x);
			y_a.push_back(obs_landmark_vector[n_l].y);
		}

		// set associations 
		SetAssociations(particles[j],associations,x_a,y_a);

		// calculate the particle weight
		double sig_x = std_landmark[0];
		double sig_y = std_landmark[1];
		// calculate normalization term
		double gauss_norm = (1 / (2 * M_PI * sig_x * sig_y) );
		double prob = 1.0;

		for (int n_a=0;n_a < particles[j].associations.size();n_a ++) {

			double mu_x = get_landmark_map(particles[j].associations[n_a],map_landmarks).x;
			double mu_y = get_landmark_map(particles[j].associations[n_a],map_landmarks).y;
			double x_obs = particles[j].sense_x[n_a];
			double y_obs = particles[j].sense_y[n_a];
			// calculate the exponent
			double exponent = pow((x_obs - mu_x),2)/(2 * pow(sig_x,2)) +pow((y_obs - mu_y),2)/(2*pow(sig_y,2));
			// calculate weight using normalization term and exponent
			double part_weight =  gauss_norm * exp(- exponent);
			// multiply all the calculated measurement probilities together
			prob  *= part_weight;
		}

		// update the weight
		particles[j].weight = prob ;

	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<Particle> particles_resample;

	double sum_weight = 0;
	double mw = 0;
	// calculate the sum of weights
	for (int i=0;i < num_particles ;i++){

		sum_weight += particles[i].weight;
	}

	//clear the weights
	weights.clear();
	// normalize the weight and get the max weight
	for (int i=0;i < num_particles;i++){

		particles[i].weight = particles[i].weight / sum_weight;

		// get the max weight
		if (i == 0) {

			mw = particles[i].weight;
		} else {

			if (mw <= particles[i].weight) {

				mw = particles[i].weight;
			}
		}

		weights.push_back(particles[i].weight);	
	}

	// resample particles
/*
	// use the resample wheel to resample the particles
	default_random_engine e ;
	uniform_real_distribution<double> u(0,1);

	int index = int(u(e) * num_particles);
	double beta = 0;

	for (int i=0; i < num_particles;i++){

		beta += u(e) * 2 * mw;
		while(particles[index].weight < beta){

			beta = beta - particles[index].weight;
			index = (index + 1) % num_particles;

		}

		particles_resample.push_back(particles[index]);
	}
*/

	// use discrete_distribution to resample the particles
	double index_sample;
	default_random_engine gen;
	std::discrete_distribution<> d{std::begin(weights),std::end(weights)};

	for (int i=0;i< num_particles;i++){

		index_sample = d(gen);
		particles_resample.push_back(particles[index_sample]);

	}

	// update the particles
	particles = particles_resample;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

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
