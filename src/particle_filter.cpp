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
#include <set>
#include <cmath> 
#include <array> 

#include "particle_filter.h"

using namespace std;

const int NUM_PARTICLES = 1000;
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	this->num_particles = NUM_PARTICLES;
	//std::vector<Particle> particles;

    //std::normal_distribution<double> gen_x(x, std[0]);
    //std::normal_distribution<double> gen_y(y, std[1]);
    //std::normal_distribution<double> gen_t(theta, std[2]);

	for (int np = 0; np < this->num_particles; np++)
	{
		Particle p;
		p.x = normal_distribution<double>(x, std[0])(gen);
		p.y = normal_distribution<double>(y, std[1])(gen);
		p.theta = normal_distribution<double>(theta, std[2])(gen);
		p.weight = 1.0;
		p.id = np;
		
		this->particles.push_back(p);
	}

	is_initialized = true;
}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	double x, y, t;
	for (int np = 0; np < this->num_particles; np++)
	{
		t = this->particles[np].theta;

		if (fabs(yaw_rate) < 1e-4)
		{
			// sin(t+dt) - sin(t) = 2 * sin((t+dt-t)/2) * cos((t+dt+t)/2) = 2 * sin(dt/2) * cos(t + dt/2)    
			// 2 * sin(dt/2) * cos(t + dt/2) / dt = sin(dt/2)/(dt/2) * cos(t+dt/2) = cos(t+dt/2)
			x = this->particles[np].x + velocity * cos(t + yaw_rate * delta_t);
			y = this->particles[np].y + velocity * sin(t + yaw_rate * delta_t);
		}
		else
		{
			x = this->particles[np].x + velocity / yaw_rate * (sin(t + yaw_rate * delta_t) - sin(t));
			y = this->particles[np].y + velocity / yaw_rate * (cos(t) - cos(t + yaw_rate * delta_t));
		}
		t += yaw_rate * delta_t;

		this->particles[np].x = normal_distribution<double>(x, std_pos[0])(gen);
		this->particles[np].y = normal_distribution<double>(y, std_pos[1])(gen);
		this->particles[np].theta = normal_distribution<double>(t, std_pos[2])(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	vector<int> visited;
	for (unsigned int io = 0; io < observations.size(); io++)
	{
		double min_dist = 1e99;
		int map_id;
		for (unsigned int ip = 0; ip < predicted.size(); ip++)
		{
			double dist = abs(predicted[ip].x - observations[io].x) + abs(predicted[ip].y - observations[io].y);
			bool been_visited = find(visited.begin(), visited.end(), observations[io].id) != visited.end();
			if ((dist < min_dist) & !been_visited)
			{
				min_dist = dist;
				map_id = observations[io].id;
			}
		}
		visited.push_back(map_id);
		//predicted[ip].id = map_id;
		observations[io].id = map_id;
	}
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

	// for every particle, find all map_landmarks predicted locations
	//std::vector<LandmarkObs> predicted
	double Wnorm = 0;
	for (unsigned int ip = 0; ip < (unsigned int)this->num_particles; ip++)
	{
		// use for observations coordinate transformation
		double xp = this->particles[ip].x;
		double yp = this->particles[ip].y;
		double thetap = this->particles[ip].theta;
		this->particles[ip].weight = 1;

		// transformed observations
		std::vector<LandmarkObs> observations_i;
		for (unsigned int io = 0; io < observations.size(); io++)
		{
			double xm = observations[io].x * cos(thetap) - observations[io].y * sin(thetap) + xp;
			double ym = observations[io].x * sin(thetap) + observations[io].y * cos(thetap) + yp;
			int id = observations[io].id;
			observations_i.push_back(LandmarkObs{id, xm, ym});
		}

		// predictions
		std::vector<LandmarkObs> predictions_i;
		for (unsigned im = 0; im < map_landmarks.landmark_list.size(); im++)
		{
			int id = map_landmarks.landmark_list[im].id_i; // Landmark ID
			float x = map_landmarks.landmark_list[im].x_f;; // Landmark x-position in the map (global coordinates)
			float y = map_landmarks.landmark_list[im].y_f;; // Landmark y-position in the map (global coordinates)
			
			// take only landmarks R < sensor_range
			double R = sqrt(pow(x-xp, 2) + pow(y-yp, 2));
			if (R < sensor_range) {
				predictions_i.push_back(LandmarkObs{id, x, y});
			}
		}

		dataAssociation(predictions_i, observations_i);

		// update weights
		default_random_engine gen;		
		for (unsigned int io = 0; io < observations_i.size(); io++)
		{
			double xo = observations_i[io].x;
			double yo = observations_i[io].y;
			double xp, yp;
			for (unsigned int ip = 0; ip < predictions_i.size(); ip++)
			{
				if (predictions_i[ip].id == observations_i[io].id)
				{
					xp = observations_i[io].x;
					yp = observations_i[io].y;
					break;
				}
			}

			double dx = xp - xo;
			double dy = yp - yo;
			double exp_arg = -0.5 * (pow(dx / std_landmark[0], 2) + pow(dy / std_landmark[1], 2));
			double gauss_norm = 1. / (2 * M_PI * std_landmark[0] * std_landmark[1]);
			
			this->particles[ip].weight *= gauss_norm * exp(exp_arg);
			//this->particles[ip].weight *= normal_distribution<double>(xp - xo, std_landmark[0])(gen);
			//this->particles[ip].weight *= normal_distribution<double>(yp - yo, std_landmark[1])(gen);

		}

		Wnorm += this->particles[ip].weight;
	}

	// normalize
	/*for (unsigned int ip = 0; ip < (unsigned int)this->num_particles; ip++){
		this->particles[ip].weight /= Wnorm;
	}*/
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// 1000 particles
	std::array<int, NUM_PARTICLES> init = {0};
	//int init[1000] = {0};
	for (int i = 0; i < this->num_particles; i++)
	{
		init[i] = (int)(1000 * this->particles[i].weight);
	}
	//{2,2,1,1,2,2,1,1,2,2};
	std::discrete_distribution<int> vec(init.begin(), init.end());
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
