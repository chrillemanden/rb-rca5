#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

extern std::default_random_engine da_generator;

class Particle
{
public:
    double row;
    double col;
	double orientation;
	double weight;

	//Constructor
	Particle(int loc_row, int loc_col, double orientation) :
		row(loc_row), col(loc_col), orientation(orientation) 
	{
		std::uniform_real_distribution<double> weight_distr(0.0, 1.0);

		weight = weight_distr(da_generator);
	}

};


void initParticles(cv::Mat& map, int N, std::vector<Particle> &particles);

void predictParticles(cv::Mat& map, std::vector<Particle>& particles, double translation, double rotation);

void getParticlesEstimatedPosition(std::vector<Particle>& particles, double &x, double &y);

double findWeightedSum(std::vector<Particle>& particles);

void normaliseWeights(std::vector<Particle>& particles);

void resampleParticles(std::vector<Particle>& particles);
