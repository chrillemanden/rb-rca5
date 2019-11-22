
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <vector>
#include <math.h>
//#include <boost/math/constants/constants.hpp>


#include "../localization/localize.h"
#include "../map_util/map_util.h"

void initParticles(cv::Mat &map, int N, std::vector<Particle> &particles)
{
	fillMapEdges(map);

	std::default_random_engine generator;
	
    std::uniform_real_distribution<double> width_distr((map.cols-20.0)/2.0, (map.cols + 20)/2.0);
    std::uniform_real_distribution<double> height_distr((map.rows-20.0) / 2.0, (map.rows + 20.0) / 2.0);
    //std::uniform_real_distribution<double> width_distr(10.0, map.cols-10.0);
    //std::uniform_real_distribution<double> height_distr(10.0, map.rows-10.0);
    //std::uniform_real_distribution<double> orientation_distr(-3.14, 3.14);
    //std::uniform_real_distribution<double> orientation_distr(-0.002 + 0.5 * 3.14, 0.002 + 0.5 * 3.14);
    std::uniform_real_distribution<double> orientation_distr(-0.002, 0.002);

	for (int i = 0; i < N; i++)
	{
		int row = height_distr(generator);
		int col = width_distr(generator);

		// if generated inside obstacle, then generate new
		if (map.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 0))
		{
			i--;
			continue;
		}

        particles.push_back(Particle(row, col, orientation_distr(generator)));
        //particles.push_back(Particle(map.rows/2, map.cols/2, 0));
	}
}

void predictParticles(cv::Mat& map, std::vector<Particle>& particles, double translation, double rotation)
{
	// Apply translation first and then rotation
	// Add noise to the updates as well
	// Translation is a distance moved

    //std::default_random_engine generator;
    std::normal_distribution<double> trans_distr(0.0, 2);
    std::normal_distribution<double> ori_distr(0, 0.2);


	for (auto& p : particles)
    {
        double max_noise = 0.6;
        double noise = trans_distr(da_generator);
        if (noise > max_noise)
        {
            noise = max_noise;
        }
        else if (noise < -max_noise)
        {
            noise = -max_noise;
        }
        double trans_col = p.col + cos(p.orientation) * (translation + noise);
        double trans_row = p.row - sin(p.orientation) * (translation + noise);

		//cv::Point2i translated = cv::Point2i(point.col + cos(point.orientation) * 10, point.row + sin(point.orientation) * -10);
	
        cv::LineIterator line_it(map, cv::Point2i(p.col, p.row), cv::Point2i(trans_col, trans_row));

        bool valid_action = true;
        for (int k = 0; k < line_it.count; line_it++, k++)
        {
            if (map.at<cv::Vec3b>(line_it.pos()) == cv::Vec3b(0, 0, 0))
            {
                valid_action = false;
            }
        }

//        p.row = trans_row;
//        p.col = trans_col;
//        p.orientation += rotation + ori_distr(da_generator);


        if (valid_action)
        {
            p.row = trans_row;
            p.col = trans_col;
            p.orientation += rotation + ori_distr(da_generator);
        }
        else
        {
            p.weight = 0;
            p.orientation += rotation + ori_distr(da_generator);
        }

	}
	
}

void getParticlesEstimatedPosition(std::vector<Particle>& particles, double &x, double &y)
{
    double sum_x = 0;
    double sum_y = 0;

    for (auto& p: particles)
    {
        sum_x += p.col;
        sum_y += p.row;
    }

    x = sum_x / particles.size();
    y = sum_y / particles.size();

}

double findWeightedSum(std::vector<Particle>& particles)
{
	double sum = 0;
	for (auto& p : particles)
	{
		sum += p.weight;
	}
	return sum;
}

void normaliseWeights(std::vector<Particle>& particles)
{
	int N = particles.size();

	double sum = findWeightedSum(particles);

	for (auto& p : particles)
	{
		p.weight = p.weight / sum * (double)particles.size();
		//std::cout << "n weight: " << p.weight << std::endl;
	}
	//std::vector<Particle> new_particles;
}

void resampleParticles(std::vector<Particle>& particles)
{
	int N = particles.size();

	std::vector<Particle> new_particles;

	// Take the best particles
	for (auto& p : particles)
	{
		//for (int i = 0; 1.0/(double)particles.size() * (double)i <= p.weight; i++)
		for (int i = 0; i < floor(p.weight); i++)
		{
			new_particles.push_back(p);
		}	
	}
	
	int n = new_particles.size();

    std::uniform_int_distribution<int> index_distr(0, particles.size()-1);

	// Fill in the remaining particles
	for (int i = 0; i < particles.size() - n; i++)
	{
		new_particles.push_back(particles[index_distr(da_generator)]);
	}

	particles = new_particles;
}
