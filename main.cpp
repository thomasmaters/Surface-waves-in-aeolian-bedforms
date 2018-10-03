#include <array>
#include <cmath>
#include <random>
#include <iostream>
#include <chrono>
#include <initializer_list>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define I_SIZE 200
#define J_SIZE 200
#define ITERATIONS 2000

#define D_CON 0.2
#define BETA_CON 1

const std::array<std::array<float,3>,3> a_kl{{
{{0.55,0.1,0.05}},
{{0.1,0.0,0.05}},
{{0.05,0.05,0.05}}}};

class SandRippel
{
public:
    SandRippel()
    {
		for (int i = 0; i < I_SIZE; ++i) {
			for (int j = 0; j < J_SIZE; ++j) {
				lattice_ij[i][j] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) / 20.0;
			}
		}
    }

    virtual ~SandRippel()
    {

    }

    std::array<std::array<float, J_SIZE>, I_SIZE>& nextTick()
    {
	for (std::size_t i = 0; i < I_SIZE; ++i) {
	    for (std::size_t j = 0; j < J_SIZE; ++j) {
		lattice_ij[i][j] += func_delta(i,j);
	    }
	}
	return lattice_ij;
    }

    float func_delta_1(float i, float j)
    {
		std::int32_t x;
		std::int32_t y;
		float sum = 0;
		for (std::int32_t k = -1; k < 2; ++k) {
			for (std::int32_t h = -1; h < 2; ++h) {
				if(k == 0 && h == 0){
					continue;
				}
			x = i + k;
			y = j + h;
			if(x == -1)
			{
				x = I_SIZE - 1;
			}
			else if(x == I_SIZE)
			{
				x = 0;
			}

			if(y == -1)
			{
				y = J_SIZE - 1;
			}
			else if(y == J_SIZE)
			{
				y = 0;
			}
			sum += a_kl[k + 1][h + 1] * lattice_ij[x][y];
	    }
	}

	return D_CON * (sum - lattice_ij[i][j]);
    }

    float func_delta_2(float i, float j)
    {
    	return BETA_CON * std::tanh(lattice_ij[i][j]) - lattice_ij[i][j];
    }

    float func_I(float i, float j)
    {
    	return func_delta_1(i,j) + func_delta_2(i,j);
    }

    float func_delta(float i, float j)
    {
		std::int32_t x;
		std::int32_t y;
		float sum = 0;
		for (std::int32_t k = -1; k < 2; ++k) {
			for (std::int32_t h = -1; h < 2; ++h) {
				if(k == 0 && h == 0){
					continue;
				}
				x = i + k;
				y = j + h;
				if(x == -1)
				{
					x = I_SIZE - 1;
				}
				else if(x == I_SIZE)
				{
					x = 0;
				}

				if(y == -1)
				{
					y = J_SIZE - 1;
				}
				else if(y == J_SIZE)
				{
					y = 0;
				}
				sum += (a_kl[k + 1][h + 1] * func_I(x, y));
			}
		}
		return func_I(i,j) - sum;
    }

    std::array<std::array<float, J_SIZE>, I_SIZE> lattice_ij;
};

long mapValue(float x, float in_min, float in_max, float out_min, float out_max)
{
	if(x < in_min){return out_min;}
	if(x > in_max){return out_max;}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char **argv) {
	cv::Mat image(I_SIZE, J_SIZE, CV_8U);
	cv::Mat image2(I_SIZE, J_SIZE, CV_8U);
    SandRippel rippel_sim;

    for (int i = 0; i < ITERATIONS; ++i)
    {
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		std::array<std::array<float, J_SIZE>, I_SIZE>& rippel_update = rippel_sim.nextTick();

		float lowest = 0;
		float highest = 0;
		for (int k = 0; k < I_SIZE; ++k) {
			for (int j = 0; j < J_SIZE; ++j) {
				if(rippel_update[k][j] < lowest)
				{
					lowest = rippel_update[k][j];
				}
				if(rippel_update[k][j] > highest)
				{
					highest = rippel_update[k][j];
				}
			}
		}
		for (int k = 0; k < I_SIZE; ++k) {
			for (int j = 0; j < J_SIZE; ++j) {
				image.row(k).col(j) = static_cast<uint8_t>(mapValue(rippel_update[k][j], lowest, highest, 0, 255));
			}
		}
		cv::imwrite( "C:\\Projecten\\Eclipse-workspace\\SandRippelSimulation\\Debug\\images\\gray\\" + std::to_string(i) + "_img.jpg", image );

		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds >( t2 - t1 ).count();
		std::cout << "Iteration: " << i << " Duration: " << duration << " high: " << highest << " lowest: " << lowest << std::endl;
    }
    return 0;
}

