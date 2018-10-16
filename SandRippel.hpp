/*
 * SandRippel.hpp
 *
 *  Created on: 16 Oct 2018
 *      Author: Thomas Maters
 *		Email : thomasmaters@hotmail.com (TG.Maters@student.han.nl)
 */

#ifndef SANDRIPPEL_HPP_
#define SANDRIPPEL_HPP_

#include <cmath>
#include "../MultibeamDataProcessor/src/Communication/RequestResponseHandler.hpp"
#include "SimulationSettings.hpp"

class SandRippel: public Communication::RequestHandler
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

    /**
     * Handler for when we got a UDP message.
     * @param data
     * @param length
     * @param time
     * @return
     */
    virtual Messages::SensorMessage handleRequest(uint8_t* data, std::size_t length, std::chrono::milliseconds::rep time = 0)
    {
    	//Is it a Simulation settings message.
    	if(length == SIMULATION_SETTINGS_MESSAGE_SIZE)
    	{
    		SimulationSettings settings(data, time);
    		std::cout << "Direction: " << settings.getDirection() << " Strength: " << settings.getStrength() << " D:" << settings.getD() << " Beta:" << settings.getBeta() << std::endl;

    		d = settings.getD();
    		beta = settings.getBeta();

    		a_kl[1][0] = settings.getStrength();
    		a_kl[0][0] = (1.0/6) * (1 - a_kl[1][0]);
    		a_kl[2][0] = (1.0/6) * (1 - a_kl[1][0]);
    		a_kl[0][1] = ((2.0/3) * (1 - a_kl[1][0]))/5;
    		a_kl[0][2] = ((2.0/3) * (1 - a_kl[1][0]))/5;
    		a_kl[1][2] = ((2.0/3) * (1 - a_kl[1][0]))/5;
    		a_kl[2][2] = ((2.0/3) * (1 - a_kl[1][0]))/5;
    		a_kl[2][1] = ((2.0/3) * (1 - a_kl[1][0]))/5;

    		rotateArrayValues(std::ceil(static_cast<float>(settings.getDirection()) / 45.0));
    		bleedArrayValues(static_cast<float>((settings.getDirection() % 45))/45);
    	}
    	else
    	{
    		std::cout << "Length mismatch expected: " << SIMULATION_SETTINGS_MESSAGE_SIZE << " got: " << length << std::endl;
    	}
    	return Messages::SensorMessage(0);
    }

    /**
     * Let some values of the kernel bleed into each other.
     * @param percentage_to_right
     */
    void bleedArrayValues(float percentage_to_right)
    {
    	if(percentage_to_right == 0)
    	{
    		return;
    	}
    	std::array<std::array<float,3>,3> temp{{{{0,0,0}},{{0,0,0}},{{0,0,0}}}};
    	temp[0][0] = a_kl[0][1] * (1.0 - percentage_to_right) + a_kl[0][0] * percentage_to_right;
    	temp[0][1] = a_kl[0][2] * (1.0 - percentage_to_right) + a_kl[0][1] * percentage_to_right;
    	temp[0][2] = a_kl[1][2] * (1.0 - percentage_to_right) + a_kl[0][2] * percentage_to_right;
    	temp[1][2] = a_kl[2][2] * (1.0 - percentage_to_right) + a_kl[1][2] * percentage_to_right;
    	temp[2][2] = a_kl[2][1] * (1.0 - percentage_to_right) + a_kl[2][2] * percentage_to_right;
    	temp[2][1] = a_kl[2][0] * (1.0 - percentage_to_right) + a_kl[2][1] * percentage_to_right;
    	temp[2][0] = a_kl[1][0] * (1.0 - percentage_to_right) + a_kl[2][0] * percentage_to_right;
    	temp[1][0] = a_kl[0][0] * (1.0 - percentage_to_right) + a_kl[1][0] * percentage_to_right;
    	a_kl = temp;
    }

    /**
     * Rotate te values in the kernel around the center.
     * @param amount
     */
    void rotateArrayValues(std::size_t amount)
    {
    	if(amount == 0)
    	{
    		return;
    	}
		float temp;
    	for (std::size_t i = 0; i < amount; ++i) {
    		temp = a_kl[0][0];
    		a_kl[0][0] = a_kl[1][0];
    		a_kl[1][0] = a_kl[2][0];
    		a_kl[2][0] = a_kl[2][1];
    		a_kl[2][1] = a_kl[2][2];
    		a_kl[2][2] = a_kl[1][2];
    		a_kl[1][2] = a_kl[0][2];
    		a_kl[0][2] = a_kl[0][1];
    		a_kl[0][1] = temp;
		}
    }

    /**
     * Generate a new 'tick' of the simulation.
     * @return
     */
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
		return d * (sum - lattice_ij[i][j]);
    }

    float func_delta_2(float i, float j)
    {
    	return beta * std::tanh(lattice_ij[i][j]) - lattice_ij[i][j];
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
    std::array<std::array<float,3>,3> a_kl{{
    {{0.1,0.05,0.05}},
    {{0.55,0.0,0.05}},
    {{0.1,0.05,0.05}}}};

    float d = D_CON;
    float beta = BETA_CON;
};



#endif /* SANDRIPPEL_HPP_ */
