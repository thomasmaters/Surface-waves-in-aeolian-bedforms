#include <array>
#include <cmath>
#include <random>
#include <iostream>
#include <chrono>
#include <initializer_list>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "../MultibeamDataProcessor/src/Communication/UDP/UDPConnection.hpp"
#include "../MultibeamDataProcessor/src/Communication/IOHandler.hpp"
#include "../MultibeamDataProcessor/src/Messages/SensorMessage.hpp"

#define I_SIZE 200
#define J_SIZE 200
#define TOTAL_SIZE I_SIZE * J_SIZE
#define ITERATIONS 2000
#define SIMULATION_SETTINGS_MESSAGE_SIZE 16
#define SS_STRENGTH_OFFSET 1
#define SS_DIRECTION_LOW 5
#define SS_DIRECTION_HIGH 6
#define SS_D_OFFSET 7
#define SS_BETA_OFFSET 11

#define D_CON 0.2
#define BETA_CON 1

#include <cstring>

class SimulationSettings: public Messages::SensorMessage
{
public:
	SimulationSettings(): Messages::SensorMessage(SIMULATION_SETTINGS_MESSAGE_SIZE)
	{

	}

	SimulationSettings(uint8_t* data, std::chrono::milliseconds::rep time = 0): Messages::SensorMessage(data, SIMULATION_SETTINGS_MESSAGE_SIZE, time)
	{

	}

	float getStrength()
	{
		float value;
		std::memcpy(&value, &(data_[SS_STRENGTH_OFFSET]), sizeof(value));
		return value;
	}

	float getD()
	{
		float value;
		std::memcpy(&value, &(data_[SS_D_OFFSET]), sizeof(value));
		return value;
	}

	float getBeta()
	{
		float value;
		std::memcpy(&value, &(data_[SS_BETA_OFFSET]), sizeof(value));
		return value;
	}

	uint16_t getDirection()
	{
		return static_cast<uint16_t>((data_[SS_DIRECTION_HIGH] << 8) | data_[SS_DIRECTION_LOW]);
	}
};

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

    virtual Messages::SensorMessage handleRequest(uint8_t* data, std::size_t length, std::chrono::milliseconds::rep time = 0)
    {
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

    void bleedArrayValues(float percentage_to_right)
    {
    	std::cout << "Percentage to right: " << percentage_to_right << std::endl;
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

    void rotateArrayValues(std::size_t amount)
    {
    	std::cout << "amount rotate: " << amount << std::endl;
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

class RippelData : public Messages::SensorMessage
{
public:
	RippelData(): Messages::SensorMessage(4 + TOTAL_SIZE)
	{
	    data_[1] = (I_SIZE >> 8);
	    data_[0] = (I_SIZE & 0xFF);
	    data_[3] = (J_SIZE >> 8);
	    data_[2] = (J_SIZE & 0xFF);
	}

	virtual ~RippelData()
	{

	}
};

long mapValue(float x, float in_min, float in_max, float out_min, float out_max)
{
	if(x < in_min){return out_min;}
	if(x > in_max){return out_max;}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char **argv) {
    SandRippel rippel_sim;
    Communication::UDP::UDPServerClient client(Communication::IOHandler::getInstance().getIOService(), "127.0.0.1", "1234", "1233");
    client.addRequestHandler(std::shared_ptr<Communication::RequestHandler>(&rippel_sim));
    Communication::IOHandler::getInstance().startIOService();

    RippelData data;

    for (int i = 0; i < 10000; ++i)
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
				data.getData()[4 + k * I_SIZE + j] = static_cast<uint8_t>(mapValue(rippel_update[k][j], lowest, highest, 0, 255));
			}
		}
		client.sendRequest(data, (std::size_t)0, false);

		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds >( t2 - t1 ).count();
//		std::cout << "Iteration: " << i << " Duration: " << duration << " high: " << highest << " lowest: " << lowest << std::endl;
    }
    Communication::IOHandler::getInstance().stopIOService();
    return 0;
}

