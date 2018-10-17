#define I_SIZE 200
#define J_SIZE 200
#define TOTAL_SIZE I_SIZE * J_SIZE
#define ITERATIONS 10000
#define SIMULATION_SETTINGS_MESSAGE_SIZE 16
#define SS_STRENGTH_OFFSET 1
#define SS_DIRECTION_LOW 5
#define SS_DIRECTION_HIGH 6
#define SS_D_OFFSET 7
#define SS_BETA_OFFSET 11

#define D_CON 0.2
#define BETA_CON 1
#include <array>
#include <random>
#include <iostream>
#include <chrono>
#include <initializer_list>
#include <cstring>

#include "../MultibeamDataProcessor/src/Communication/UDP/UDPConnection.hpp"
#include "../MultibeamDataProcessor/src/Communication/IOHandler.hpp"
#include "../MultibeamDataProcessor/src/Messages/SensorMessage.hpp"
#include "SimulationSettings.hpp"
#include "RippelData.hpp"
#include "SandRippel.hpp"

long mapValue(float x, float in_min, float in_max, float out_min, float out_max)
{
	if(x < in_min){return out_min;}
	if(x > in_max){return out_max;}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char **argv) {
	std::size_t iteraties = ITERATIONS;
	if(argc == 2)
	{
		iteraties = std::atoi(argv[1]);
	}
    Simulation::SandRippel rippel_sim;
    Communication::UDP::UDPServerClient client(Communication::IOHandler::getInstance().getIOService(), "127.0.0.1", "1234", "1233");
    client.addRequestHandler(std::shared_ptr<Communication::RequestHandler>(&rippel_sim));
    Communication::IOHandler::getInstance().startIOService();

    Messages::RippelData data;

    for (std::size_t i = 0; i < iteraties; ++i)
    {
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		std::array<std::array<float, J_SIZE>, I_SIZE>& rippel_update = rippel_sim.nextTick();

		//Get lowest and highest value in arrays.
		float lowest = 0;
		float highest = 0;
		for (std::size_t k = 0; k < I_SIZE; ++k) {
			for (std::size_t j = 0; j < J_SIZE; ++j) {
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

		//Set message data.
		for (std::size_t k = 0; k < I_SIZE; ++k) {
			for (std::size_t j = 0; j < J_SIZE; ++j) {
				data.getData()[4 + k * I_SIZE + j] = static_cast<uint8_t>(mapValue(rippel_update[k][j], lowest, highest, 0, 255));
			}
		}
		client.sendRequest(data, (std::size_t)0, false);

		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds >( t2 - t1 ).count();
		std::cout << "Iteration: " << i << " Duration: " << duration << " high: " << highest << " lowest: " << lowest << std::endl;
    }
    Communication::IOHandler::getInstance().stopIOService();
    return 0;
}

