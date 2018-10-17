/*
 * SimulationSettings.hpp
 *
 *  Created on: 16 Oct 2018
 *      Author: Thomas Maters
 *		Email : thomasmaters@hotmail.com (TG.Maters@student.han.nl)
 */

#ifndef SIMULATIONSETTINGS_HPP_
#define SIMULATIONSETTINGS_HPP_

#include "../MultibeamDataProcessor/src/Messages/SensorMessage.hpp"
namespace Messages
{
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
} //Namespace Messages.
#endif /* SIMULATIONSETTINGS_HPP_ */
