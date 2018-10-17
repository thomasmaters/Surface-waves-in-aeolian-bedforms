/*
 * RippelData.hpp
 *
 *  Created on: 16 Oct 2018
 *      Author: Thomas Maters
 *		Email : thomasmaters@hotmail.com (TG.Maters@student.han.nl)
 */

#ifndef RIPPELDATA_HPP_
#define RIPPELDATA_HPP_

#include "../MultibeamDataProcessor/src/Messages/SensorMessage.hpp"
namespace Messages
{
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

} //Namespace Messages


#endif /* RIPPELDATA_HPP_ */
