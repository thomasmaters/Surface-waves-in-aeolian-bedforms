/*
 * RippelData.hpp
 *
 *  Created on: 16 Oct 2018
 *      Author: Thomas Maters
 *		Email : thomasmaters@hotmail.com (TG.Maters@student.han.nl)
 */

#ifndef RIPPELDATA_HPP_
#define RIPPELDATA_HPP_

#define RD_X_LOW 0
#define RD_X_HIGH 1
#define RD_Y_LOW 2
#define RD_Y_HIGH 3
#define RD_HEADER_SIZE 4

#include "../MultibeamDataProcessor/src/Messages/SensorMessage.hpp"
namespace Messages
{
class RippelData : public Messages::SensorMessage
{
public:
	RippelData(): Messages::SensorMessage(RD_HEADER_SIZE + TOTAL_SIZE)
	{
	    data_[RD_X_HIGH] = (I_SIZE >> 8);
	    data_[RD_X_LOW] = (I_SIZE & 0xFF);
	    data_[RD_Y_HIGH] = (J_SIZE >> 8);
	    data_[RD_Y_LOW] = (J_SIZE & 0xFF);
	}

	virtual ~RippelData()
	{

	}
};

} //Namespace Messages


#endif /* RIPPELDATA_HPP_ */
