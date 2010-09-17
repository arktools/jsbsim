/*
 * sci_ardupilotmegaHil.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * sci_ardupilotmegaHil.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sci_ardupilotmegaHil.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "utilities.h"
#include "AsyncSerial.h"
#include <iostream>

class ArdupilotmegaHil
{
public:
	// data
	double roll, pitch, throttle, rudder, wpDistance, bearingError;
	double nextWpAlt, energyError;
	int wpIndex, controlMode;
	
	// commmunication
	BufferedAsyncSerial serial;
	int headerCount;
	bool headerFound;
	std::vector<char> message;
	std::vector<char> header;

	// constants
	static const int packetLength = 18;

	// constructor
	ArdupilotmegaHil(const std::string & device, const int baudRate) :
		roll(), pitch(), throttle(), rudder(), wpDistance(), bearingError(),
		nextWpAlt(), energyError(), wpIndex(), controlMode(),
		serial(device,baudRate), headerCount(0), headerFound(false),
		message(), header()
	{
		header.push_back('A');
		header.push_back('A');
		header.push_back('A');
	}
	
	// read new data
	void read(double * u)
	{
		std::cout << "reading" << std::endl;
		std::vector<char> buffer = serial.read();
		int i = 0;

		// look for header
		if (!headerFound)
		{
			while(i<buffer.size())
			{
				std::cout << buffer[i];
				if(buffer[i++] == header[headerCount]) headerCount++;
				else headerCount = 0;

				// read message
				if (headerCount >= header.size())
				{
					//std::cout << "header found" << std::endl;
					headerCount = 0;
					headerFound = true;
					break;
				}
			}
		}

		// read message
		if (headerFound)
		{
			std::cout << "header found" << std::endl;
			while(i<buffer.size()) message.push_back(buffer[i++]);
			if (message.size() >= packetLength)
			{
				std::cout << "message found" << std::endl;
				// message complete, read into packet
				decode();
				print();
				headerFound = false;
				//std::cout << "message emptying" << std::endl;
				message.clear();
			}
		}

		// set u
		u[0] = throttle;
		u[1] = roll;
		u[2] = pitch;
		u[3] = rudder;
	}

	// if message complete, then decode
	void decode()
	{
		print();
		// message complete, read into packet
		roll = bytesToInt16(message[0],message[1])/100.0;
		pitch = bytesToInt16(message[2],message[3])/100.0;
		throttle = bytesToInt16(message[4],message[5])/100.0;
		rudder = bytesToInt16(message[6],message[7])/100.0;
		wpDistance = bytesToInt16(message[8],message[9]);
		bearingError = bytesToInt16(message[10],message[11]);
		nextWpAlt = bytesToInt16(message[12],message[13])*100.0;
		energyError = bytesToInt16(message[14],message[15]);
		wpIndex = message[16];
		controlMode = message[17];
	}

	void print()
	{
		std::cout
			<< "\nroll:\t\t\t" << roll
			<< "\npitch:\t\t\t" << pitch
			<< "\nthrottle:\t\t" << throttle
			<< "\nrudder:\t\t\t" << rudder
			<< "\nwaypoint distance:\t" << wpDistance
			<< "\nbearing errror:\t\t" << bearingError
			<< "\nnext waypoint alt:\t" << nextWpAlt
			<< "\nenergy error:\t\t" << energyError
			<< "\nwaypoint index:\t\t" << wpIndex
			<< "\ncontrol mode:\t\t" << controlMode
			<< std::endl;
	}
private:
	int16_t bytesToInt16(uint8_t byte0, uint8_t byte1)
	{
		union
		{
			int16_t asInt16;
			uint8_t asUint8[2];
		} data;
		data.asUint8[1] = byte0;
		data.asUint8[0] = byte1;
		return data.asInt16;
	}
};

extern "C"
{

#include <scicos/scicos_block4.h>
#include <math.h>
#include "definitions.hpp"

    void sci_ardupilotmegaHil(scicos_block *block, scicos::enumScicosFlags flag)
    {
        // data
        double * x=GetRealInPortPtrs(block,1);
        double * u=GetRealOutPortPtrs(block,1);
        int * ipar=block->ipar;
		static char * device;
		static int baudRate;
        static char ** stringArray;
        static int * intArray;

        // serial port
        static ArdupilotmegaHil * hil = NULL;

        //handle flags
        if (flag==scicos::initialize || flag==scicos::reinitialize)
        {
            if (!hil)
			{
				getIpars(1,1,ipar,&stringArray,&intArray); 
				device = stringArray[0];
				baudRate = intArray[0];	
				hil = new ArdupilotmegaHil(device,baudRate);
			}
        }
        else if (flag==scicos::terminate)
        {
            if (hil)
            {
                delete hil;
                hil = NULL;
            }
        }
        else if (flag==scicos::updateState)
        {
			x[0] = 0;
        }
        else if (flag==scicos::computeDeriv)
        {
        }
        else if (flag==scicos::computeOutput)
        {
            hil->read(u);
        }
        else
        {
        }
    }

} // extern c

// vim:ts=4:sw=4
