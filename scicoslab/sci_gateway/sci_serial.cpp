/*
 * sci_serial.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * sci_serial.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sci_serial.cpp is distributed in the hope that it will be useful, but
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

void read(BufferedAsyncSerial * serial)
{
	std::vector<char> data = serial->read();
	for (int i=0;i<data.size();i++) std::cout << data[i];
	std::cout << std::endl;
}


extern "C"
{

#include <scicos/scicos_block4.h>
#include <math.h>
#include "definitions.hpp"

    void sci_serial(scicos_block *block, scicos::enumScicosFlags flag)
    {
        // data
        char * u=Getint8InPortPtrs(block,1);
        char * y=Getint8OutPortPtrs(block,2);
        int * ipar=block->ipar;

		// serial port
		static BufferedAsyncSerial * serial = NULL;

        //handle flags
        if (flag==scicos::initialize || flag==scicos::reinitialize)
        {
			if (!serial) serial = new BufferedAsyncSerial("/dev/ttyUSB1",38400);
        }
        else if (flag==scicos::terminate)
        {
			if (serial)
			{
				delete serial;
				serial = NULL;
			}
        }
        else if (flag==scicos::updateState)
        {
        }
        else if (flag==scicos::computeDeriv)
        {
        }
        else if (flag==scicos::computeOutput)
        {
			read(serial);
        }
        else
        {
        }
    }

} // extern c

// vim:ts=4:sw=4
