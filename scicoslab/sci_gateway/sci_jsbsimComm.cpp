/*
 * sci_jsbsimComm.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * sci_jsbsimComm.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sci_jsbsimComm.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FGFDMExec.h"
#include "models/FGFCS.h"
#include "math/FGStateSpace.h"
#include <iostream>
#include <string>
#include <cstdlib>
#include "input_output/FGPropertyManager.h"
#include "input_output/flightGearIO.h"

namespace JSBSim
{

class JSBSimComm
{
public:
    JSBSimComm(char * aircraftPath, char * enginePath,
               char * systemsPath, char * modelName,
               double * x0, double * u0) : prop(), fdm(&prop), ss(fdm),
 			   socket("localhost",5500,FGfdmSocket::ptUDP)
    {
        std::cout << "initializing" << std::endl;
        fdm.LoadModel(std::string(aircraftPath),
				std::string(enginePath),
				std::string(systemsPath),
				std::string(modelName));
        fdm.SetDebugLevel(0);

        // defaults
        bool variablePropPitch = false;

        // get propulsion pointer to determine type/ etc.
        FGEngine * engine0 = fdm.GetPropulsion()->GetEngine(0);
        FGThruster * thruster0 = engine0->GetThruster();

        // longitudinal states
        ss.x.add(new FGStateSpace::Vt);
        ss.x.add(new FGStateSpace::Alpha);
        ss.x.add(new FGStateSpace::Theta);
        ss.x.add(new FGStateSpace::Q);

        if (thruster0->GetType()==FGThruster::ttPropeller)
        {
            ss.x.add(new FGStateSpace::Rpm);
            if (variablePropPitch) ss.x.add(new FGStateSpace::PropPitch);
        }
        ss.x.add(new FGStateSpace::Alt);

		// lateral states
        ss.x.add(new FGStateSpace::Beta);
        ss.x.add(new FGStateSpace::Phi);
        ss.x.add(new FGStateSpace::P);
        ss.x.add(new FGStateSpace::R);
        ss.x.add(new FGStateSpace::Psi);

		// nav states
        ss.x.add(new FGStateSpace::Longitude);
        ss.x.add(new FGStateSpace::Latitude);

		// input
        ss.u.add(new FGStateSpace::ThrottleCmd);
        ss.u.add(new FGStateSpace::DaCmd);
        ss.u.add(new FGStateSpace::DeCmd);
        ss.u.add(new FGStateSpace::DrCmd);

        // state feedback
        ss.y = ss.x;

		// turn on propulsion
		fdm.GetPropulsion()->InitRunning(-1);

        // set initial conditions
        ss.x.set(x0);
        ss.u.set(u0);
    }
    virtual ~JSBSimComm()
    {
    }
	void sendToFlightGear()
	{
		FGNetFDM netFdm;
		JSBSim2FlightGearNetFDM(fdm,netFdm);
        socket.Send((char *)(& netFdm), sizeof(netFdm));
		//std::cout << ss << std::endl;
		//std::cout << ss.x.getDeriv() << std::endl;
	}
public:
    FGPropertyManager prop;
    FGFDMExec fdm;
    FGStateSpace ss;
	FGfdmSocket socket;
};

} // JSBSim

extern "C"
{

#include <scicos/scicos_block4.h>
#include <math.h>
#include "definitions.hpp"

	void getStringArray(int nStrings, int * ipar, char *** stringArray)
	{
		// get strings
		int iChar = 0, iString=0, iStringChar=0;
		*stringArray = (char**)calloc(nStrings+1,sizeof(char*));
		while(1)
		{
			// if start of a new string
			if (iStringChar==0)
			{
				int n = ipar[iChar];
				//printf("\nnew string of length : %d\n", n);
				(*stringArray)[iString] = (char *)calloc(n+1, sizeof(char));
				iChar = iChar + 1;
			}

			// read character
			char c = ipar[iChar];
			//printf("iString: %d, iChar: %d, Char: %c\n", iString, iChar, c);

			(*stringArray)[iString][iStringChar] = c;	
			//printf("stringArray: %c\n", (*stringArray)[iString][iStringChar]);

			iStringChar = iStringChar + 1;
			iChar = iChar + 1;

			// check for string completion
			if (c==0)
			{
				iStringChar = 0;
				iString = iString + 1;
				if (iString >= nStrings) break; // finished
			}
		}
	}

    void sci_jsbsimComm(scicos_block *block, scicos::enumScicosFlags flag)
    {
        static JSBSim::JSBSimComm * comm = NULL;
        static JSBSim::FGPropertyManager propManager;
		static char * modelName, * aircraftPath, * enginePath, * systemsPath;
		static char ** stringArray;

        // data
        double *u=(double*)GetInPortPtrs(block,1);
        double *xOut=(double*)GetOutPortPtrs(block,1);
        double *y=(double*)GetOutPortPtrs(block,2);
        double *x=(double*)GetState(block);
        double *xd=(double*)GetDerState(block);
		int * ipar=block->ipar;

        //handle flags
        if (flag==scicos::initialize || flag==scicos::reinitialize)
        {
            if (comm)
            {
                std::cout << "deleting comm" << std::endl;
                delete comm;
                comm = NULL;
            }

			getStringArray(4,ipar,&stringArray);
			aircraftPath = stringArray[0];
			enginePath = stringArray[1];
			systemsPath = stringArray[2];
			modelName = stringArray[3];
		
            comm = new JSBSim::JSBSimComm(aircraftPath,enginePath,systemsPath,modelName,x,u);
        }
        else if (flag==scicos::terminate)
        {
            if (comm)
            {
                delete comm;
                comm = NULL;
            }
        }
        else if (flag==scicos::updateState)
        {
			comm->ss.u.set(u);
            comm->ss.x.set(x);
			comm->sendToFlightGear();
		}
        else if (flag==scicos::computeDeriv)
        {
			comm->ss.u.set(u);
            comm->ss.x.set(x);
            comm->ss.x.getDeriv(xd);
        }
        else if (flag==scicos::computeOutput)
        {
            comm->ss.x.get(xOut);
            comm->ss.y.get(y);
        }
        else
        {
            std::cout << "unhandled flag: " << flag << std::endl;
        }
    }

} // extern c

// vim:ts=4:sw=4
