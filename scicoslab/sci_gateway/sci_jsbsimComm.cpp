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

class JSBSimComm
{
public:
    JSBSimComm(std::string aircraftPath, std::string enginePath,
               std::string systemsPath, std::string modelName,
               double * x0, double * u0) : prop(), fdm(&prop), ss(fdm)
    {
        using namespace JSBSim;
        std::cout << "initializing" << std::endl;
        fdm.LoadModel(aircraftPath,enginePath,systemsPath,modelName);
        fdm.SetDebugLevel(0);
        fdm.Setdt(1./120.);

        // defaults
        bool variablePropPitch = false;

        // get propulsion pointer to determine type/ etc.
        FGEngine * engine0 = fdm.GetPropulsion()->GetEngine(0);
        FGThruster * thruster0 = engine0->GetThruster();

        // state space
        ss.x.add(new FGStateSpace::Vt);
        ss.x.add(new FGStateSpace::Alpha);
        ss.x.add(new FGStateSpace::Theta);
        ss.x.add(new FGStateSpace::Q);

        if (thruster0->GetType()==FGThruster::ttPropeller)
        {
            ss.x.add(new FGStateSpace::Rpm);
            if (variablePropPitch) ss.x.add(new FGStateSpace::Pitch);
        }
        switch (engine0->GetType())
        {
        case FGEngine::etTurbine:
            ss.x.add(new FGStateSpace::N2);
            break;
        case FGEngine::etTurboprop:
            ss.x.add(new FGStateSpace::N1);
            break;
        default:
            break;
        }
        ss.x.add(new FGStateSpace::Beta);
        ss.x.add(new FGStateSpace::Phi);
        ss.x.add(new FGStateSpace::P);
        ss.x.add(new FGStateSpace::R);
        ss.x.add(new FGStateSpace::Alt);
        ss.x.add(new FGStateSpace::Psi);

        ss.x.add(new FGStateSpace::ThrottlePos);
        ss.x.add(new FGStateSpace::DaPos);
        ss.x.add(new FGStateSpace::DePos);
        ss.x.add(new FGStateSpace::DrPos);

        ss.u.add(new FGStateSpace::ThrottleCmd);
        ss.u.add(new FGStateSpace::DaCmd);
        ss.u.add(new FGStateSpace::DeCmd);
        ss.u.add(new FGStateSpace::DrCmd);

        // state feedback
        ss.y = ss.x;

        // set initial conditions
        ss.x.set(x0);
        ss.u.set(u0);

        // make sure it's steady, shouldn't be needed if state is full
        fdm.GetPropulsion()->GetSteadyState();
    }
    virtual ~JSBSimComm()
    {
    }
public:
    JSBSim::FGPropertyManager prop;
    JSBSim::FGFDMExec fdm;
    JSBSim::FGStateSpace ss;
};

extern "C"
{

#include <scicos/scicos_block4.h>
#include <math.h>
#include "definitions.hpp"

    void sci_jsbsimComm(scicos_block *block, scicos::enumScicosFlags flag)
    {
        static JSBSimComm * comm = NULL;
        static JSBSim::FGPropertyManager propManager;

        // data
        double *u=(double*)GetInPortPtrs(block,1);
        double *xOut=(double*)GetOutPortPtrs(block,1);
        double *y=(double*)GetOutPortPtrs(block,2);
        double *x=(double*)GetState(block);
        double *xd=(double*)GetDerState(block);

        //handle flags
        if (flag==scicos::initialize || flag==scicos::reinitialize)
        {
            std::string jsbsimPath = getenv("JSBSim");
            std::string aircraftPath = jsbsimPath + "/aircraft";
            std::string enginePath = jsbsimPath + "/engine";
            std::string systemsPath = jsbsimPath + "/systems";
            std::string modelName = "f16";
            if (comm)
            {
                std::cout << "deleting comm" << std::endl;
                delete comm;
                comm = NULL;
            }
            comm = new JSBSimComm(aircraftPath,enginePath,systemsPath,modelName,x,u);
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
        }
        else if (flag==scicos::computeDeriv)
        {
            comm->fdm.RunIC();
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
