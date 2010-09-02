/*
 * sci_JSBSimComm.cpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * sci_JSBSimComm.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sci_JSBSimComm.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FGFDMExec.h"
#include "models/FGFCS.h"
#include "Trim.h"
#include "FGStateSpace.h"
#include <iostream>
#include <string>

extern "C"
{

#include <scicos/scicos_block4.h>
#include <math.h>
#include "definitions.hpp"

    void sci_jsbSimComm(scicos_block *block, scicos::enumScicosFlags flag)
    {
        using namespace JSBSim;
        //definitions
        static FGFDMExec fdm;
        static FGStateSpace ss(fdm);

        // data
        double *u=(double*)GetInPortPtrs(block,1);
        double *xOut=(double*)GetOutPortPtrs(block,1);
        double *y=(double*)GetOutPortPtrs(block,2);
        double *x=(double*)GetState(block);
        double *xd=(double*)GetDerState(block);

        //handle flags
        if (flag==scicos::initialize || flag==scicos::reinitialize)
        {
            std::cout << "initializing" << std::endl;
            fdm.LoadModel("../aircraft","../engine","../systems","f16");

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
        }
        else if (flag==scicos::terminate)
        {
            std::cout << "terminating" << std::endl;
        }
        else if (flag==scicos::updateState)
        {
            // make sure we have initialized first

            // update the state
            //fdm->updateState(x);
        }
        else if (flag==scicos::computeDeriv)
        {
            // update the state
            //sci_JSBSimComm(block,scicos::updateState);

            // run the communications
            //fdm->calculateXd(x,u,xd);
        }
        else if (flag==scicos::computeOutput)
        {
            // calculate the output
            //fdm->calculateY(x,u,y,xOut);
        }
        else
        {
            std::cout << "unhandled flag: " << flag << std::endl;
        }
    }

} // extern c

// vim:ts=4:sw=4
