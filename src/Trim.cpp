/*
 * Trim.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * Trim.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * Trim.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FGFDMExec.h"
#include "Trim.h"
#include "models/FGFCS.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"
#include "models/propulsion/FGThruster.h"
#include "models/FGAuxiliary.h"
#include "models/FGAircraft.h"
#include <limits>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include "FGStateSpace.h"

namespace JSBSim
{

FGNelderMead::FGNelderMead(Function & f, const std::vector<double> & initialGuess,
                           const std::vector<double> & lowerBound,
                           const std::vector<double> & upperBound,
                           const std::vector<double> initialStepSize, int iterMax,
                           double rtol, double abstol, double speed, bool showConvergeStatus,
                           bool showSimplex, bool pause) :
        m_f(f), m_lowerBound(lowerBound), m_upperBound(upperBound),
        m_nDim(initialGuess.size()), m_nVert(m_nDim+1),
        m_iMax(1), m_iNextMax(1), m_iMin(1),
        m_simplex(m_nVert), m_cost(m_nVert), m_elemSum(m_nDim),
        m_showSimplex(showSimplex)
{
    // setup
    std::cout.precision(3);
    double rtolI = 0;
    double minCostPrevResize = 0, minCost = 0;
    double minCostPrev = 0, maxCost = 0, nextMaxCost = 0;
    int iter = 0;
	double costTryPrev = 0, costTryPrevPrev = 0;

    // solve simplex
    while (1)
    {
        // reinitialize simplex whenever rtol condition is met
        if ( rtolI < rtol || iter == 0)
        {
            std::vector<double> guess(m_nDim);
            if (iter == 0)
            {
                //std::cout << "constructing simplex" << std::endl;
                guess = initialGuess;
            }
            else
            {
                if (std::abs(minCost-minCostPrevResize) < 1e-20)
                {
                    std::cout << "\nunable to escape local minimum" << std::endl;
                    break;
                }
                //std::cout << "reinitializing step size" << std::endl;
                guess = m_simplex[m_iMin];
                minCostPrevResize = minCost;
            }
            constructSimplex(guess,initialStepSize);
        }

        // find vertex costs
        for (int vertex=0;vertex<m_nVert;vertex++)
        {
            m_cost[vertex] = m_f.eval(m_simplex[vertex]);
        }

        // find max cost, next max cost, and min cost
        m_iMax = m_iNextMax = m_iMin = 0;
        for (int vertex=0;vertex<m_nVert;vertex++)
        {
            if ( m_cost[vertex] > m_cost[m_iMax] )
            {
                m_iMax = vertex;
            }
            else if ( m_cost[vertex] > m_cost[m_iNextMax] || m_iMax == m_iNextMax ) m_iNextMax = vertex;
            else if ( m_cost[vertex] < m_cost[m_iMin] ) m_iMin = vertex;

        }

        // compute relative tolerance
        rtolI = 2*std::abs(m_cost[m_iMax] -
                           m_cost[m_iMin])/(std::abs(m_cost[m_iMax]+std::abs(m_cost[m_iMin])+
                                                     std::numeric_limits<double>::epsilon()));

        // check for max iteratin break condition
        if (iter > iterMax)
        {
            std::cout << "\nmax iterations exceeded" << std::endl;
            break;

        }
        // check for convergence break condition
        else if ( m_cost[m_iMin] < abstol )
        {
            std::cout << "\nsimplex converged" << std::endl;
            break;
        }

        // compute element sum of simplex vertices
        for (int dim=0;dim<m_nDim;dim++)
        {
            m_elemSum[dim] = 0;
            for (int vertex=0;vertex<m_nVert;vertex++)
                m_elemSum[dim] += m_simplex[vertex][dim];
        }

        // min and max costs
        minCostPrev = minCost;
        minCost = m_cost[m_iMin];
        maxCost = m_cost[m_iMax];
        nextMaxCost = m_cost[m_iNextMax];

        // output cost and simplex
        if (showConvergeStatus)
        {
            if ( (minCostPrev + std::numeric_limits<float>::epsilon() )
                    < minCost && minCostPrev != 0)
            {
                std::cout << "\twarning: simplex cost increased"
                          << std::scientific
                          << "\n\tcost: " << minCost
                          << "\n\tcost previous: " << minCostPrev
                          << std::fixed << std::endl;
            }

            std::cout << "\ti: " << iter
                      << std::scientific
                      << "\tcost: " << m_cost[m_iMin]
                      << "\trtol: " << rtolI
                      << std::fixed
                      << "\talpha: " << m_simplex[m_iMin][2]*180/M_PI
                      << "\tbeta: " << m_simplex[m_iMin][5]*180/M_PI
                      << "\tthrottle: " << m_simplex[m_iMin][0]
                      << "\televator: " << m_simplex[m_iMin][1]
                      << "\taileron: " << m_simplex[m_iMin][3]
                      << "\trudder: " << m_simplex[m_iMin][4]
                      << std::endl;
        }
        if (showSimplex)
        {
            std::cout << "simplex: " << std::endl;;
            for (int j=0;j<m_nVert;j++)
                std::cout << "\t" << std::scientific
                          << std::setw(10) << m_cost[j];
            std::cout << std::endl;
            for (int j=0;j<m_nVert;j++) std::cout << "\t\t" << j;
            std::cout << std::endl;
            for (int i=0;i<m_nDim;i++)
            {
                for (int j=0;j<m_nVert;j++)
                    std::cout << "\t" << std::setw(10) << m_simplex[j][i];
                std::cout << std::endl;
            }
            std::cout << std::fixed
                      << "\n\tiMax: " <<  m_iMax
                      << "\t\tiNextMax: " <<  m_iNextMax
                      << "\t\tiMin: " <<  m_iMin << std::endl;
        }
        if (pause)
        {
            std::cout << "paused, press any key to continue" << std::endl;
            std::cin.get();
        }

		// costs

        // try inversion
        double costTry = tryStretch(-1.0);

        // if lower cost than best, then try further stretch by speed factor
		if ( abs(costTry-costTryPrevPrev) < 10*std::numeric_limits<double>::epsilon())
		{
			// stuck so contract
			contract();
		}
		else if (costTry < minCost)
        {
        	costTry = tryStretch(speed);
        }
        // otherwise try a contraction
        else if (costTry > nextMaxCost)
        {
            // 1d contraction
            costTry = tryStretch(1./speed);

            // if greater than max cost, contract about min
            if (costTry > maxCost)
            {
                if (showSimplex)
                    std::cout << "multiD contraction about: " << m_iMin << std::endl;
				contract();
            }
        }
		
		// iteration
        iter++;
		costTryPrev = costTry;
		costTryPrevPrev = costTryPrev;
    }
    std::cout << "\ti\t: " << iter << std::endl;
    std::cout << std::scientific;
    std::cout << "\trtol\t: " << rtolI << std::endl;
    std::cout << "\tcost\t: " << m_cost[m_iMin] << std::endl;
    std::cout << std::fixed;
}

std::vector<double> FGNelderMead::getSolution()
{
    return m_simplex[m_iMin];
}

double FGNelderMead::tryStretch(double factor)
{
    // create trial vertex
    double a= (1.0-factor)/m_nDim;
    double b = a - factor;
    std::vector<double> tryVertex(m_nDim);
    for (int dim=0;dim<m_nDim;dim++)
    {
        tryVertex[dim] = m_elemSum[dim]*a - m_simplex[m_iMax][dim]*b;
        boundVertex(tryVertex,m_lowerBound,m_upperBound);
    }

    // find trial cost
    double costTry0 = m_f.eval(tryVertex);
    double costTry = m_f.eval(tryVertex);

    if (std::abs(costTry0-costTry) > std::numeric_limits<float>::epsilon())
    {
        std::cout << "\twarning: dynamics not stable!" << std::endl;
        return 2*m_cost[m_iMax];
    }

    // if trial cost lower than max
    if (costTry < m_cost[m_iMax])
    {
        // update the element sum of the simplex
        for (int dim=0;dim<m_nDim;dim++) m_elemSum[dim] +=
                tryVertex[dim] - m_simplex[m_iMax][dim];
        // replace the max vertex with the trial vertex
        for (int dim=0;dim<m_nDim;dim++) m_simplex[m_iMax][dim] = tryVertex[dim];
        // update the cost
        m_cost[m_iMax] = costTry;
        if (m_showSimplex) std::cout << "stretched\t" << m_iMax << "\tby : " << factor << std::endl;
    }
    return costTry;
}

void FGNelderMead::contract()
{
 	for (int dim=0;dim<m_nDim;dim++)
	{
		for (int vertex=0;vertex<m_nVert;vertex++)
		{
			m_simplex[vertex][dim] =
				0.5*(m_simplex[vertex][dim] +
					 m_simplex[m_iMin][dim]);
		}
	}
}

void FGNelderMead::constructSimplex(const std::vector<double> & guess,
                                    const std::vector<double> & stepSize)
{
    for (int vertex=0;vertex<m_nVert;vertex++)
    {
        m_simplex[vertex] = guess;
        std::vector<double> upperBound(guess.size());
        for (int dim=0;dim<m_nDim;dim++) upperBound[dim] = m_upperBound[dim]-stepSize[dim];
        boundVertex(m_simplex[vertex],m_lowerBound,upperBound);
    }
    for (int dim=0;dim<m_nDim;dim++)
    {
        int vertex = dim + 1;
        m_simplex[vertex][dim] += stepSize[dim];
    }
    if (m_showSimplex)
    {
        std::cout << "simplex: " << std::endl;;
        for (int j=0;j<m_nVert;j++) std::cout << "\t\t" << j;
        std::cout << std::endl;
        for (int i=0;i<m_nDim;i++)
        {
            for (int j=0;j<m_nVert;j++)
                std::cout << "\t" << std::setw(10) << m_simplex[j][i];
            std::cout << std::endl;
        }
    }
}

void FGNelderMead::boundVertex(vector<double> & vertex,
                               const vector<double> & lowerBound,
                               const vector<double> & upperBound)
{
    for (int dim=0;dim<m_nDim;dim++)
    {
        if (vertex[dim] > upperBound[dim]) vertex[dim] = upperBound[dim];
        else if (vertex[dim] < lowerBound[dim]) vertex[dim] = lowerBound[dim];
    }
}

FGTrimmer::FGTrimmer(FGFDMExec & fdm, Constraints & constraints) :
        m_fdm(fdm), m_constraints(constraints)
{
    m_fdm.Setdt(1./120.);
}

void FGTrimmer::constrain(const std::vector<double> & v)
{
    // unpack design vector
    double throttle = v[0];
    double elevator = v[1];
    double alpha = v[2];
    double aileron = v[3];
    double rudder = v[4];
    double beta = v[5];

    // initialize constraints
    double vt = m_constraints.velocity;
    double altitude = m_constraints.altitude;
    double phi = 0.0, theta = 0.0, psi = 0.0;
    double p = 0.0, q = 0.0, r= 0.0;

    // precomputation
    double sGam = sin(m_constraints.gamma);
    double sBeta = sin(beta);
    double cBeta = cos(beta);
    double tAlpha = tan(alpha);
    double cAlpha = cos(alpha);

    // rate of climb constraint
    double a = cAlpha*cBeta;
    double b = sin(phi)*sBeta+cos(phi)*sin(alpha)*cBeta;
    theta = atan((a*b+sGam*sqrt(a*a-sGam*sGam+b*b))/(a*a-sGam*sGam));

    // turn coordination constraint, lewis pg. 190
    double gd = 32.17;
    double gc = m_constraints.yawRate*vt/gd;
    a = 1 - gc*tAlpha*sBeta;
    b = sGam/cBeta;
    double c = 1 + gc*gc*cBeta*cBeta;
    phi = atan((gc*cBeta*(a-b*b)+
                b*tAlpha*sqrt(c*(1-b*b)+gc*gc*sBeta*sBeta))/
               (cAlpha*(a*a-b*b*(1+c*tAlpha*tAlpha))));

    // turn rates
    if (m_constraints.rollRate != 0.0) // rolling
    {
        p = m_constraints.rollRate;
        q = 0.0;
        if (m_constraints.stabAxisRoll) // stability axis roll
        {
            r = m_constraints.rollRate*tan(alpha);
        }
        else // body axis roll
        {
            r = m_constraints.rollRate;
        }
    }
    else if (m_constraints.yawRate != 0.0) // yawing
    {
        p = -m_constraints.yawRate*sin(theta);
        q = m_constraints.yawRate*sin(phi)*cos(theta);
        r = m_constraints.yawRate*cos(phi)*cos(theta);
    }
    else if (m_constraints.pitchRate != 0.0) // pitching
    {
        p = 0.0;
        q = m_constraints.pitchRate;
        r = 0.0;
    }

    // state
    fgic()->SetVtrueFpsIC(vt);
    fgic()->SetAlphaRadIC(alpha);
    fgic()->SetThetaRadIC(theta);
    fgic()->SetFlightPathAngleRadIC(m_constraints.gamma);
    fgic()->SetQRadpsIC(q);
    // thrust handled below
    fgic()->SetBetaRadIC(beta);
    fgic()->SetPhiRadIC(phi);
    fgic()->SetPRadpsIC(p);
    fgic()->SetRRadpsIC(r);

    // actuator states handled below

    // nav state
    fgic()->SetAltitudeASLFtIC(altitude);
    fgic()->SetPsiRadIC(psi);
    fgic()->SetLatitudeRadIC(0);
    fgic()->SetLongitudeRadIC(0);

    // apply state
    m_fdm.RunIC();

    // set controls
    fcs()->SetDeCmd(elevator);
    fcs()->SetDaCmd(aileron);
    fcs()->SetDrCmd(rudder);
    double tmin = propulsion()->GetEngine(0)->GetThrottleMin();
    double tmax = propulsion()->GetEngine(0)->GetThrottleMax();
    for (int i=0; i<propulsion()->GetNumEngines(); i++)
    {
        FGEngine * engine = propulsion()->GetEngine(i);
        propulsion()->GetEngine(i)->InitRunning();
        propulsion()->GetEngine(i)->SetTrimMode(true);
        fcs()->SetThrottleCmd(i,tmin+throttle*(tmax-tmin));
    }

    // steady propulsion system
    propulsion()->GetSteadyState();

    //std::cout << "\tthrust: " << propulsion()->GetEngine(0)->GetThruster()->GetThrust();
    //std::cout << "\t\trpm: " << propulsion()->GetEngine(0)->GetThruster()->GetRPM() << std::endl;
    /*std::cout
    	<< std::scientific
    	<< "phi\tdeg\t" << phi*180/M_PI
    	<< "\ttheta\tdeg\t" << theta*180/M_PI
    	<< "\tpsi\tdeg\t" << psi*180/M_PI
    	<< "\tp\trad\t" << p
    	<< "\tq\trad/s\t" << q
    	<< "\tr\trad/s\t" << r
    	<< std::fixed
    	<< std::endl;*/
}

void FGTrimmer::getSolution(const std::vector<double> & v, std::vector<double> & x, std::vector<double> & u)
{
    eval(v);
    m_fdm.RunIC();

    // state
    x[0] = fgic()->GetVtrueFpsIC();
    x[1] = fgic()->GetAlphaRadIC();
    x[2] = fgic()->GetThetaRadIC();
    x[3] = fgic()->GetQRadpsIC();
    x[4] = propulsion()->GetEngine(0)->GetThruster()->GetThrust();
    x[5] = fgic()->GetBetaRadIC();
    x[6] = fgic()->GetPhiRadIC();
    x[7] = fgic()->GetPRadpsIC();
    x[8] = fgic()->GetRRadpsIC();

    // actuator states
    double tmin = propulsion()->GetEngine(0)->GetThrottleMin();
    double tmax = propulsion()->GetEngine(0)->GetThrottleMax();
    double throttlePosNorm = (fcs()->GetThrottlePos(0)-tmin)/(tmax-tmin)/2;
    // TODO: shouldn't have to divide by 2 here, something wrong
    x[9] = throttlePosNorm;
    x[10] = fcs()->GetDePos(ofNorm);
    x[11] = fcs()->GetDaLPos(ofNorm);
    x[12] = fcs()->GetDrPos(ofNorm);

    // nav state
    x[13] = fgic()->GetAltitudeASLFtIC();
    x[14] = fgic()->GetPsiRadIC();
    x[15] = fgic()->GetLatitudeRadIC();
    x[16] = fgic()->GetLongitudeRadIC();

    // input
    u[0] = fcs()->GetThrottleCmd(0);
    u[1] = fcs()->GetDeCmd();
    u[2] = fcs()->GetDaCmd();
    u[3] = fcs()->GetDrCmd();
}

void FGTrimmer::printSolution(const std::vector<double> & v)
{
    eval(v);
    double tmin = propulsion()->GetEngine(0)->GetThrottleMin();
    double tmax = propulsion()->GetEngine(0)->GetThrottleMax();
    double throttlePosNorm = (fcs()->GetThrottlePos(0)-tmin)/(tmax-tmin)/2;
    // TODO: shouldn't have to divide by 2 here, something wrong
    double dt = 1./120;

    double thrust = propulsion()->GetEngine(0)->GetThruster()->GetThrust();
    double elevator = fcs()->GetDePos();
    double aileron = fcs()->GetDaLPos();
    double rudder = fcs()->GetDrPos();
    double throttle = fcs()->GetThrottlePos(0);
    double lat = propagate()->GetLatitudeDeg();
    double lon = propagate()->GetLongitudeDeg();

    // run a setp to compute derivatives
    m_fdm.RunIC();

    double dthrust = (propulsion()->GetEngine(0)->
                      GetThruster()->GetThrust()-thrust)/dt;
    double delevator = (fcs()->GetDePos()-elevator)/dt;
    double daileron = (fcs()->GetDaLPos()-aileron)/dt;
    double drudder = (fcs()->GetDrPos()-rudder)/dt;
    double dthrottle = (fcs()->GetThrottlePos(0)-throttle)/dt;
    double dlat = (propagate()->GetLatitudeDeg()-lat)/dt;
    double dlon = (propagate()->GetLongitudeDeg()-lon)/dt;
    double dvt = (propagate()->GetUVW(1)*propagate()->GetUVWdot(1) +
                  propagate()->GetUVW(2)*propagate()->GetUVWdot(2) +
                  propagate()->GetUVW(3)*propagate()->GetUVWdot(3))/
                 aux()->GetVt(); // from lewis, vtrue dot

    // state
    std::cout << std::setw(10)
              // aircraft state
              << "\naircraft state"
              << "\n\tvt\t\t:\t" << fgic()->GetVtrueFpsIC()
              << "\n\talpha, deg\t:\t" << fgic()->GetAlphaDegIC()
              << "\n\ttheta, deg\t:\t" << fgic()->GetThetaDegIC()
              << "\n\tq, rad/s\t:\t" << fgic()->GetQRadpsIC()
              << "\n\tthrust, lbf\t:\t" << propulsion()->GetEngine(0)->GetThruster()->GetThrust()
              << "\n\tbeta, deg\t:\t" << fgic()->GetBetaDegIC()
              << "\n\tphi, deg\t:\t" << fgic()->GetPhiDegIC()
              << "\n\tp, rad/s\t:\t" << fgic()->GetPRadpsIC()
              << "\n\tr, rad/s\t:\t" << fgic()->GetRRadpsIC()

              // actuator states
              << "\n\nactuator state"
              << "\n\tthrottle, %\t:\t" << 100*throttle
              << "\n\televator, deg\t:\t" << 100*elevator
              << "\n\taileron, deg\t:\t" << 100*aileron
              << "\n\trudder, %\t:\t" << 100*rudder

              // nav state
              << "\n\nnav state"
              << "\n\taltitude, ft\t:\t" << fgic()->GetAltitudeASLFtIC()
              << "\n\tpsi, deg\t:\t" << 100*fgic()->GetPsiRadIC()
              << "\n\tlat, deg\t:\t" << lat
              << "\n\tlon, deg\t:\t" << lon

              // d/dt aircraft state
              << "\n\naircraft d/dt state"
              << std::scientific

              << "\n\td/dt vt\t\t\t:\t" << dvt
              << "\n\td/dt alpha, deg/s\t:\t" << aux()->Getadot()*180/M_PI
              << "\n\td/dt theta, deg/s\t:\t" << aux()->GetEulerRates(2)*180/M_PI
              << "\n\td/dt q, rad/s^2\t\t:\t" << propagate()->GetPQRdot(2)
              << "\n\td/dt thrust, lbf\t:\t" << dthrust
              << "\n\td/dt beta, deg/s\t:\t" << aux()->Getbdot()*180/M_PI
              << "\n\td/dt phi, deg/s\t\t:\t" << aux()->GetEulerRates(1)*180/M_PI
              << "\n\td/dt p, rad/s^2\t\t:\t" << propagate()->GetPQR(1)
              << "\n\td/dt r, rad/s^2\t\t:\t" << propagate()->GetPQR(3)

              // d/dt actuator states
              << "\n\nd/dt actuator state"
              << "\n\td/dt throttle, %\t:\t" << dthrottle
              << "\n\td/dt elevator, deg\t:\t" << delevator
              << "\n\td/dt aileron, deg\t:\t" << daileron
              << "\n\td/dt rudder, %\t\t:\t" << drudder

              // nav state
              << "\n\nd/dt nav state"
              << "\n\td/dt altitude, ft\t:\t" << propagate()->Gethdot()
              << "\n\td/dt psi, deg\t\t:\t" << aux()->GetEulerRates(3)
              << "\n\td/dt lat, deg\t\t:\t" << dlat
              << "\n\td/dt lon, deg\t\t:\t" << dlon
              << std::fixed

              // input
              << "\n\ninput"
              << "\n\tthrottle cmd, %\t:\t" << 100*fcs()->GetThrottleCmd(0)
              << "\n\televator cmd, %\t:\t" << 100*fcs()->GetDeCmd()
              << "\n\taileron cmd, %\t:\t" << 100*fcs()->GetDaCmd()
              << "\n\trudder cmd, %\t:\t" << 100*fcs()->GetDrCmd()
              << std::endl;
}

void FGTrimmer::printState()
{
    double tmin = propulsion()->GetEngine(0)->GetThrottleMin();
    double tmax = propulsion()->GetEngine(0)->GetThrottleMax();
    double throttlePosNorm = (fcs()->GetThrottlePos(0)-tmin)/(tmax-tmin)/2;
    // TODO: shouldn't have to divide by 2 here, something wrong

    // state
    std::cout << std::setw(10)
              << "\n\naircraft state"
              << "\nvt\t\t:\t" << aux()->GetVt()
              << "\nalpha, deg\t:\t" << aux()->Getalpha(ofDeg)
              << "\ntheta, deg\t:\t" << propagate()->GetEuler(2)*180/M_PI
              << "\nq, rad/s\t:\t" << propagate()->GetPQR(2)
              << "\nthrust, lbf\t:\t" << propulsion()->GetEngine(0)->GetThruster()->GetThrust()
              << "\nbeta, deg\t:\t" << aux()->Getbeta(ofDeg)
              << "\nphi, deg\t:\t" << propagate()->GetEuler(1)*180/M_PI
              << "\np, rad/s\t:\t" << propagate()->GetPQR(1)
              << "\nr, rad/s\t:\t" << propagate()->GetPQR(3)

              // actuator states
              << "\n\nactuator state"
              << "\nthrottle, %\t:\t" << 100*throttlePosNorm
              << "\nelevator, %\t:\t" << 100*fcs()->GetDePos(ofNorm)
              << "\naileron, %\t:\t" << 100*fcs()->GetDaLPos(ofNorm)
              << "\nrudder, %\t:\t" << 100*fcs()->GetDrPos(ofNorm)

              // nav state
              << "\n\nnav state"
              << "\naltitude, ft\t:\t" << propagate()->GetAltitudeASL()
              << "\npsi, deg\t:\t" << propagate()->GetEuler(3)*180/M_PI
              << "\nlat, deg\t:\t" << propagate()->GetLatitudeDeg()
              << "\nlon, deg\t:\t" << propagate()->GetLongitudeDeg()

              // input
              << "\n\ninput"
              << "\nthrottle cmd, %\t:\t" << 100*fcs()->GetThrottleCmd(0)
              << "\nelevator cmd, %\t:\t" << 100*fcs()->GetDeCmd()
              << "\naileron cmd, %\t:\t" << 100*fcs()->GetDaCmd()
              << "\nrudder cmd, %\t:\t" << 100*fcs()->GetDrCmd()
              << std::endl;

}

double FGTrimmer::eval(const std::vector<double> & v)
{
    double dvt0=-1;
    double dvt=0;
    double cost = 0;
    for (int iter=0;;iter++)
    {

        constrain(v);
        dvt = (propagate()->GetUVW(1)*propagate()->GetUVWdot(1) +
               propagate()->GetUVW(2)*propagate()->GetUVWdot(2) +
               propagate()->GetUVW(3)*propagate()->GetUVWdot(3))/
              aux()->GetVt(); // from lewis, vtrue dot
        double dalpha = aux()->Getadot();
        double dbeta = aux()->Getbdot();
        double dp = propagate()->GetPQRdot(1);
        double dq = propagate()->GetPQRdot(2);
        double dr = propagate()->GetPQRdot(3);
        cost = dvt*dvt +
               100.0*(dalpha*dalpha + dbeta*dbeta) +
               10.0*(dp*dp + dq*dq + dr*dr);

        if (std::abs(dvt0-dvt) < 10*std::numeric_limits<double>::epsilon())
        {
            //std::cout << "\tcost converged in " << iter << " cycles" << std::endl;
            break;
        }
        else if (iter>100)
        {
            std::cout << "\ncost failed to converge to steady value"
                      << std::scientific
                      << "\ndelta dvt: " << std::abs(dvt0-dvt)
                      << "\nmost likely out of the flight envelope"
                      << "\ncheck constraints and initial conditions"
                      << std::endl;
            exit(1);
        }
        else dvt0=dvt;
    }
    //std::cout << "\tdvt\t: " << dvt;
    //std::cout << "\tdalpha\t: " << dalpha;
    //std::cout << "\tdbeta\t: " << dbeta;
    //std::cout << "\tdp\t: " << dp;
    //std::cout << "\tdq\t: " << dq;
    //std::cout << "\tdr\t: " << dr << std::endl;
    return cost;
}

} // JSBSim

template <class varType>
void prompt(const std::string & str, varType & var)
{
    std::cout << str + " [" << std::setw(10) << var << "]\t: ";
    if (std::cin.peek() != '\n')
    {
        std::cin >> var;
        std::cin.ignore(1000, '\n');
    }
    else std::cin.get();
}

int main (int argc, char const* argv[])
{
    using namespace JSBSim;

    // variables
    FGFDMExec fdm;
    FGTrimmer::Constraints constraints;

    std::cout << "\n==============================================\n";
    std::cout << "\tJSBSim Trimming Utility\n";
    std::cout << "==============================================\n" << std::endl;

    // defaults
    constraints.velocity = 500;
    std::string aircraft="f16";
    double rtol = std::numeric_limits<float>::epsilon();
    double abstol = std::numeric_limits<double>::epsilon();
    double speed = 2.0;
    int iterMax = 2000;
    bool showConvergeStatus = false;
    bool pause = false;
    bool showSimplex = false;
    bool variablePropPitch = false;



    // input
    std::cout << "input ( press enter to accept [default] )\n" << std::endl;

    // load model
    std::cout << "model selection" << std::endl;
    while (1)
    {
        prompt("\taircraft\t\t",aircraft);
        fdm.LoadModel("../aircraft","../engine","../systems",aircraft);
        std::string aircraftName = fdm.GetAircraft()->GetAircraftName();
        if (aircraftName == "")
        {
            std::cout << "\tfailed to load aircraft" << std::endl;
        }
        else
        {
            std::cout << "\tsuccessfully loaded: " <<  aircraftName << std::endl;
            break;
        }
    }

    // get propulsion pointer to determine type/ etc.
    FGEngine * engine0 = fdm.GetPropulsion()->GetEngine(0);
    FGThruster * thruster0 = engine0->GetThruster();

    // flight conditions
    std::cout << "\nflight conditions: " << std::endl;
    prompt("\taltitude, ft\t\t",constraints.altitude);
    prompt("\tvelocity, ft/s\t\t",constraints.velocity);
    prompt("\tgamma, deg\t\t",constraints.gamma);
    if (thruster0->GetType()==FGThruster::ttPropeller)
        prompt("\tvariable prop pitch?\t\t",variablePropPitch);
    constraints.gamma *= M_PI/180;

    // mode menu
    while (1)
    {
        int mode = 0;
        prompt("\tmode < non-turning(0), rolling(1), pitching(2), yawing(3) >",mode);
        constraints.rollRate = 0;
        constraints.pitchRate = 0;
        constraints.yawRate = 0;
        if (mode == 0) break;
        else if (mode == 1)
        {
            prompt("\troll rate, rad/s",constraints.rollRate);
            prompt("\tstability axis roll",constraints.stabAxisRoll);
            break;
        }
        else if (mode == 2)
        {
            prompt("\tpitch rate, rad/s",constraints.pitchRate);
            break;
        }
        else if (mode == 3)
        {
            prompt("\tyaw rate, rad/s",constraints.yawRate);
            break;
        }
        else std::cout << "\tunknown mode: " << mode << std::endl;
    }

    // solver properties
    std::cout << "\nsolver properties: " << std::endl;
    std::cout << std::scientific;
    prompt("\tshow converge status?\t",showConvergeStatus);
    prompt("\tshow simplex?\t\t",showSimplex);
    prompt("\tpause?\t\t\t",pause);
    //prompt("\trelative tolerance\t",rtol);
    //prompt("\tabsolute tolerance\t",abstol);
    //prompt("\tmax iterations\t\t",iterMax);
    //prompt("\tconvergence speed\t",speed);
    std::cout << std::fixed;

    // initial solver state
    int n = 6;
    std::vector<double> initialGuess(n), lowerBound(n), upperBound(n), initialStepSize(n);

    lowerBound[0] = 0; //throttle
    lowerBound[1] = -1; // elevator
    lowerBound[2] = -90*M_PI/180; // alpha
    lowerBound[3] = -1; // aileron
    lowerBound[4] = -1; // rudder
    lowerBound[5] = -90*M_PI/180; // beta

    upperBound[0] = 1; //throttle
    upperBound[1] = 1; // elevator
    upperBound[2] = 90*M_PI/180; // alpha
    upperBound[3] = 1; // aileron
    upperBound[4] = 1; // rudder
    upperBound[5] = 90*M_PI/180; // beta

    initialStepSize[0] = 0.2; //throttle
    initialStepSize[1] = 0.1; // elevator
    initialStepSize[2] = 0.1; // alpha
    initialStepSize[3] = 0.1; // aileron
    initialStepSize[4] = 0.1; // rudder
    initialStepSize[5] = 0.1; // beta

    initialGuess[0] = 0.5; // throttle
    initialGuess[1] = 0; // elevator
    initialGuess[2] = 0; // alpha
    initialGuess[3] = 0; // aileron
    initialGuess[4] = 0; // rudder
    initialGuess[5] = 0; // beta

    // solve
    FGTrimmer trimmer(fdm, constraints);
    FGNelderMead solver(trimmer,initialGuess, lowerBound, upperBound, initialStepSize,
                        iterMax,rtol,abstol,speed,showConvergeStatus,showSimplex,pause);

    // output
    trimmer.printSolution(solver.getSolution()); // this also loads the solution into the fdm

    //std::cout << "\nsimulating flight to determine trim stability" << std::endl;

    //std::cout << "\nt = 5 seconds" << std::endl;
    //for (int i=0;i<5*120;i++) fdm.Run();
    //trimmer.printState();

    //std::cout << "\nt = 10 seconds" << std::endl;
    //for (int i=0;i<5*120;i++) fdm.Run();
    //trimmer.printState();

    std::cout << "\nlinearization: " << std::endl;
    FGStateSpace ss(fdm);

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

    std::vector< std::vector<double> > A,B,C,D;
    std::vector<double> x0 = ss.x.get(), u0 = ss.u.get();
    std::vector<double> y0 = x0; // state feedback
    std::cout << ss << std::endl;

    ss.linearize(x0,u0,y0,A,B,C,D);

    std::cout << std::scientific;
    std::cout << "\nA\n" << A << std::endl;
    std::cout << "\nB\n" << B << std::endl;
    std::cout << "\nC\n" << C << std::endl;
    std::cout << "\nD\n" << D << std::endl;
    std::cout << std::fixed;
}

// vim:ts=4:sw=4
