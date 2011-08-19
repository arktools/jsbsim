/*
 * FGTrimmer.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * FGTrimmer.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * FGTrimmer.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FGTrimmer.h"
#include "models/FGFCS.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"
#include "models/propulsion/FGThruster.h"
#include "models/FGAuxiliary.h"
#include "models/FGAircraft.h"
#include <iomanip>
#include <cstdlib>
#include <stdexcept>

namespace JSBSim
{

FGTrimmer::FGTrimmer(FGFDMExec & fdm, Constraints & constraints) :
        m_fdm(fdm), m_constraints(constraints)
{
    m_fdm.Setdt(1./120.);
}

std::vector<double> FGTrimmer::constrain(const std::vector<double> & v)
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
    double phi = 0.0, theta = 0.0, psi = 0.0*M_PI/180.0;
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
    double gd = m_fdm.GetInertial()->gravity();
    double gc = m_constraints.yawRate*vt/gd;
    a = 1 - gc*tAlpha*sBeta;
    b = sGam/cBeta;
    double c = 1 + gc*gc*cBeta*cBeta;
    phi = atan((gc*cBeta*((a-b*b)+
                b*tAlpha*sqrt(c*(1-b*b)+gc*gc*sBeta*sBeta)))/
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
    //fgic()->SetPsiRadIC(psi);
    //fgic()->SetLatitudeRadIC(0);
    //fgic()->SetLongitudeRadIC(0);

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
    std::vector<double> data;
    data.push_back(phi);
    data.push_back(theta);
    return data;
}

void FGTrimmer::printSolution(const std::vector<double> & v)
{
    eval(v);
    double tmin = propulsion()->GetEngine(0)->GetThrottleMin();
    double tmax = propulsion()->GetEngine(0)->GetThrottleMax();
    double throttlePosNorm = (fcs()->GetThrottlePos(0)-tmin)/(tmax-tmin)/2;
    double dt = 1./120;

    double thrust = propulsion()->GetEngine(0)->GetThruster()->GetThrust();
    double elevator = fcs()->GetDePos(ofNorm);
    double aileron = fcs()->GetDaLPos(ofNorm);
    double rudder = fcs()->GetDrPos(ofNorm);
    double throttle = fcs()->GetThrottlePos(0);
    double lat = propagate()->GetLatitudeDeg();
    double lon = propagate()->GetLongitudeDeg();


    // fgic is corrupting rudder position
    //std::cout << "rudder before: " << rudder;
    
    // run a step to compute derivatives
    m_fdm.RunIC();

    //std::cout << "rudder after: " << fcs()->GetDrPos(ofNorm);
    double dthrust = (propulsion()->GetEngine(0)->
                      GetThruster()->GetThrust()-thrust)/dt;
    double delevator = (fcs()->GetDePos(ofNorm)-elevator)/dt;
    double daileron = (fcs()->GetDaLPos(ofNorm)-aileron)/dt;
    double drudder = (fcs()->GetDrPos(ofNorm)-rudder)/dt;
    double dthrottle = (fcs()->GetThrottlePos(0)-throttle)/dt;
    double dlat = (propagate()->GetLatitudeDeg()-lat)/dt;
    double dlon = (propagate()->GetLongitudeDeg()-lon)/dt;
    double dvt = (propagate()->GetUVW(1)*propagate()->GetUVWdot(1) +
                  propagate()->GetUVW(2)*propagate()->GetUVWdot(2) +
                  propagate()->GetUVW(3)*propagate()->GetUVWdot(3))/
                 aux()->GetVt(); // from lewis, vtrue dot

    // reinitialize with correct state
    eval(v);

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
              << "\n\televator, %\t:\t" << 100*elevator
              << "\n\taileron, %\t:\t" << 100*aileron
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
              << "\n\td/dt p, rad/s^2\t\t:\t" << propagate()->GetPQRdot(1)
              << "\n\td/dt r, rad/s^2\t\t:\t" << propagate()->GetPQRdot(3)

              // d/dt actuator states
              << "\n\nd/dt actuator state"
              << "\n\td/dt throttle, %\t:\t" << dthrottle
              << "\n\td/dt elevator, %\t:\t" << delevator
              << "\n\td/dt aileron, %\t\t:\t" << daileron
              << "\n\td/dt rudder, %\t\t:\t" << drudder << "\t<-- error due to fgic?"

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

              // interval method comparison
              << "\n\ninterval method comparison"
              << std::scientific << std::setw(10)
              << "\n\tangle of attack\t:\t" << aux()->Getalpha(ofDeg) << "\twdot: " << propagate()->GetUVWdot(3)
              << "\n\tthrottle\t:\t" << fcs()->GetThrottlePos(0) << "\tudot: " << propagate()->GetUVWdot(1)
              << "\n\tpitch trim\t:\t" << fcs()->GetDePos(ofNorm) << "\tqdot: " << propagate()->GetPQRdot(2)
              << "\n\tsideslip\t:\t" << aux()->Getbeta(ofDeg) << "\tvdot: " << propagate()->GetUVWdot(2)
              << "\n\tailerons\t:\t" << fcs()->GetDaLPos(ofNorm) << "\tpdot: " << propagate()->GetPQRdot(1)
              << "\n\trudder\t\t:\t" << fcs()->GetDrPos(ofNorm) << "\trdot: " << propagate()->GetPQRdot(3)

              << "\n" << std::endl;
}

void FGTrimmer::printState()
{
    double tmin = propulsion()->GetEngine(0)->GetThrottleMin();
    double tmax = propulsion()->GetEngine(0)->GetThrottleMax();
    double throttlePosNorm = (fcs()->GetThrottlePos(0)-tmin)/(tmax-tmin)/2;

    // state
    std::cout << std::setw(10)

              // interval method comparison
              << "\n\ninterval method comparison"
              << "\nAngle of Attack: \t:\t" << aux()->Getalpha(ofDeg) << "\twdot: " << propagate()->GetUVWdot(3)
              << "\nThrottle: \t:\t" << 100*fcs()->GetThrottlePos(0) << "\tudot: " << propagate()->GetUVWdot(1)
              << "\nPitch Trim: \t:\t" << 100*fcs()->GetDePos(ofNorm) << "\tqdot: " << propagate()->GetPQRdot(2)
              << "\nSideslip: \t:\t" << aux()->Getbeta(ofDeg) << "\tvdot: " << propagate()->GetUVWdot(2)
              << "\nAilerons: \t:\t" << 100*fcs()->GetDaLPos(ofNorm) << "\tpdot: " << propagate()->GetPQRdot(1)
              << "\nRudder: \t:\t" << 100*fcs()->GetDrPos(ofNorm) << "\trdot: " << propagate()->GetPQRdot(3)

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

        if (std::abs(dvt0-dvt) < 5*std::numeric_limits<double>::epsilon())
        {
            //std::cout << "\tcost converged in " << iter << " cycles" << std::endl;
            break;
        }
        else if (iter>1000)
        {
            std::cout << "\ncost failed to converge to steady value"
                      << std::scientific
                      << "\ndelta dvt: " << std::abs(dvt0-dvt)
                      << "\nmost likely out of the flight envelope"
                      << "\ncheck constraints and initial conditions"
                      << std::endl;
            throw(std::runtime_error("FGTrimmer: cost failed to converge to steady value, most likely out of flight envelope, check constraints and initial conditions"));
        }
        dvt0=dvt;
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


// vim:ts=4:sw=4:expandtab
