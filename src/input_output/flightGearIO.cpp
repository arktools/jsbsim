/*
 * flightGearIO.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * flightGearIO.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * flightGearIO.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "flightGearIO.h"

#include "models/FGAuxiliary.h"
#include "models/FGAircraft.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"
#include "models/propulsion/FGTurbine.h"
#include "models/propulsion/FGTurboProp.h"
#include "models/propulsion/FGPropeller.h"
#include "models/propulsion/FGTank.h"
#include "models/propulsion/FGThruster.h"
#include "models/FGFCS.h"

namespace JSBSim
{

void htond (double &x)
{
    if ( sgIsLittleEndian() )
    {
        int    *Double_Overlay;
        int     Holding_Buffer;

        Double_Overlay = (int *) &x;
        Holding_Buffer = Double_Overlay [0];

        Double_Overlay [0] = htonl (Double_Overlay [1]);
        Double_Overlay [1] = htonl (Holding_Buffer);
    }
    else
    {
        return;
    }
}

void htonf (float &x)
{
    if ( sgIsLittleEndian() )
    {
        int    *Float_Overlay;
        int     Holding_Buffer;

        Float_Overlay = (int *) &x;
        Holding_Buffer = Float_Overlay [0];

        Float_Overlay [0] = htonl (Holding_Buffer);
    }
    else
    {
        return;
    }
}

void JSBSim2FlightGearNetFDM(FGFDMExec & fdm, FGNetFDM & net, bool netByteOrder)
{
    unsigned int i;

    // Version sanity checking
    net.version = FG_NET_FDM_VERSION;

    // Aero parameters
    net.longitude = fdm.GetPropagate()->GetLongitude();
    net.latitude = fdm.GetPropagate()->GetLatitude();
    net.altitude = fdm.GetPropagate()->GetAltitudeASLmeters();
    double feet2meter = 0.3048;
    net.agl = fdm.GetPropagate()->GetDistanceAGL() * feet2meter;
    net.phi = fdm.GetPropagate()->GetEuler(1);
    net.theta = fdm.GetPropagate()->GetEuler(2);
    net.psi = fdm.GetPropagate()->GetEuler(3);
    net.alpha = fdm.GetAuxiliary()->Getalpha();
    net.beta = fdm.GetAuxiliary()->Getbeta();
    net.phidot = fdm.GetAuxiliary()->GetEulerRates(1);
    net.thetadot = fdm.GetAuxiliary()->GetEulerRates(2);
    net.psidot = fdm.GetAuxiliary()->GetEulerRates(3);

    net.vcas = fdm.GetAuxiliary()->GetVcalibratedKTS();
    net.climb_rate = fdm.GetPropagate()->Gethdot();

    net.v_north = fdm.GetPropagate()->GetVel(1);
    net.v_east = fdm.GetPropagate()->GetVel(2);
    net.v_down = fdm.GetPropagate()->GetVel(3);
    net.v_wind_body_north = fdm.GetAuxiliary()->GetAeroUVW(1);
    net.v_wind_body_east = fdm.GetAuxiliary()->GetAeroUVW(2);
    net.v_wind_body_down = fdm.GetAuxiliary()->GetAeroUVW(3);

    net.A_X_pilot = fdm.GetAircraft()->GetBodyAccel(1);
    net.A_Y_pilot = fdm.GetAircraft()->GetBodyAccel(2);
    net.A_Z_pilot = fdm.GetAircraft()->GetBodyAccel(3);

    net.stall_warning = 0;
    net.slip_deg = fdm.GetAuxiliary()->Getbeta(ofDeg);

    // Engine parameters
    net.num_engines = fdm.GetPropulsion()->GetNumEngines();
    for ( i = 0; i < net.num_engines; ++i )
    {
        FGEngine * engine = fdm.GetPropulsion()->GetEngine(i);
        if ( engine->GetRunning() )
        {
            net.eng_state[i] = 2;
        }
        else if ( engine->GetCranking() )
        {
            net.eng_state[i] = 1;
        }
        else
        {
            net.eng_state[i] = 0;
        }
        net.rpm[i] = engine->GetThruster()->GetRPM();
        net.fuel_flow[i] = engine->GetFuelFlowRate();
        net.fuel_px[i] = 0; // psi
        net.egt[i] = 0; // degf
        net.cht[i] = 0; // degf
        net.mp_osi[i] = 0; // osi
        net.tit[i] = 0;
        net.oil_temp[i] = 0; // degf
        net.oil_px[i] = 0; // psi
    }

    // Consumables
    net.num_tanks = fdm.GetPropulsion()->GetNumTanks();
    for ( i = 0; i < net.num_tanks; ++i )
    {
        net.fuel_quantity[i] = fdm.GetPropulsion()->GetTank(i)->GetContents();
    }

    // Gear
    net.num_wheels = 0;
    for (i = 0; i < net.num_wheels; ++i )
    {
        net.wow[i] = 0;
        net.gear_pos[i] = 0; // norm
        net.gear_steer[i] = 0; // norm
        net.gear_compression[i] = 0; // norm
    }

    // the following really aren't used in this context
    net.cur_time = fdm.GetSimTime();
    net.warp = 0;
    net.visibility = 20000; //m

    // Control surface positions
    net.elevator = fdm.GetFCS()->GetDePos(ofNorm);
    net.elevator_trim_tab = 0;
    net.left_flap = fdm.GetFCS()->GetDfPos(ofNorm);
    net.right_flap = fdm.GetFCS()->GetDfPos(ofNorm);
    net.left_aileron = fdm.GetFCS()->GetDaLPos(ofNorm);
    net.right_aileron = fdm.GetFCS()->GetDaRPos(ofNorm);
    net.rudder = fdm.GetFCS()->GetDrPos(ofNorm);
    net.nose_wheel = 0; // norm
    net.speedbrake = 0; // norm
    net.spoilers = 0; // norm

    if ( netByteOrder )
    {
        // Convert the net.buffer to net.ork format
        net.version = htonl(net.version);

        htond(net.longitude);
        htond(net.latitude);
        htond(net.altitude);
        htonf(net.agl);
        htonf(net.phi);
        htonf(net.theta);
        htonf(net.psi);
        htonf(net.alpha);
        htonf(net.beta);

        htonf(net.phidot);
        htonf(net.thetadot);
        htonf(net.psidot);
        htonf(net.vcas);
        htonf(net.climb_rate);
        htonf(net.v_north);
        htonf(net.v_east);
        htonf(net.v_down);
        htonf(net.v_wind_body_north);
        htonf(net.v_wind_body_east);
        htonf(net.v_wind_body_down);

        htonf(net.A_X_pilot);
        htonf(net.A_Y_pilot);
        htonf(net.A_Z_pilot);

        htonf(net.stall_warning);
        htonf(net.slip_deg);

        for ( i = 0; i < net.num_engines; ++i )
        {
            net.eng_state[i] = htonl(net.eng_state[i]);
            htonf(net.rpm[i]);
            htonf(net.fuel_flow[i]);
            htonf(net.fuel_px[i]);
            htonf(net.egt[i]);
            htonf(net.cht[i]);
            htonf(net.mp_osi[i]);
            htonf(net.tit[i]);
            htonf(net.oil_temp[i]);
            htonf(net.oil_px[i]);
        }
        net.num_engines = htonl(net.num_engines);

        for ( i = 0; i < net.num_tanks; ++i )
        {
            htonf(net.fuel_quantity[i]);
        }
        net.num_tanks = htonl(net.num_tanks);

        for ( i = 0; i < net.num_wheels; ++i )
        {
            net.wow[i] = htonl(net.wow[i]);
            htonf(net.gear_pos[i]);
            htonf(net.gear_steer[i]);
            htonf(net.gear_compression[i]);
        }
        net.num_wheels = htonl(net.num_wheels);

        net.cur_time = htonl( net.cur_time );
        net.warp = htonl( net.warp );
        htonf(net.visibility);

        htonf(net.elevator);
        htonf(net.elevator_trim_tab);
        htonf(net.left_flap);
        htonf(net.right_flap);
        htonf(net.left_aileron);
        htonf(net.right_aileron);
        htonf(net.rudder);
        htonf(net.nose_wheel);
        htonf(net.speedbrake);
        htonf(net.spoilers);
    }
}

} // JSBSim


// vim:ts=4:sw=4
