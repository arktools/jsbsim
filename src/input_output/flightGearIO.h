/*
 * flightGearIO.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * flightGearIO.h is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * flightGearIO.h is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef JSBSim_flightGearIO_H
#define JSBSim_flightGearIO_H

#include "FGFDMExec.h"
#include "math/FGStateSpace.h"
#include "input_output/FGfdmSocket.h"
#include "net_fdm.hxx"

namespace JSBSim
{

static void htond (double &x);
static void htonf (float &x);
void JSBSim2FlightGearNetFDM(FGFDMExec & fdm, FGNetFDM & net, bool netByteOrder=true);

} // JSBSim

#endif

// vim:ts=4:sw=4
