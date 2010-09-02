/*
 * definitions.hpp
 * Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
 *
 * definitions.hpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * definitions.hpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef definitions_HPP
#define definitions_HPP

namespace scicos
{

enum enumScicosFlags
{
    computeDeriv=0,
    computeOutput=1,
    updateState=2,
    outputTimeDelays=3,
    initialize=4,
    terminate=5,
    reinitialize=6,
    internal=7,
    computeZeroCrossSurfsSetModes=8,
    computeZeroCrossSurfs=9
};

}

#endif

// vim:ts=4:sw=4
