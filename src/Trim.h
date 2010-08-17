/*
 * Trim.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * Trim.h is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * Trim.h is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FGFDMExec.h"
#include <limits>

namespace JSBSim
{

class FGNelderMead
{
public:
	class Function
	{
	public:
		virtual double eval(const vector<double> & v) const = 0;
		virtual ~Function(){};
	};
	FGNelderMead(const Function & f, const std::vector<double> & initialGuess, 
			const std::vector<double> initialStepSize, int iterMax=1000,
			double rtol=1e-6, double speed = 2.0);
	std::vector<double> getSolution();
private:
	// attributes
	const Function & myF;
	int nDim, nVert, iMax, iNextMax, iMin;
	std::vector< std::vector<double> > simplex;
	std::vector<double> cost;
	std::vector<double> elemSum;

	// methods
	double tryStretch(double factor);
};

class FGTrimmer : public FGNelderMead::Function
{
public:
	FGTrimmer(FGFDMExec & fdm);
	void constrain(const vector<double> & v, vector<double> & x) const;
	double eval(const vector<double> & v) const;
private:
	FGFDMExec & myFdm;
};

} // JSBSim

// vim:ts=4:sw=4
