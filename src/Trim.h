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
		virtual double eval(const vector<double> & v)  = 0;
		virtual ~Function(){};
	};
	FGNelderMead(Function & f, const std::vector<double> & initialGuess, 
			const std::vector<double> initialStepSize, int iterMax=3000,
			double rtol=1e-6, double speed = 2.0);
	std::vector<double> getSolution();
private:
	// attributes
	Function & m_f;
	int m_nDim, m_nVert, m_iMax, m_iNextMax, m_iMin;
	std::vector< std::vector<double> > m_simplex;
	std::vector<double> m_cost;
	std::vector<double> m_elemSum;

	// methods
	double tryStretch(double factor);
	void constructSimplex(const vector<double> & guess, const vector<double> & stepSize);
};

class FGTrimmer : public FGNelderMead::Function
{
public:
	struct Constraints
	{
		Constraints() :
			velocity(100), altitude(1000), gamma(0), 
			rollRate(0), pitchRate(0), yawRate(0),
			coordinatedTurn(true), stabAxisRoll(true)
		{
		}
		double velocity, altitude, gamma;
		double rollRate, pitchRate, yawRate;
		bool coordinatedTurn, stabAxisRoll;
	};
	FGTrimmer(FGFDMExec & fdm, const Constraints & constraints);
	void constrain(const vector<double> & v);
	double eval(const vector<double> & v);
private:
	FGFDMExec & m_fdm;
	FGInitialCondition * fgic() { return m_fdm.GetIC(); };
	FGPropulsion * propulsion() { return m_fdm.GetPropulsion(); }
	FGFCS * fcs() { return m_fdm.GetFCS(); }
	FGAuxiliary * aux() { return m_fdm.GetAuxiliary(); }
	FGPropagate * propagate() { return m_fdm.GetPropagate(); }
	const Constraints & m_constraints;
};

} // JSBSim

// vim:ts=4:sw=4
