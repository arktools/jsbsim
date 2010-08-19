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
		virtual double eval(const vector<double> & v) = 0;
		virtual ~Function(){};
	};
	FGNelderMead(Function & f, const std::vector<double> & initialGuess, 
			const std::vector<double> initialStepSize, int iterMax=1000,
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
};

namespace stateSpaceEnums
{
	enum{x_vt,x_alpha,x_theta,x_q,x_alt,
		x_beta,x_phi,x_p,x_r,x_psi,x_rpm};
	enum{u_throttle,u_elevator,u_aileron,u_rudder};
}

class FGStateSpace
{
public:
	FGStateSpace(FGFDMExec & fdm);
	void getX(vector<double> & x);
	void getXd(vector<double> & xd);
	void setX(const vector<double> & x);
	void getU(vector<double> & x);
	void setU(const vector<double> & x);
	static int getXSize() { return m_xSize; }
	static int getUSize() { return m_uSize; }
	static void printX(const vector<double> & x);
	static void printXd(const vector<double> & xd);
	static void printU(const vector<double> & u);
	void printX();
	void printXd();
	void printU();
private:
	FGFDMExec & m_fdm;
	const static int m_xSize=11, m_uSize=4;
	const static double propulsionDeltaT = 0.1;
};

class FGTrimmer : public FGNelderMead::Function
{
public:
	enum{v_throtle,v_elevator,v_alpha,
		v_aileron,v_rudder,v_beta};
	struct Constraints
	{
		Constraints() :
			velocity(500), altitude(0), gamma(0),
			rollRate(0), pitchRate(0), yawRate(0),
			coordinatedTurn(true), stabAxisRoll(true)
		{
		}
		double velocity, altitude, gamma;
		double rollRate, pitchRate, yawRate;
		bool coordinatedTurn, stabAxisRoll;
	};
	FGTrimmer(FGFDMExec & fdm, const Constraints & constraints);
	void constrain(const vector<double> & v, vector<double> & x, vector<double> & u);
	double eval(const vector<double> & v);
	static int getVSize() { return m_vSize; }
private:
	FGFDMExec m_fdm;
	FGStateSpace m_ss;
	const Constraints & m_constraints;
	const static int m_vSize = 6;
};

} // JSBSim

// vim:ts=4:sw=4
