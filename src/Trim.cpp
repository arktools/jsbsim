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

#include "Trim.h"
#include "FGFDMExec.h"
#include "models/FGAuxiliary.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"
#include "models/propulsion/FGThruster.h"
#include "models/FGPropagate.h"
#include <limits>
#include <cmath>

namespace JSBSim
{

FGNelderMead::FGNelderMead(Function & f, const std::vector<double> & initialGuess, 
		const std::vector<double> initialStepSize, int iterMax,
		double rtol, double speed) :
	m_f(f), m_nDim(initialGuess.size()), m_nVert(m_nDim+1),
	m_iMax(1), m_iNextMax(1), m_iMin(1),
	m_simplex(m_nVert), m_cost(m_nVert), m_elemSum(m_nDim)
{
	// set precision
	std::cout.precision(10);

	// construct simplex
	for (int vertex=0;vertex<m_nVert;vertex++) 
	{
		m_simplex[vertex] = initialGuess;
	}
	for (int dim=0;dim<m_nDim;dim++)
	{
		int vertex = dim + 1;
		m_simplex[vertex][dim] += initialStepSize[dim];
	}

	// solve simplex
	for(int iter=0;;iter++)
	{
		// find vertex costs
		for (int vertex=0;vertex<m_nVert;vertex++)
		{
			m_cost[vertex] = m_f.eval(m_simplex[vertex]);
		}

		// find max cost, next max cost, and min cost
		for (int vertex=0;vertex<m_nVert;vertex++)
		{
			if ( m_cost[vertex] > m_cost[m_iMax] )
			{
				m_iNextMax = m_iMax;
				m_iMax = vertex;
			}
			else if ( m_cost[vertex] < m_cost[m_iMin] ) m_iMin = vertex;
		}

		// compute relative tolerance
		double rtolI = 2*std::abs(m_cost[m_iMax] - 
				m_cost[m_iMin])/(std::abs(m_cost[m_iMax]+std::abs(m_cost[m_iMin])+
				std::numeric_limits<double>::epsilon())); 

		// check for max iteratin break condition
		if (iter > iterMax)
		{
			std::cout << "max iterations exceeded" << std::endl;
			break;
		}
		// check for relative tolernace break condition
		else if (rtolI < rtol)
		{
			std::cout << "simplex converged" << std::endl;
			std::cout << "rtol: " 
				<< std::scientific << rtolI << std::endl;
			break;
		}

		// compute element sum of simplex vertices
		for (int dim=0;dim<m_nDim;dim++)
		{
			m_elemSum[dim] = 0;
			for (int vertex=0;vertex<m_nVert;vertex++) 
				m_elemSum[dim] += m_simplex[vertex][dim];
		}

		// try stretch by -1, (invet max vertex about min face)
		double costTry = tryStretch(-1.0);

		// if lower cost then best, then try further stretch by 2
		if (costTry < m_cost[m_iMin])
		{
			costTry = tryStretch(speed);
		}
		// else if worse than second worst try a contraction
		else if (costTry > m_cost[m_iNextMax])
		{
			costTry = tryStretch(1/speed);
		}

		// output cost and simplex
		std::cout << std::scientific << "cost: " 
			<< m_cost[m_iMin] << std::endl;
		std::cout << "simplex: " << std::endl;
		for (int i=0;i<m_nDim;i++)
		{
			for (int j=0;j<m_nVert;j++) std::cout << 
				std::scientific << m_simplex[j][i] << " ";
			std::cout << std::endl;
		}
	}
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
		tryVertex[dim] = m_elemSum[dim]*a - m_simplex[m_iMax][dim]*b;

	// find trial cost
	double costTry = m_f.eval(tryVertex);

	// if trial cost lower than max
	if (costTry < m_cost[m_iMax])
	{
		// update the element sum of the simplex
		for (int dim=0;dim<m_nDim;dim++) m_elemSum[dim] += 
			tryVertex[dim] - m_simplex[m_iMax][dim];
		// replace the max vertex with the trial vertex
		for (int dim=0;dim<m_nDim;dim++) m_simplex[m_iMax][dim] = tryVertex[dim];
		//std::cout << "simplex stretched by : " << factor << std::endl;
	}
	return costTry;
}

FGTrimmer::FGTrimmer(FGFDMExec & fdm, const Constraints & constraints) :
	m_ss(fdm), m_constraints(constraints)
{
}

void FGTrimmer::constrain(const std::vector<double> & v, std::vector<double> & x, std::vector<double> & u)
{
	using namespace stateSpaceEnums;
	u[u_throttle] = v[v_throtle];
	u[u_elevator] = v[v_elevator];
	u[u_aileron] = v[v_aileron];
	u[u_rudder] = v[v_rudder];
	m_ss.setU(u);
	double alpha = v[v_alpha], beta = v[v_beta];
	double phi = 0, theta = 0, psi = 0;
	double p = 0, q = 0, r = 0;
	double rpm = 10000;
	x[x_vt] = m_constraints.velocity;
	x[x_alpha] = alpha;
	x[x_theta] = theta;
	x[x_q] = q;
	x[x_alt] = m_constraints.altitude;
	x[x_beta] = beta;
	x[x_phi] = phi;
	x[x_p] = p;
	x[x_r] = r;
	x[x_psi] = psi;
	x[x_rpm] = rpm;
}

double FGTrimmer::eval(const std::vector<double> & v)
{
	using namespace stateSpaceEnums;
	std::vector<double> x(FGStateSpace::getXSize());
	std::vector<double> u(FGStateSpace::getUSize());
	std::vector<double> xd(FGStateSpace::getXSize());
	constrain(v,x,u);
	m_ss.setX(x);
	m_ss.setU(u);
	m_ss.getXd(xd);
	double cost = xd[x_vt]*xd[x_vt] +
		100*(xd[x_alpha]*xd[x_alpha] + xd[x_beta]*xd[x_beta]) +
		10*(xd[x_p]*xd[x_p] + xd[x_q]*xd[x_q] + xd[x_r]*xd[x_r]);
	return cost;
}

FGStateSpace::FGStateSpace(FGFDMExec & fdm) :
	m_fdm(fdm)
{
}

void FGStateSpace::getX(vector<double> & x)
{
	using namespace stateSpaceEnums;
	if (x.size()!=m_xSize) std::cerr << 
		"warning: getX, x size" << std::endl;
	x[x_vt] = m_fdm.GetAuxiliary()->GetAeroUVW(1);
	x[x_alpha] = m_fdm.GetAuxiliary()->Getalpha();
	x[x_theta] = m_fdm.GetPropagate()->GetEuler(2);
	x[x_q] = m_fdm.GetAuxiliary()->GetEulerRates(2);
	x[x_alt] = m_fdm.GetPropagate()->GetAltitudeASL();
	x[x_beta] = m_fdm.GetAuxiliary()->Getbeta();
	x[x_phi] = m_fdm.GetPropagate()->GetEuler(1);
	x[x_p] = m_fdm.GetAuxiliary()->GetEulerRates(1);
	x[x_r] = m_fdm.GetAuxiliary()->GetEulerRates(3);
	x[x_psi] = m_fdm.GetPropagate()->GetEuler(3);
	x[x_rpm] = m_fdm.GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();
}

void FGStateSpace::getXd(vector<double> & xd)
{
	using namespace stateSpaceEnums;
	m_fdm.RunIC();
	if (xd.size()!=m_xSize) std::cerr << 
		"warning: getXd, xd size" << std::endl;
    xd[x_vt] = (m_fdm.GetPropagate()->GetUVW(1)*
			m_fdm.GetPropagate()->GetUVWdot(1) +
		m_fdm.GetPropagate()->GetUVW(2)*
			m_fdm.GetPropagate()->GetUVWdot(2) + 
		m_fdm.GetPropagate()->GetUVW(3)*
			m_fdm.GetPropagate()->GetUVWdot(3))/ 
		m_fdm.GetAuxiliary()->GetVt(); // from lewis, vtrue dot
    xd[x_alpha] = m_fdm.GetAuxiliary()->Getadot();
    xd[x_theta] = m_fdm.GetAuxiliary()->GetEulerRates(2);
    xd[x_q] = m_fdm.GetPropagate()->GetPQRdot(2);
    xd[x_alt] = m_fdm.GetPropagate()->Gethdot();
    xd[x_beta] = m_fdm.GetAuxiliary()->Getbdot();
    xd[x_phi] = m_fdm.GetAuxiliary()->GetEulerRates(1);
    xd[x_p] = m_fdm.GetPropagate()->GetPQRdot(1);
    xd[x_r] = m_fdm.GetPropagate()->GetPQRdot(3);
    xd[x_psi] = m_fdm.GetAuxiliary()->GetEulerRates(3);
	// approximation of rpm derivative
	double rpm0 = m_fdm.GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();
    m_fdm.Run(); // propagate engine dynamics by deltaT
    xd[x_rpm] = (m_fdm.GetPropulsion()->GetEngine(0)->
		GetThruster()->GetRPM()-rpm0)/propulsionDeltaT;
	m_fdm.RunIC();
}

void FGStateSpace::setX(const vector<double> & x)
{
	using namespace stateSpaceEnums;
	if (x.size()!=m_xSize) std::cerr << 
		"warning: setX, x size" << std::endl;
 	m_fdm.GetIC()->SetVtrueFpsIC(x[x_vt]);
    m_fdm.GetIC()->SetAlphaRadIC(x[x_alpha]);
    m_fdm.GetIC()->SetThetaRadIC(x[x_theta]);
    m_fdm.GetIC()->SetQRadpsIC(x[x_q]);
    m_fdm.GetIC()->SetAltitudeASLFtIC(x[x_alt]);
    m_fdm.GetIC()->SetBetaRadIC(x[x_beta]);
    m_fdm.GetIC()->SetPhiRadIC(x[x_phi]);
    m_fdm.GetIC()->SetPRadpsIC(x[x_p]);
    m_fdm.GetIC()->SetRRadpsIC(x[x_r]);
    m_fdm.GetIC()->SetPsiRadIC(x[x_psi]);
	for (int i=0;i<m_fdm.GetPropulsion()->GetNumEngines();i++)
    {
        m_fdm.GetPropulsion()->GetEngine(i)->GetThruster()->SetRPM(x[10]);
    }
	m_fdm.RunIC();
}

void FGStateSpace::getU(vector<double> & u)
{
	using namespace stateSpaceEnums;

	if (u.size()!=m_uSize) std::cerr << 
		"warning: getU, u size" << std::endl;
}

void FGStateSpace::setU(const vector<double> & u)
{
	using namespace stateSpaceEnums;
	if (u.size()!=m_uSize) std::cerr << 
		"warning: setU, u size" << std::endl;
 	m_fdm.GetFCS()->SetDaCmd(u[u_aileron]); //aileron
    m_fdm.GetFCS()->SetDeCmd(u[u_elevator]); //elevator
    m_fdm.GetFCS()->SetDrCmd(u[u_rudder]); //rudder
    for (int i=0;i<m_fdm.GetPropulsion()->GetNumEngines();i++)
    {
        FGEngine * engine = m_fdm.GetPropulsion()->GetEngine(i);
        FGThruster * thruster = engine->GetThruster();
        m_fdm.GetFCS()->SetThrottleCmd(i,u[u_throttle]);
        thruster->SetdeltaT(propulsionDeltaT);
    }
	m_fdm.RunIC();
}

void FGStateSpace::printX(const vector<double> & x)
{
	using namespace stateSpaceEnums;
	std::cout << "\nstate " << std::endl;
	std::cout << "\tvelocity\t: " <<  x[x_vt] << std::endl;
	std::cout << "\talpha\t\t: " <<  x[x_alpha] << std::endl;
	std::cout << "\ttheta\t\t: " <<  x[x_theta] << std::endl;
	std::cout << "\tq\t\t: " <<  x[x_q] << std::endl;
	std::cout << "\taltitude\t: " <<  x[x_alt] << std::endl;
	std::cout << "\tbeta\t\t: " <<  x[x_beta] << std::endl;
	std::cout << "\tphi\t\t: " <<  x[x_phi] << std::endl;
	std::cout << "\tp\t\t: " <<  x[x_p] << std::endl;
	std::cout << "\tr\t\t: " <<  x[x_r] << std::endl;
	std::cout << "\tpsi\t\t: " <<  x[x_psi] << std::endl;
	std::cout << "\trpm\t\t: " <<  x[x_rpm] << std::endl;
}

void FGStateSpace::printXd(const vector<double> & x)
{
	using namespace stateSpaceEnums;
	std::cout << "\nd state " << std::endl;
	std::cout << "\td/dt velocity\t: " <<  x[x_vt] << std::endl;
	std::cout << "\td/dt alpha\t: " <<  x[x_alpha] << std::endl;
	std::cout << "\td/dt theta\t: " <<  x[x_theta] << std::endl;
	std::cout << "\td/dt q\t\t: " <<  x[x_q] << std::endl;
	std::cout << "\td/dt altitude\t: " <<  x[x_alt] << std::endl;
	std::cout << "\td/dt beta\t: " <<  x[x_beta] << std::endl;
	std::cout << "\td/dt phi\t: " <<  x[x_phi] << std::endl;
	std::cout << "\td/dt p\t\t: " <<  x[x_p] << std::endl;
	std::cout << "\td/dt r\t\t: " <<  x[x_r] << std::endl;
	std::cout << "\td/dt psi\t: " <<  x[x_psi] << std::endl;
	std::cout << "\td/dt rpm\t: " <<  x[x_rpm] << std::endl;
}

void FGStateSpace::printU(const vector<double> & u)
{
	using namespace stateSpaceEnums;
	std::cout << "\ninput " << std::endl;
	std::cout << "\tthrottle\t: " << u[u_throttle] << std::endl;
	std::cout << "\televator\t: " <<  u[u_elevator] << std::endl;
	std::cout << "\taileron\t\t: " <<  u[u_aileron] << std::endl;
	std::cout << "\trudder\t\t: " <<  u[u_rudder] << std::endl;
}

void FGStateSpace::printX()
{
	vector<double> x(m_xSize);
	getX(x);
	printX(x);
}

void FGStateSpace::printXd()
{
	vector<double> xd(m_xSize);
	getXd(xd);
	printXd(xd);
}

void FGStateSpace::printU()
{
	vector<double> u(m_uSize);
	getU(u);
	printU(u);
}

} // JSBSim

int main (int argc, char const* argv[])
{
	// load model
	JSBSim::FGFDMExec fdm;		
	fdm.LoadModel("../aircraft","../engine","../systems","f16");

	// setup constraints
	JSBSim::FGTrimmer::Constraints constraints;
	constraints.gamma = 0.0;
	constraints.velocity = 502.0;

	// initial solver state
	int n = JSBSim::FGTrimmer::getVSize();
	std::vector<double> initialGuess(n), initialStepSize(n);
	for (int i=0;i<n;i++) initialStepSize[i] = 0.01;
	for (int i=0;i<n;i++) initialGuess[i] = 0.0;

	// solve
	JSBSim::FGTrimmer trimmer(fdm, constraints);
	JSBSim::FGNelderMead solver(trimmer,initialGuess, initialStepSize);
	
	// output
	std::vector<double> x(JSBSim::FGStateSpace::getXSize());
	std::vector<double> u(JSBSim::FGStateSpace::getUSize());
	trimmer.constrain(solver.getSolution(),x,u);
	std::cout << "solution: " << std::endl;
	JSBSim::FGStateSpace::printX(x);
	JSBSim::FGStateSpace::printU(u);
}

// vim:ts=4:sw=4
