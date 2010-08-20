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
#include <limits>
#include <cmath>
#include <cstdlib>

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

	// solve simplex
	for(int iter=0;;iter++)
	{
		// reinitialize simplex every 500 cycles
		if ( (iter % 1000) == 0)
		{
			vector<double> guess(m_nDim);
			if (iter == 0)
			{
				//std::cout << "constructing simplex" << std::endl;
				guess = initialGuess;
			}
			else
			{
				//std::cout << "reinitializing step size" << std::endl;
 				guess = m_simplex[m_iMin];
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
			std::cout << "i\t: " << iter << std::endl;
			std::cout << std::scientific;
			std::cout << "rtol\t: " << rtolI << std::endl;
			std::cout << "cost\t: " << m_cost[m_iMin] << std::endl;
			std::cout << std::fixed;
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
		double minCost = m_cost[m_iMin];
		double maxCost = m_cost[m_iMax];
		double nextMaxCost = m_cost[m_iNextMax];

		// output cost and simplex
		//std::cout << "iMax: " <<  m_iMax;
		//std::cout << "\t\tiNextMax: " <<  m_iNextMax;
		//std::cout << "\t\tiMin: " <<  m_iMin;
		//std::cout << "\t\tcost: " << m_cost[m_iMin] << std::endl;
		//std::cout << "simplex: " << std::endl;
		//for (int j=0;j<m_nVert;j++) std::cout << "\t" << j << "\t\t";
		//std::cout << std::endl;
		//for (int j=0;j<m_nVert;j++) std::cout << m_cost[j] << "\t\t";
		//std::cout << std::endl;
		//for (int i=0;i<m_nDim;i++)
		//{
			//for (int j=0;j<m_nVert;j++) std::cout << 
				//m_simplex[j][i] << "\t\t";
			//std::cout << std::endl;
		//}
		//std::cin.get();

		// try inversion
		double costTry = tryStretch(-1.0);

		// if lower cost than best, then try further stretch by 2
		if (costTry <= minCost)
		{
			costTry = tryStretch(speed);
		}
		// otherwise try a contraction
		else if (costTry >= nextMaxCost)
		{
			// 1d contraction
			costTry = tryStretch(1./speed);
		
			// if greater than max cost, contract about min
			if (costTry >= maxCost)
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
		// update the cost
		m_cost[m_iMax] = costTry;
		//std::cout << "stretched\t" << m_iMax << "\tby : " << factor << std::endl;
	}
	return costTry;
}

void FGNelderMead::constructSimplex(const vector<double> & guess, 
	const vector<double> & stepSize)
{
	for (int vertex=0;vertex<m_nVert;vertex++) 
	{
		m_simplex[vertex] = guess;
	}
	for (int dim=0;dim<m_nDim;dim++)
	{
		int vertex = dim + 1;
		m_simplex[vertex][dim] += stepSize[dim];
	}
}

FGTrimmer::FGTrimmer(FGFDMExec & fdm, const Constraints & constraints) :
	m_fdm(fdm), m_constraints(constraints)
{
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

	// apply constraints
	double velocity = m_constraints.velocity; 
	double altitude = 1000.0;
	double phi = 0.0;
	double theta = alpha;
	double psi = 0.0;
	double p = 0.0;
	double q = 0.0;
	double r= 0.0;

	// set state
	fgic()->SetVtrueFpsIC(velocity);
	fgic()->SetAltitudeASLFtIC(altitude);
	fgic()->SetPhiRadIC(phi);
	fgic()->SetThetaRadIC(theta);
	fgic()->SetPsiRadIC(psi);
	fgic()->SetPRadpsIC(p);
	fgic()->SetQRadpsIC(q);
	fgic()->SetRRadpsIC(r);
	fgic()->SetAlphaRadIC(alpha);
	fgic()->SetBetaRadIC(beta);

	
	double udot0 = -1;
	while(1)
	{
		m_fdm.RunIC();

		// controls 
		fcs()->SetDeCmd(elevator);
		fcs()->SetDaCmd(aileron);
		fcs()->SetDrCmd(rudder);
		for (int i=0; i<propulsion()->GetNumEngines(); i++) 
		{
			propulsion()->GetEngine(i)->InitRunning();
			propulsion()->GetEngine(i)->SetTrimMode(true);
			fcs()->SetThrottleCmd(i,throttle);
		}
		
		// steady propulsion system
		propulsion()->GetSteadyState();
		//std::cout << "\tthrust: " << propulsion()->GetEngine(0)->GetThruster()->GetThrust();
		//std::cout << "\t\trpm: " << propulsion()->GetEngine(0)->GetThruster()->GetRPM() << std::endl;
		double udot = propagate()->GetUVWdot(1);
		if (std::abs(udot0 - udot) < 1e-15) break;
		else
		{
			//std::cout << "delta udot: " << std::scientific << 
				//std::abs(udot-udot0) << std::fixed << std::endl;
			udot0 = udot;
		}
	}
}

double FGTrimmer::eval(const std::vector<double> & v)
{
	constrain(v);
	double dvt = (propagate()->GetUVW(1)*propagate()->GetUVWdot(1) +
		propagate()->GetUVW(2)*propagate()->GetUVWdot(2) + 
		propagate()->GetUVW(3)*propagate()->GetUVWdot(3))/ 
		aux()->GetVt(); // from lewis, vtrue dot
	double dalpha = aux()->Getadot();
	double dbeta = aux()->Getbdot();
	double dp = propagate()->GetPQRdot(1);
	double dq = propagate()->GetPQRdot(2);
	double dr = propagate()->GetPQRdot(3);
	double cost = dvt*dvt + 
		100.0*(dalpha*dalpha + dbeta*dbeta) + 
		10.0*(dp*dp + dq*dq + dr*dr);
	//std::cout << "\tdvt\t: " << dvt;
	//std::cout << "\t\tdalpha\t: " << dalpha;
	//std::cout << "\t\tdbeta\t: " << dbeta;
	//std::cout << "\t\tdp\t: " << dp;
	//std::cout << "\t\tdq\t: " << dq;
	//std::cout << "\t\tdr\t: " << dr << std::endl;
	return cost;
}

} // JSBSim

int main (int argc, char const* argv[])
{
	// load model
	JSBSim::FGFDMExec fdm;		
	fdm.LoadModel("../aircraft","../engine","../systems",argv[1]);

	// setup constraints
	JSBSim::FGTrimmer::Constraints constraints;
	constraints.gamma = 0.0;
	constraints.velocity = atof(argv[2]);

	// initial solver state
	int n = 6;
	std::vector<double> initialGuess(n), initialStepSize(n);
	for (int i=0;i<n;i++) initialStepSize[i] = 0.2;
	for (int i=0;i<n;i++) initialGuess[i] = 0.0;
	initialGuess[0] = 0.5;

	// solve
	JSBSim::FGTrimmer trimmer(fdm, constraints);
	JSBSim::FGNelderMead solver(trimmer,initialGuess, initialStepSize);
	
	// output
	std::vector<double> v = solver.getSolution();
	double throttle = v[0];
	double elevator = v[1];
	double alpha = v[2];
	double aileron = v[3];
	double rudder = v[4];
	double beta = v[5];
	std::cout << "design vector" << std::endl;
	std::cout << "\tthrottle\t%\t: " << throttle*100 << std::endl;
	std::cout << "\televator\t%\t: " << elevator*100 << std::endl;
	std::cout << "\taileron\t\t%\t: " << aileron*100 << std::endl;
	std::cout << "\trudder\t\t%\t: " << rudder*100 << std::endl;
	std::cout << "\talpha\t\tdeg\t: " << alpha*180/M_PI << std::endl;
	std::cout << "\tbeta\t\tdeg\t: " << beta*180/M_PI << std::endl;
}

// vim:ts=4:sw=4
