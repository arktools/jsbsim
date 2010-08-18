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
#include <limits>
#include <cmath>

namespace JSBSim
{

FGNelderMead::FGNelderMead(const Function & f, const std::vector<double> & initialGuess, 
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
		//std::cout << std::scientific << "cost: " 
			//<< m_cost[m_iMin] << std::endl;
		//std::cout << "simplex: " << std::endl;
		//for (int i=0;i<m_nDim;i++)
		//{
			//for (int j=0;j<m_nVert;j++) std::cout << 
				//std::scientific << m_simplex[j][i] << " ";
			//std::cout << std::endl;
		//}
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
	if (costTry <= m_cost[m_iMax]) // <= allows contraction around hard constraints
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
	m_fdm(fdm), m_constraints(constraints)
{
}

void FGTrimmer::constrain(const std::vector<double> & v, std::vector<double> & x) const
{
	for (int i=0;i<v.size();i++)
		if (v[i] < .124) x[i] = .124;
}

double FGTrimmer::eval(const std::vector<double> & v) const
{
	std::vector<double> x = v;
	constrain(v,x);
	double cost = 0;
	for (int i=0;i<v.size();i++)
	{
		cost += x[i]*x[i];
	}
	return cost;
}

} // JSBSim

int main (int argc, char const* argv[])
{
	// load model
	JSBSim::FGFDMExec fdm;		
	fdm.LoadModel("../aircraft","../engine","../systems","f16");

	// setup constraints
	JSBSim::FGTrimmer::Constraints constraints;
	constraints.gam = 1.0;

	// initial solver state
	int n = 6;
	std::vector<double> initialGuess(n), initialStepSize(n);
	for (int i=0;i<n;i++) initialStepSize[i] = 0.1;
	for (int i=0;i<n;i++) initialGuess[i] = 1.0;

	// solve
	JSBSim::FGTrimmer trimmer(fdm, constraints);
	JSBSim::FGNelderMead solver(trimmer,initialGuess, initialStepSize);
	
	// output
	std::vector<double> x(n);
	trimmer.constrain(solver.getSolution(),x);
	std::cout << "solution: " << std::endl;
	for (int i=0;i<x.size();i++) std::cout << x[i] << std::endl;
}

// vim:ts=4:sw=4
