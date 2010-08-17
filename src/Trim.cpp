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
	myF(f), nDim(initialGuess.size()), nVert(nDim+1),
	iMax(1), iNextMax(1), iMin(1),
	simplex(nVert), cost(nVert), elemSum(nDim)
{
	// set precision
	std::cout.precision(10);

	// construct simplex
	for (int vertex=0;vertex<nVert;vertex++) 
	{
		simplex[vertex] = initialGuess;
	}
	for (int dim=0;dim<nDim;dim++)
	{
		int vertex = dim + 1;
		simplex[vertex][dim] += initialStepSize[dim];
	}

	// solve simplex
	for(int iter=0;;iter++)
	{
		// find vertex costs
		for (int vertex=0;vertex<nVert;vertex++)
		{
			cost[vertex] = myF.eval(simplex[vertex]);
		}

		// find max cost, next max cost, and min cost
		for (int vertex=0;vertex<nVert;vertex++)
		{
			if ( cost[vertex] > cost[iMax] )
			{
				iNextMax = iMax;
				iMax = vertex;
			}
			else if ( cost[vertex] < cost[iMin] ) iMin = vertex;
		}

		// compute relative tolerance
		double rtolI = 2*std::abs(cost[iMax] - 
				cost[iMin])/(std::abs(cost[iMax]+std::abs(cost[iMin])+
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
		for (int dim=0;dim<nDim;dim++)
		{
			elemSum[dim] = 0;
			for (int vertex=0;vertex<nVert;vertex++) 
				elemSum[dim] += simplex[vertex][dim];
		}

		// try stretch by -1, (invet max vertex about min face)
		double costTry = tryStretch(-1.0);

		// if lower cost, then try further stretch by 2
		if (costTry < cost[iMax])
		{
			costTry = tryStretch(speed);
		}
		// otherwise try a contration
		else
		{
			costTry = tryStretch(1/speed);
		}

		// output cost and simplex
		//std::cout << std::scientific << "cost: " 
			//<< cost[iMin] << std::endl;
		//std::cout << "simplex: " << std::endl;
		//for (int i=0;i<nDim;i++)
		//{
			//for (int j=0;j<nVert;j++) std::cout << 
				//std::scientific << simplex[j][i] << " ";
			//std::cout << std::endl;
		//}
	}
}

std::vector<double> FGNelderMead::getSolution()
{
	return simplex[iMin];
}

double FGNelderMead::tryStretch(double factor)
{
	// create trial vertex
	double a= (1.0-factor)/nDim;
	double b = a - factor;
	std::vector<double> tryVertex(nDim);
	for (int dim=0;dim<nDim;dim++) 
		tryVertex[dim] = elemSum[dim]*a - simplex[iMax][dim]*b;

	// find trial cost
	double costTry = myF.eval(tryVertex);

	// if trial cost lower than max
	if (costTry <= cost[iMax]) // <= allows contraction around hard constraints
	{
		// update the element sum of the simplex
		for (int dim=0;dim<nDim;dim++) elemSum[dim] += tryVertex[dim] - simplex[iMax][dim];
		// replace the max vertex with the trial vertex
		for (int dim=0;dim<nDim;dim++) simplex[iMax][dim] = tryVertex[dim];
		//std::cout << "simplex stretched by : " << factor << std::endl;
	}
	return costTry;
}

FGTrimmer::FGTrimmer(FGFDMExec & fdm) :
	myFdm(fdm)
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

using namespace JSBSim;
int main (int argc, char const* argv[])
{
	FGFDMExec fdm;		
	fdm.LoadModel("../aircraft","../engine","../systems","f16");

	// initial conditions
	int n = 6;
	std::vector<double> initialGuess(n), initialStepSize(n);
	for (int i=0;i<n;i++) initialStepSize[i] = 0.1;
	for (int i=0;i<n;i++) initialGuess[i] = 1.0;

	// solve
	FGTrimmer trimmer(fdm);
	FGNelderMead solver(trimmer,initialGuess, initialStepSize);
	std::vector<double> x(n);
	trimmer.constrain(solver.getSolution(),x);
	std::cout << "solution: " << std::endl;
	for (int i=0;i<x.size();i++)
		std::cout << x[i] << std::endl;
}

// vim:ts=4:sw=4
