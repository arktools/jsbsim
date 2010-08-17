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

namespace JSBSim
{

using namespace std;

class ConstrainedFunction
{
public:
	virtual void constrain(vector<double> & v) = 0;
	virtual double eval(const vector<double> & v) = 0;
};

class FGNelderMead
{
public:
	FGNelderMead(ConstrainedFunction * f, const vector<double> & initialGuess, 
			const vector<double> initialStepSize) :
		f(), simplex()
	{
		nDims = initialGuess.size();
		nVerts = nDims + 1;
		// construct simplex
		for (int vertex=0;vertex<nVerts;vertex++) simplex[vertex] = initialGuess;
		for (int dim=0;dim<nDims;dim++)
		{
			int vertex = dim + 1;
			simplex[vertex][dim] += initialStepSize[dim];
		}
		// find vertex costs
		for (int vertex=0;vertex<nVerts;vertex++)
		{
			f->constrain(simplex[vertex]);
			cost[vertex] = f->eval(simplex[vertex]);
		}
		// sort by cost
		// solve
	}
private:
	int nVerts, nDims;
	ConstrainedFunction * f;
	vector< vector<double> > simplex;
	vector<double> cost;
};

class FGTrimmer : public ConstrainedFunction
{
public:
	FGTrimmer(FGFDMExec * fdm) :
		myFdm(fdm)
	{
	}
private:
	FGFDMExec * myFdm;
	void constrain(vector<double> & v)	
	{
		v[0] = 1;
	}
	double eval(const vector<double> & v)
	{
		return 0;
	}
};

} // JSBSim

using namespace JSBSim;
int main (int argc, char const* argv[])
{
	// load an aircraft
	FGFDMExec * fdm = new FGFDMExec;		
	fdm->LoadModel("../aircraft","../engine","../systems","f16");
	// solve for trim condition
	FGTrimmer * trimmer = new FGTrimmer(fdm);
	int n = 6;
	vector<double> initialGuess(n), initialStepSize(n);
	for (int i=0;i<n;i++) initialStepSize[i] = 0.1;
	for (int i=0;i<n;i++) initialGuess[i] = 0.0;
	FGNelderMead * solver = new FGNelderMead(trimmer,initialGuess, initialStepSize);

	// cleanup
	delete fdm;
	delete trimmer;
	delete solver;
	return 0;
}

// vim:ts=4:sw=4
