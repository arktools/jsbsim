/*
 * FGStateSpace.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * FGStateSpace.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * FGStateSpace.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FGStateSpace.h"

namespace JSBSim
{

void FGStateSpace::linearize(std::vector<double> X0, std::vector<double> U0,
		std::vector< std::vector<double> > & A,
		std::vector< std::vector<double> > & B)
{
	int n = x.getSize();
	int p = u.getSize();
	double h = 1e-5;

	x.set(X0);
	u.set(U0);

	// A, f(x,u)/dx
	A.resize(n);
	for (int i=0;i<n;i++)
	{
		A[i].resize(n);
		for (int j=0;j<n;j++)
		{
			double f1 = x.get(i);
			x.set(i,x.get(i)+h);
			m_fdm.Run();
			double f2 = x.get(i);
			A[i][j] = (f2-f1)/h;
			x.set(X0);
			u.set(U0);
		}
	}

	// B, f(x,u)/du
	B.resize(n);
	for (int i=0;i<n;i++)
	{
		B[i].resize(p);
		for (int j=0;j<p;j++)
		{
			double f1 = x.get(i);
			u.set(i,u.get(i)+h);
			m_fdm.Run();
			double f2 = x.get(i);
			A[i][j] = (f2-f1)/h;
			x.set(X0);
			u.set(U0);
		}
	}
}

ostream &operator<<( ostream &out, const FGStateSpace::Component &c )
{
	out << "\t" << c.getName()
		<< "\t" << c.getUnit()
		<< "\t:\t" << c.get() << std::endl;
}
ostream &operator<<( ostream &out, const FGStateSpace::ComponentVector &v )
{
	for (int i=0; i< v.getSize(); i++)
	{
		out << *(v.getComp(i));
	}
	out << std::endl;
}
ostream &operator<<( ostream &out, const FGStateSpace &ss )
{
	out << "\nX:\n" << ss.x << "\nU:\n" << ss.u << std::endl;
}

} // JSBSim


// vim:ts=4:sw=4
