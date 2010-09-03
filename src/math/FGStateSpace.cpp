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
#include <limits>

namespace JSBSim
{

void FGStateSpace::linearize(
    std::vector<double> x0,
    std::vector<double> u0,
    std::vector<double> y0,
    std::vector< std::vector<double> > & A,
    std::vector< std::vector<double> > & B,
    std::vector< std::vector<double> > & C,
    std::vector< std::vector<double> > & D)
{
    double h = 1e-3;

    // A, d(x)/dx
    numericalJacobian(A,x,x,x0,x0,h,true);
    // B, d(x)/du
    numericalJacobian(B,x,u,x0,u0,h,true);
    // C, d(y)/dx
    numericalJacobian(C,y,x,y0,x0,h);
    // D, d(y)/du
    numericalJacobian(D,y,u,y0,u0,h);

}

void FGStateSpace::numericalJacobian(std::vector< std::vector<double> >  & J, ComponentVector & y,
                                     ComponentVector & x, const std::vector<double> & y0, const std::vector<double> & x0, double h, bool computeYDerivative)
{
    int nX = x.getSize();
    int nY = y.getSize();
    double f1 = 0, f2 = 0, fn1 = 0, fn2 = 0;
    J.resize(nY);
    for (int iY=0;iY<nY;iY++)
    {
        J[iY].resize(nX);
        for (int iX=0;iX<nX;iX++)
        {
            x.set(x0);
            x.set(iX,x.get(iX)+h);
            if (computeYDerivative) f1 = y.getDeriv(iY);
            else f1 = y.get(iY);

            x.set(x0);
            x.set(iX,x.get(iX)+2*h);
            if (computeYDerivative) f2 = y.getDeriv(iY);
            else f2 = y.get(iY);

            x.set(x0);
            x.set(iX,x.get(iX)-h);
            if (computeYDerivative) fn1 = y.getDeriv(iY);
            else fn1 = y.get(iY);

            x.set(x0);
            x.set(iX,x.get(iX)-2*h);
            if (computeYDerivative) fn2 = y.getDeriv(iY);
            else fn2 = y.get(iY);

            J[iY][iX] = (8*(f1-fn1)-(f2-fn2))/(12*h); // 3rd order taylor approx from lewis, pg 203
            x.set(x0);

            if (m_fdm.GetDebugLevel() > 0)
            {
                std::cout << std::scientific << "\ty:\t" << y.getName(iY) << "\tx:\t"
                          << x.getName(iX)
                          << "\tfn2:\t" << fn2 << "\tfn1:\t" << fn1
                          << "\tf1:\t" << f1 << "\tf2:\t" << f2
                          << "\tf1-fn1:\t" << f1-fn1
                          << "\tf2-fn2:\t" << f2-fn2
                          << "\tdf/dx:\t" << J[iY][iX]
                          << std::fixed << std::endl;
            }
        }
    }
}

std::ostream &operator<<( std::ostream &out, const FGStateSpace::Component &c )
{
    out << "\t" << c.getName()
    << "\t" << c.getUnit()
    << "\t:\t" << c.get() << std::ends;
}

std::ostream &operator<<( std::ostream &out, const FGStateSpace::ComponentVector &v )
{
    for (int i=0; i< v.getSize(); i++)
    {
        out << *(v.getComp(i)) << std::endl;
    }
    out << std::ends;
}

std::ostream &operator<<( std::ostream &out, const FGStateSpace &ss )
{
    out << "\nX:\n" << ss.x
    << "\nU:\n" << ss.u
    << "\nY:\n" << ss.y
    << std::ends;
}

std::ostream &operator<<( std::ostream &out, const std::vector< std::vector<double> > &vec2d )
{
    for (int i=0;i<vec2d.size();i++)
    {
        for (int j=0;j<vec2d[0].size();j++)
        {
            out << "\t" << vec2d[i][j];
            if (j==vec2d[0].size()-1)  out << ";";
            else out << ",";
        }
        out << std::endl;
    }
    out << std::ends;
}


} // JSBSim


// vim:ts=4:sw=4
