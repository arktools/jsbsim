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
    double h = 1e-8;
    double dt = 1e-2;

    m_fdm.Setdt(dt);

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
    int n = x.getSize();
    int m = y.getSize();
    double f1 = 0, f2 = 0, fn1 = 0, fn2 = 0;
    J.resize(m);
    for (int i=0;i<m;i++)
    {
        J[i].resize(n);
        for (int j=0;j<n;j++)
        {
            x.set(x0);
            y.set(y0);
            x.set(j,x.get(j)+h);
            if (computeYDerivative)
            {
                m_fdm.SuspendIntegration();
                m_fdm.Run();
                m_fdm.ResumeIntegration();
                f1 = y.getDeriv(i);
            }
            else f1 = y.get(i);

            x.set(x0);
            y.set(y0);
            x.set(j,x.get(j)+2*h);
            if (computeYDerivative)
            {
                m_fdm.SuspendIntegration();
                m_fdm.Run();
                m_fdm.ResumeIntegration();
                f2 = y.getDeriv(i);
            }
            else f2 = y.get(i);

            x.set(x0);
            y.set(y0);
            x.set(j,x.get(j)-h);
            if (computeYDerivative)
            {
                m_fdm.SuspendIntegration();
                m_fdm.Run();
                m_fdm.ResumeIntegration();
                fn1 = y.getDeriv(i);
            }
            else fn1 = y.get(i);

            x.set(x0);
            y.set(y0);
            x.set(j,x.get(j)-2*h);
            if (computeYDerivative)
            {
                m_fdm.SuspendIntegration();
                m_fdm.Run();
                m_fdm.ResumeIntegration();
                fn2 = y.getDeriv(i);
            }
            else fn2 = y.get(i);

            J[i][j] = (8*(f1-fn1)-(f2-fn2))/(12*h); // 3rd order taylor approx from lewis, pg 203
            x.set(x0);
            y.set(y0);

            if (m_fdm.GetDebugLevel() > 0)
            {
                std::cout << std::scientific << "\ti:\t" << y.getName(i) << "\tj:\t"
                          << x.getName(j)
                          << "\tfn2:\t" << fn2 << "\tfn1:\t" << fn1
                          << "\tf1:\t" << f1 << "\tf2:\t" << f2
                          << "\tf1-fn1:\t" << f1-fn1
                          << "\tf2-fn2:\t" << f2-fn2
                          << "\tdf/dx:\t" << J[i][j]
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
        }
        out << std::endl;
    }
    out << std::ends;
}


} // JSBSim


// vim:ts=4:sw=4
