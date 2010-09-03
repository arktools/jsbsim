/*
 * FGNelderMead.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * FGNelderMead.h is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * FGNelderMead.h is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef JSBSim_FGNelderMead_H
#define JSBSim_FGNelderMead_H

#include <vector>
#include <limits>

namespace JSBSim
{

class FGNelderMead
{
public:
    class Function
    {
    public:
        virtual double eval(const std::vector<double> & v)  = 0;
        virtual ~Function() {};
    };
    FGNelderMead(Function & f, const std::vector<double> & initialGuess,
                 const std::vector<double> & lowerBound,
                 const std::vector<double> & upperBound,
                 const std::vector<double> initialStepSize, int iterMax=2000,
                 double rtol=std::numeric_limits<float>::epsilon(),
                 double abstol=std::numeric_limits<float>::epsilon(),
                 double speed = 2.0,
                 bool showConvergeStatus=true,bool showSimplex=false,
                 bool pause=false);
    std::vector<double> getSolution();
private:
    // attributes
    Function & m_f;
    const std::vector<double> & m_lowerBound;
    const std::vector<double> & m_upperBound;
    int m_nDim, m_nVert, m_iMax, m_iNextMax, m_iMin;
    std::vector< std::vector<double> > m_simplex;
    std::vector<double> m_cost;
    std::vector<double> m_elemSum;
    bool m_showSimplex;

    // methods
    double tryStretch(double factor);
    void contract();
    void constructSimplex(const std::vector<double> & guess, const std::vector<double> & stepSize);
    void boundVertex(std::vector<double> & vertex,
                     const std::vector<double> & upperBound,
                     const std::vector<double> & lowerBound);
};

} // JSBSim

#endif

// vim:ts=4:sw=4
