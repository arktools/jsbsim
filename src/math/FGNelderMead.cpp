/*
 * FGNelderMead.cpp
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * FGNelderMead.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * FGNelderMead.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FGNelderMead.h"
#include <limits>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>

namespace JSBSim
{

FGNelderMead::FGNelderMead(Function & f, const std::vector<double> & initialGuess,
                           const std::vector<double> & lowerBound,
                           const std::vector<double> & upperBound,
                           const std::vector<double> initialStepSize, int iterMax,
                           double rtol, double abstol, double speed, bool showConvergeStatus,
                           bool showSimplex, bool pause, Callback * callback) :
        m_f(f), m_lowerBound(lowerBound), m_upperBound(upperBound),
        m_nDim(initialGuess.size()), m_nVert(m_nDim+1),
        m_iMax(1), m_iNextMax(1), m_iMin(1),
        m_simplex(m_nVert), m_cost(m_nVert), m_elemSum(m_nDim),
        m_showSimplex(showSimplex), m_callback(callback)
{
    // setup
    std::cout.precision(3);
    double rtolI = 0;
    double minCostPrevResize = 0, minCost = 0;
    double minCostPrev = 0, maxCost = 0, nextMaxCost = 0;
    int iter = 0;

    // solve simplex
    while (1)
    {
        // reinitialize simplex whenever rtol condition is met
        if ( rtolI < rtol || iter == 0)
        {
            std::vector<double> guess(m_nDim);
            if (iter == 0)
            {
                //std::cout << "constructing simplex" << std::endl;
                guess = initialGuess;
            }
            else
            {
                if (std::abs(minCost-minCostPrevResize) < 1e-20)
                {
                    std::cout << "\nunable to escape local minimum" << std::endl;
                    break;
                }
                //std::cout << "reinitializing step size" << std::endl;
                guess = m_simplex[m_iMin];
                minCostPrevResize = minCost;
            }
            constructSimplex(guess,initialStepSize);
        }

        // find vertex costs
        for (int vertex=0;vertex<m_nVert;vertex++)
        {
            m_cost[vertex] = m_f.eval(m_simplex[vertex]);
			if (m_callback) m_callback->eval(m_simplex[vertex]);
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
        rtolI = 2*std::abs(m_cost[m_iMax] -
                           m_cost[m_iMin])/(std::abs(m_cost[m_iMax]+std::abs(m_cost[m_iMin])+
                                                     std::numeric_limits<double>::epsilon()));

        // check for max iteratin break condition
        if (iter > iterMax)
        {
            std::cout << "\nmax iterations exceeded" << std::endl;
            break;

        }
        // check for convergence break condition
        else if ( m_cost[m_iMin] < abstol )
        {
            std::cout << "\nsimplex converged" << std::endl;
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
        minCostPrev = minCost;
        minCost = m_cost[m_iMin];
        maxCost = m_cost[m_iMax];
        nextMaxCost = m_cost[m_iNextMax];

        // output cost and simplex
        if (showConvergeStatus)
        {
            if ( (minCostPrev + std::numeric_limits<float>::epsilon() )
                    < minCost && minCostPrev != 0)
            {
                std::cout << "\twarning: simplex cost increased"
                          << std::scientific
                          << "\n\tcost: " << minCost
                          << "\n\tcost previous: " << minCostPrev
                          << std::fixed << std::endl;
            }

            std::cout << "\ti: " << iter
                      << std::scientific
                      << "\tcost: " << m_cost[m_iMin]
                      << "\trtol: " << rtolI
                      << std::fixed
                      << "\talpha: " << m_simplex[m_iMin][2]*180/M_PI
                      << "\tbeta: " << m_simplex[m_iMin][5]*180/M_PI
                      << "\tthrottle: " << m_simplex[m_iMin][0]
                      << "\televator: " << m_simplex[m_iMin][1]
                      << "\taileron: " << m_simplex[m_iMin][3]
                      << "\trudder: " << m_simplex[m_iMin][4]
                      << std::endl;
        }
        if (showSimplex)
        {
            std::cout << "simplex: " << std::endl;;
            for (int j=0;j<m_nVert;j++)
                std::cout << "\t" << std::scientific
                          << std::setw(10) << m_cost[j];
            std::cout << std::endl;
            for (int j=0;j<m_nVert;j++) std::cout << "\t\t" << j;
            std::cout << std::endl;
            for (int i=0;i<m_nDim;i++)
            {
                for (int j=0;j<m_nVert;j++)
                    std::cout << "\t" << std::setw(10) << m_simplex[j][i];
                std::cout << std::endl;
            }
            std::cout << std::fixed
                      << "\n\tiMax: " <<  m_iMax
                      << "\t\tiNextMax: " <<  m_iNextMax
                      << "\t\tiMin: " <<  m_iMin << std::endl;
        }

        if (pause)
        {
            std::cout << "paused, press any key to continue" << std::endl;
            std::cin.get();
        }


        // costs

        // try inversion
        double costTry = tryStretch(-1.0);

        // if lower cost than best, then try further stretch by speed factor
        if (costTry < minCost)
        {
            costTry = tryStretch(speed);
        }
        // otherwise try a contraction
        else if (costTry > nextMaxCost)
        {
            // 1d contraction
            costTry = tryStretch(1./speed);

            // if greater than max cost, contract about min
            if (costTry > maxCost)
            {
                if (showSimplex)
                    std::cout << "multiD contraction about: " << m_iMin << std::endl;
                contract();
            }
        }

        // iteration
        iter++;
    }
    std::cout << "\ti\t: " << iter << std::endl;
    std::cout << std::scientific;
    std::cout << "\trtol\t: " << rtolI << std::endl;
    std::cout << "\tcost\t: " << m_cost[m_iMin] << std::endl;
    std::cout << std::fixed;
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
    {
        tryVertex[dim] = m_elemSum[dim]*a - m_simplex[m_iMax][dim]*b;
        boundVertex(tryVertex,m_lowerBound,m_upperBound);
    }

    // find trial cost
    double costTry0 = m_f.eval(tryVertex);
    double costTry = m_f.eval(tryVertex);

    if (std::abs(costTry0-costTry) > std::numeric_limits<float>::epsilon())
    {
        std::cout << "\twarning: dynamics not stable!" << std::endl;
        return 2*m_cost[m_iMax];
    }

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
        if (m_showSimplex) std::cout << "stretched\t" << m_iMax << "\tby : " << factor << std::endl;
    }
    return costTry;
}

void FGNelderMead::contract()
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

void FGNelderMead::constructSimplex(const std::vector<double> & guess,
                                    const std::vector<double> & stepSize)
{
    for (int vertex=0;vertex<m_nVert;vertex++)
    {
        m_simplex[vertex] = guess;
        std::vector<double> upperBound(guess.size());
        for (int dim=0;dim<m_nDim;dim++) upperBound[dim] = m_upperBound[dim]-stepSize[dim];
        boundVertex(m_simplex[vertex],m_lowerBound,upperBound);
    }
    for (int dim=0;dim<m_nDim;dim++)
    {
        int vertex = dim + 1;
        m_simplex[vertex][dim] += stepSize[dim];
    }
    if (m_showSimplex)
    {
        std::cout << "simplex: " << std::endl;;
        for (int j=0;j<m_nVert;j++) std::cout << "\t\t" << j;
        std::cout << std::endl;
        for (int i=0;i<m_nDim;i++)
        {
            for (int j=0;j<m_nVert;j++)
                std::cout << "\t" << std::setw(10) << m_simplex[j][i];
            std::cout << std::endl;
        }
    }
}

void FGNelderMead::boundVertex(std::vector<double> & vertex,
                               const std::vector<double> & lowerBound,
                               const std::vector<double> & upperBound)
{
    for (int dim=0;dim<m_nDim;dim++)
    {
        if (vertex[dim] > upperBound[dim]) vertex[dim] = upperBound[dim];
        else if (vertex[dim] < lowerBound[dim]) vertex[dim] = lowerBound[dim];
    }
}

} // JSBSim


// vim:ts=4:sw=4
