/*
 * FGStateSpace.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * FGStateSpace.h is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * FGStateSpace.h is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef JSBSim_FGStateSpace_H
#define JSBSim_FGStateSpace_H

#include "FGFDMExec.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"
#include "models/propulsion/FGThruster.h"
#include "models/propulsion/FGTurbine.h"
#include "models/propulsion/FGTurboProp.h"
#include "models/FGAuxiliary.h"
#include "models/FGFCS.h"
#include <fstream>
#include <iostream>

namespace JSBSim
{

class FGStateSpace
{
public:

    // component class
    class Component
    {
    protected:
        FGStateSpace * m_stateSpace;
        FGFDMExec * m_fdm;
        std::string m_name, m_unit;
    public:
        Component(const std::string & name, const std::string & unit) :
                m_stateSpace(), m_fdm(), m_name(name), m_unit(unit) {};
        virtual ~Component() {};
        virtual double get() const = 0;
        virtual void set(double val) = 0;
        virtual double getDeriv() const
        {
            // by default should calculate using finite difference approx
            std::vector<double> x0 = m_stateSpace->x.get();
            double f0 = get();
            double dt0 = m_fdm->GetDeltaT();
            m_fdm->Setdt(1e-5);
            m_fdm->Run();
            double f1 = get();
            m_stateSpace->x.set(x0);
            if (m_fdm->GetDebugLevel() > 0)
            {
                std::cout << std::scientific
                          << "name: " << m_name
                          << "\nf1: " << f0
                          << "\nf2: " << f1
                          << "\ndt: " << m_fdm->GetDeltaT()
                          << "\tdf/dt: " << (f1-f0)/m_fdm->GetDeltaT()
                          << std::fixed << std::endl;
            }
            m_fdm->Setdt(dt0); // restore original value
            return (f1-f0)/m_fdm->GetDeltaT();
        };
        void setStateSpace(FGStateSpace * stateSpace)
        {
            m_stateSpace = stateSpace;
        }
        void setFdm(FGFDMExec * fdm)
        {
            m_fdm = fdm;
        }
        const std::string & getName() const
        {
            return m_name;
        }
        const std::string & getUnit() const
        {
            return m_unit;
        }
    };

    // component vector class
    class ComponentVector
    {
    public:
        ComponentVector(FGFDMExec & fdm, FGStateSpace * stateSpace) :
                m_stateSpace(stateSpace), m_fdm(fdm), m_components() {}
        ComponentVector & operator=(ComponentVector & componentVector)
        {
            m_stateSpace = componentVector.m_stateSpace;
            m_fdm = componentVector.m_fdm;
            m_components = componentVector.m_components;
            return *this;
        }
        ComponentVector(const ComponentVector & componentVector) :
                m_stateSpace(componentVector.m_stateSpace),
                m_fdm(componentVector.m_fdm),
                m_components(componentVector.m_components)
        {
        }
        void add(Component * comp)
        {
            comp->setStateSpace(m_stateSpace);
            comp->setFdm(&m_fdm);
            m_components.push_back(comp);
        }
        int getSize() const
        {
            return m_components.size();
        }
        Component * getComp(int i) const
        {
            return m_components[i];
        };
        Component * getComp(int i)
        {
            return m_components[i];
        };
        double get(int i) const
        {
            return m_components[i]->get();
        };
        void set(int i, double val)
        {
            m_components[i]->set(val);
            m_fdm.RunIC();
        };
        double get(int i)
        {
            return m_components[i]->get();
        };
        std::vector<double> get() const
        {
            std::vector<double> val;
            for (int i=0;i<getSize();i++) val.push_back(m_components[i]->get());
            return val;
        }
        void get(double * array) const
        {
            for (int i=0;i<getSize();i++) array[i] = m_components[i]->get();
        }
        double getDeriv(int i)
        {
            return m_components[i]->getDeriv();
        };
        std::vector<double> getDeriv() const
        {
            std::vector<double> val;
            for (int i=0;i<getSize();i++) val.push_back(m_components[i]->getDeriv());
            return val;
        }
        void getDeriv(double * array) const
        {
            for (int i=0;i<getSize();i++) array[i] = m_components[i]->getDeriv();
        }
        void set(vector<double> vals)
        {
            for (int i=0;i<getSize();i++) m_components[i]->set(vals[i]);
            m_fdm.RunIC();
        }
        void set(double * array)
        {
            for (int i=0;i<getSize();i++) m_components[i]->set(array[i]);
            m_fdm.RunIC();
        }
        std::string getName(int i) const
        {
            return m_components[i]->getName();
        };
        std::vector<std::string> getName() const
        {
            std::vector<std::string> name;
            for (int i=0;i<getSize();i++) name.push_back(m_components[i]->getName());
            return name;
        }
        std::string getUnit(int i) const
        {
            return m_components[i]->getUnit();
        };
        std::vector<std::string> getUnit() const
        {
            std::vector<std::string> unit;
            for (int i=0;i<getSize();i++) unit.push_back(m_components[i]->getUnit());
            return unit;
        }
    private:
        FGStateSpace * m_stateSpace;
        FGFDMExec & m_fdm;
        std::vector<Component *> m_components;
    };

    // component vectors
    ComponentVector x, u, y;

    // constructor
    FGStateSpace(FGFDMExec & fdm) : x(fdm,this), u(fdm,this), y(fdm,this), m_fdm(fdm) {};

    // deconstructor
    virtual ~FGStateSpace() {};

    // linearization function
    void linearize(std::vector<double> x0, std::vector<double> u0, std::vector<double> y0,
                   std::vector< std::vector<double> > & A,
                   std::vector< std::vector<double> > & B,
                   std::vector< std::vector<double> > & C,
                   std::vector< std::vector<double> > & D);


private:

    // compute numerical jacobian of a matrix
    void numericalJacobian(std::vector< std::vector<double> > & J, ComponentVector & y,
                           ComponentVector & x, const std::vector<double> & y0,
                           const std::vector<double> & x0, double h=1e-5, bool computeYDerivative = false);

    // flight dynamcis model
    FGFDMExec & m_fdm;

public:

    // components

    class Vt : public Component
    {
    public:
        Vt() : Component("Vt","ft/s") {};
        double get() const
        {
            return m_fdm->GetAuxiliary()->GetVt();
        }
        void set(double val)
        {
            m_fdm->GetIC()->SetVtrueFpsIC(val);
        }
        double getDeriv() const
        {

            return (m_fdm->GetPropagate()->GetUVW(1)*m_fdm->GetPropagate()->GetUVWdot(1) +
                    m_fdm->GetPropagate()->GetUVW(2)*m_fdm->GetPropagate()->GetUVWdot(2) +
                    m_fdm->GetPropagate()->GetUVW(3)*m_fdm->GetPropagate()->GetUVWdot(3))/
                   m_fdm->GetAuxiliary()->GetVt(); // from lewis, vtrue dot
        }

    };

    class Alpha : public Component
    {
    public:
        Alpha() : Component("Alpha","rad") {};
        double get() const
        {
            return m_fdm->GetAuxiliary()->Getalpha();
        }
        void set(double val)
        {
			m_fdm->GetIC()->SetFlightPathAngleRadIC(m_fdm->GetIC()->GetThetaRadIC()-val);
            m_fdm->GetIC()->SetAlphaRadIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetAuxiliary()->Getadot();
        }
    };

    class Theta : public Component
    {
    public:
        Theta() : Component("Theta","rad") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetEuler(2);
        }
        void set(double val)
        {
			m_fdm->GetIC()->SetFlightPathAngleRadIC(val-m_fdm->GetIC()->GetAlphaRadIC());
            m_fdm->GetIC()->SetThetaRadIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetAuxiliary()->GetEulerRates(2);
        }
    };

    class Q : public Component
    {
    public:
        Q() : Component("Q","rad/s") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetPQR(2);
        }
        void set(double val)
        {
            m_fdm->GetIC()->SetQRadpsIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetPropagate()->GetPQRdot(2);
        }
    };

    class Alt : public Component
    {
    public:
        Alt() : Component("Alt","ft") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetAltitudeASL();
        }
        void set(double val)
        {
            m_fdm->GetIC()->SetAltitudeASLFtIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetPropagate()->Gethdot();
        }
    };

    class Beta : public Component
    {
    public:
        Beta() : Component("Beta","rad") {};
        double get() const
        {
            return m_fdm->GetAuxiliary()->Getbeta();
        }
        void set(double val)
        {
            m_fdm->GetIC()->SetBetaRadIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetAuxiliary()->Getbdot();
        }
    };

    class Phi : public Component
    {
    public:
        Phi() : Component("Phi","rad") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetEuler(1);
        }
        void set(double val)
        {
            m_fdm->GetIC()->SetPhiRadIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetAuxiliary()->GetEulerRates(1);
        }
    };

    class P : public Component
    {
    public:
        P() : Component("P","rad/s") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetPQR(1);
        }
        void set(double val)
        {
            m_fdm->GetIC()->SetPRadpsIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetPropagate()->GetPQRdot(1);
        }
    };

    class R : public Component
    {
    public:
        R() : Component("R","rad/s") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetPQR(3);
        }
        void set(double val)
        {
            m_fdm->GetIC()->SetRRadpsIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetPropagate()->GetPQRdot(3);
        }
    };

    class Psi : public Component
    {
    public:
        Psi() : Component("Psi","rad") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetEuler(3);
        }
        void set(double val)
        {
            m_fdm->GetIC()->SetPsiRadIC(val);
        }
        double getDeriv() const
        {
            return m_fdm->GetAuxiliary()->GetEulerRates(3);
        }
    };

    class ThrottleCmd : public Component
    {
    public:
        ThrottleCmd() : Component("ThtlCmd","norm") {};
        double get() const
        {
            return m_fdm->GetFCS()->GetThrottleCmd(0);
        }
        void set(double val)
        {
            for (int i=0;i<m_fdm->GetPropulsion()->GetNumEngines();i++)
                m_fdm->GetFCS()->SetThrottleCmd(i,val);
			m_fdm->GetFCS()->Run();
        }
    };

    class ThrottlePos : public Component
    {
    public:
        ThrottlePos() : Component("ThtlPos","norm") {};
        double get() const
        {
            return m_fdm->GetFCS()->GetThrottlePos(0);
        }
        void set(double val)
        {
            for (int i=0;i<m_fdm->GetPropulsion()->GetNumEngines();i++)
                m_fdm->GetFCS()->SetThrottlePos(i,val);
        }
    };

    class DaCmd : public Component
    {
    public:
        DaCmd() : Component("DaCmd","norm") {};
        double get() const
        {
            return m_fdm->GetFCS()->GetDaCmd();
        }
        void set(double val)
        {
            m_fdm->GetFCS()->SetDaCmd(val);
			m_fdm->GetFCS()->Run();
        }
    };

    class DaPos : public Component
    {
    public:
        DaPos() : Component("DaPos","norm") {};
        double get() const
        {
            return m_fdm->GetFCS()->GetDaLPos();
        }
        void set(double val)
        {
            m_fdm->GetFCS()->SetDaLPos(ofRad,val);
            m_fdm->GetFCS()->SetDaRPos(ofRad,val); // TODO: check if this is neg.
        }
    };

    class DeCmd : public Component
    {
    public:
        DeCmd() : Component("DeCmd","norm") {};
        double get() const
        {
            return m_fdm->GetFCS()->GetDeCmd();
        }
        void set(double val)
        {
            m_fdm->GetFCS()->SetDeCmd(val);
			m_fdm->GetFCS()->Run();
        }
    };

    class DePos : public Component
    {
    public:
        DePos() : Component("DePos","norm") {};
        double get() const
        {
            return m_fdm->GetFCS()->GetDePos();
        }
        void set(double val)
        {
            m_fdm->GetFCS()->SetDePos(ofRad,val);
        }
    };

    class DrCmd : public Component
    {
    public:
        DrCmd() : Component("DrCmd","norm") {};
        double get() const
        {
            return m_fdm->GetFCS()->GetDrCmd();
        }
        void set(double val)
        {
            m_fdm->GetFCS()->SetDrCmd(val);
			m_fdm->GetFCS()->Run();
        }
    };

    class DrPos : public Component
    {
    public:
        DrPos() : Component("DrPos","norm") {};
        double get() const
        {
            return m_fdm->GetFCS()->GetDrPos();
        }
        void set(double val)
        {
            m_fdm->GetFCS()->SetDrPos(ofRad,val);
        }
    };

    class Rpm : public Component
    {
    public:
        Rpm() : Component("Rpm","rev/min") {};
        double get() const
        {
            return m_fdm->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();
        }
        void set(double val)
        {
            for (int i=0;i<m_fdm->GetPropulsion()->GetNumEngines();i++)
			{
                m_fdm->GetPropulsion()->GetEngine(i)->GetThruster()->SetRPM(val);
			}
        }
    };

    class PropPitch : public Component
    {
    public:
        PropPitch() : Component("Prop Pitch","deg") {};
        double get() const
        {
            return m_fdm->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();
        }
        void set(double val)
        {
            for (int i=0;i<m_fdm->GetPropulsion()->GetNumEngines();i++)
                m_fdm->GetPropulsion()->GetEngine(i)->GetThruster()->SetPitch(val);
        }
    };

   	class Longitude : public Component
    {
    public:
        Longitude() : Component("Longitude","rad") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetLongitude();
        }
        void set(double val)
        {
			m_fdm->GetIC()->SetLongitudeRadIC(val);
        }
    };

   	class Latitude : public Component
    {
    public:
        Latitude() : Component("Latitude","rad") {};
        double get() const
        {
            return m_fdm->GetPropagate()->GetLatitude();
        }
        void set(double val)
        {
			m_fdm->GetIC()->SetLatitudeRadIC(val);
        }
    };
};

// stream output
std::ostream &operator<<(std::ostream &out, const FGStateSpace::Component &c );
std::ostream &operator<<(std::ostream &out, const FGStateSpace::ComponentVector &v );
std::ostream &operator<<(std::ostream &out, const FGStateSpace &ss );
std::ostream &operator<<( std::ostream &out, const std::vector< std::vector<double> > &vec2d );
std::ostream &operator<<( std::ostream &out, const std::vector<double> &vec );

} // JSBSim

#endif

// vim:ts=4:sw=4
