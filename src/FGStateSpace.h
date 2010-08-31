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
    class Component
    {
    protected:
        FGFDMExec * m_fdm;
        std::string m_name, m_unit;
    public:
        Component(const std::string & name, const std::string & unit) :
                m_fdm(), m_name(name), m_unit(unit) {};
        virtual ~Component() {};
        virtual double get() const = 0;
        virtual void set(double val) = 0;
        virtual double getDeriv() const
        {
            return 0;
        };
        // by default should calculate using finite difference approx
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
            m_fdm->GetAuxiliary()->SetVt(val);
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
            m_fdm->GetAuxiliary()->Setalpha(val);
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
            return m_fdm->GetPropagate()->GetEuler(1);
        }
        void set(double val)
        {
            double phi = m_fdm->GetPropagate()->GetEuler(1);
            double psi = m_fdm->GetPropagate()->GetEuler(3);
            FGQuaternion qAttitudeLocal(phi,val,psi);
            m_fdm->GetPropagate()->GetVState()->qAttitudeLocal = qAttitudeLocal;
            FGMatrix33 ti2l = m_fdm->GetPropagate()->GetTi2l();
            FGQuaternion qAttitudeECI = ti2l.GetQuaternion()*qAttitudeLocal;
            qAttitudeECI.Normalize();
            m_fdm->GetPropagate()->GetVState()->qAttitudeECI =
                ti2l.GetQuaternion()*qAttitudeLocal;
        }
        double getDeriv() const
        {
            return m_fdm->GetAuxiliary()->GetEulerRates(1);
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
            m_fdm->GetPropagate()->SetPQR(2,val);
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
            m_fdm->GetPropagate()->SetAltitudeASL(val);
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
            m_fdm->GetAuxiliary()->Setbeta(val);
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
            double theta = m_fdm->GetPropagate()->GetEuler(2);
            double psi = m_fdm->GetPropagate()->GetEuler(3);
            FGQuaternion qAttitudeLocal(val,theta,psi);
            m_fdm->GetPropagate()->GetVState()->qAttitudeLocal = qAttitudeLocal;
            FGMatrix33 ti2l = m_fdm->GetPropagate()->GetTi2l();
            FGQuaternion qAttitudeECI = ti2l.GetQuaternion()*qAttitudeLocal;
            qAttitudeECI.Normalize();
            m_fdm->GetPropagate()->GetVState()->qAttitudeECI =
                ti2l.GetQuaternion()*qAttitudeLocal;
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
            m_fdm->GetPropagate()->SetPQR(1,val);
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
            m_fdm->GetPropagate()->SetPQR(3,val);
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
            double phi = m_fdm->GetPropagate()->GetEuler(1);
            double theta = m_fdm->GetPropagate()->GetEuler(2);
            FGQuaternion qAttitudeLocal(phi,theta,val);
            m_fdm->GetPropagate()->GetVState()->qAttitudeLocal = qAttitudeLocal;
            FGMatrix33 ti2l = m_fdm->GetPropagate()->GetTi2l();
            FGQuaternion qAttitudeECI = ti2l.GetQuaternion()*qAttitudeLocal;
            qAttitudeECI.Normalize();
            m_fdm->GetPropagate()->GetVState()->qAttitudeECI =
                ti2l.GetQuaternion()*qAttitudeLocal;
        }
        double getDeriv() const
        {
            return m_fdm->GetAuxiliary()->GetEulerRates(1);
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
			m_fdm->GetFCS()->Run();
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
			m_fdm->GetFCS()->Run();
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
			m_fdm->GetFCS()->Run();
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
			m_fdm->GetFCS()->Run();
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
                m_fdm->GetPropulsion()->GetEngine(i)->GetThruster()->SetRPM(val);
        }
    };
    class Pitch : public Component
    {
    public:
        Pitch() : Component("Pitch","deg") {};
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
    class N1 : public Component
    {
    public:
        N1() : Component("N1","% rpm") {};
        double get() const
        {
            FGEngine * engine = m_fdm->GetPropulsion()->GetEngine(0);
            switch (engine->GetType())
            {
            case FGEngine::etTurbine:
                return ((FGTurbine *)engine)->GetN1();
                break;
            case FGEngine::etTurboprop:
                return ((FGTurboProp *)engine)->GetN1();
                break;
            default:
                std::cerr << "N1:get not implemented for engine type" << std::endl;
            }
        }
        void set(double val)
        {
            FGEngine * engine = m_fdm->GetPropulsion()->GetEngine(0);
            switch (engine->GetType())
            {
            case FGEngine::etTurbine:
                ((FGTurbine *)engine)->SetN1(val);
                break;
            case FGEngine::etTurboprop:
                ((FGTurboProp *)engine)->SetN1(val);
                break;
            default:
                std::cerr << "N1:set not implemented for engine type" << std::endl;
            }
        }
    };
    class N2 : public Component
    {
    public:
        N2() : Component("N2","% rpm") {};
        double get() const
        {
            FGEngine * engine = m_fdm->GetPropulsion()->GetEngine(0);
            switch (engine->GetType())
            {
            case FGEngine::etTurbine:
                return ((FGTurbine *)engine)->GetN2();
                break;
            case FGEngine::etTurboprop:
                return ((FGTurboProp *)engine)->GetN2();
                break;
            default:
                std::cerr << "N2:get not implemented for engine type" << std::endl;
            }
        }
        void set(double val)
        {
            FGEngine * engine = m_fdm->GetPropulsion()->GetEngine(0);
            switch (engine->GetType())
            {
            case FGEngine::etTurbine:
                ((FGTurbine *)engine)->SetN2(val);
                break;
            case FGEngine::etTurboprop:
                ((FGTurboProp *)engine)->SetN2(val);
                break;
            default:
                std::cerr << "N2:set not implemented for engine type" << std::endl;
            }
        }
    };
    FGStateSpace(FGFDMExec & fdm) : x(fdm), u(fdm), y(fdm), m_fdm(fdm) {};
    virtual ~FGStateSpace() {};
    class ComponentVector
    {
    public:
        ComponentVector(FGFDMExec & fdm) : m_fdm(fdm), m_v() {}
        ComponentVector & operator=(ComponentVector & cv)
        {
            m_fdm = cv.m_fdm;
            m_v = cv.m_v;
            return *this;
        }
        ComponentVector(const ComponentVector & cv) : m_fdm(cv.m_fdm), m_v(cv.m_v) {}
        void add(Component * comp)
        {
            comp->setFdm(&m_fdm);
            m_v.push_back(comp);
        }
        int getSize() const
        {
            return m_v.size();
        }
        Component * getComp(int i) const
        {
            return m_v[i];
        };
        Component * getComp(int i)
        {
            return m_v[i];
        };
        double get(int i) const
        {
            return m_v[i]->get();
        };
        void set(int i, double val) const
        {
            return m_v[i]->set(val);
        };
        double get(int i)
        {
            return m_v[i]->get();
        };
        std::vector<double> get() const
        {
            std::vector<double> val;
            for (int i=0;i<getSize();i++) val.push_back(m_v[i]->get());
            return val;
        }
        double getDeriv(int i)
        {
            return m_v[i]->getDeriv();
        };
        std::vector<double> getDeriv() const
        {
            std::vector<double> val;
            for (int i=0;i<getSize();i++) val.push_back(m_v[i]->getDeriv());
            return val;
        }
        void set(vector<double> vals)
        {
            for (int i=0;i<getSize();i++) m_v[i]->set(vals[i]);
        }
        std::string getName(int i) const
        {
            return m_v[i]->getName();
        };
        std::vector<std::string> getName() const
        {
            std::vector<std::string> name;
            for (int i=0;i<getSize();i++) name.push_back(m_v[i]->getName());
            return name;
        }
        std::string getUnit(int i) const
        {
            return m_v[i]->getUnit();
        };
        std::vector<std::string> getUnit() const
        {
            std::vector<std::string> unit;
            for (int i=0;i<getSize();i++) unit.push_back(m_v[i]->getUnit());
            return unit;
        }
    private:
        FGFDMExec & m_fdm;
        std::vector<Component *> m_v;
    };
    void linearize(std::vector<double> x0, std::vector<double> u0, std::vector<double> y0,
                   std::vector< std::vector<double> > & A,
                   std::vector< std::vector<double> > & B,
                   std::vector< std::vector<double> > & C,
                   std::vector< std::vector<double> > & D);
    void numericalJacobian(std::vector< std::vector<double> > & J, ComponentVector & y,
    	ComponentVector & x, const std::vector<double> & y0,
    	const std::vector<double> & x0, double h=1e-5, bool computeYDerivative = false);
	double diffStep(
		ComponentVector & y,
		ComponentVector & x,
		std::vector<double> y0,
		std::vector<double> x0,
		double h, int yI, int xI,
		bool computeYDerivative);
    ComponentVector x, u, y;
private:
    FGFDMExec & m_fdm;
};

std::ostream &operator<<(std::ostream &out, const FGStateSpace::Component &c );
std::ostream &operator<<(std::ostream &out, const FGStateSpace::ComponentVector &v );
std::ostream &operator<<(std::ostream &out, const FGStateSpace &ss );
std::ostream &operator<<( std::ostream &out, const std::vector< std::vector<double> > &vec2d );

} // JSBSim

#endif

// vim:ts=4:sw=4
