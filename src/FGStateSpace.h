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
		virtual ~Component(){};
		virtual double get() = 0;
		virtual void set(double val) = 0;
		virtual double getIC() = 0;
		virtual void setIC(double val) = 0;
		virtual double getDeriv() { return 0; }; // by default should calculate using finite difference approx
		void setFdm(FGFDMExec * fdm) { m_fdm = fdm; }
		const std::string & getName() {return m_name; }
		const std::string & getUnit() {return m_unit; }
	};
	class Vt : public Component
	{
	public:
		Vt() : Component("Vt","ft/s") {};
		double get() { return m_fdm->GetAuxiliary()->GetVt(); }
		void set(double val) { m_fdm->GetAuxiliary()->SetVt(val); }
		double getIC() { return m_fdm->GetIC()->GetVtrueFpsIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetVtrueFpsIC(val); }
	};
	class Alpha : public Component
	{
	public:
		Alpha() : Component("Alpha","rad") {};
		double get() { return m_fdm->GetAuxiliary()->Getalpha(); }
		void set(double val) { m_fdm->GetAuxiliary()->Setalpha(val); }
		double getIC() { return m_fdm->GetIC()->GetAlphaRadIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetAlphaRadIC(val); }
		double getDeriv() { return m_fdm->GetAuxiliary()->Getadot(); }
	};
	class Theta : public Component
	{
	public:
		Theta() : Component("Theta","rad") {};
		double get() { return m_fdm->GetPropagate()->GetEuler(2); }
		void set(double val) { std::cerr << "Theta::set not implemented." << std::endl; }
		double getIC() { return m_fdm->GetIC()->GetThetaRadIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetThetaRadIC(val); }
		double getDeriv() { return m_fdm->GetAuxiliary()->GetEulerRates(2); }
	};
	class Q : public Component
	{
	public:
		Q() : Component("Q","rad/s") {};
		double get() { return m_fdm->GetPropagate()->GetPQR(2); }
		void set(double val) { m_fdm->GetPropagate()->SetPQR(2,val); }
		double getIC() { return m_fdm->GetIC()->GetQRadpsIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetQRadpsIC(val); }
		double getDeriv() { return m_fdm->GetPropagate()->GetPQRdot(2); }
	};
	class Alt : public Component
	{
	public:
		Alt() : Component("Alt","ft") {};
		double get() { return m_fdm->GetPropagate()->GetAltitudeASL(); }
		void set(double val) { std::cerr << "Alt::set not implemented." << std::endl; }
		double getIC() { return m_fdm->GetIC()->GetAltitudeASLFtIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetAltitudeASLFtIC(val); }
		double getDeriv() { return m_fdm->GetPropagate()->Gethdot(); }
	};
	class Beta : public Component
	{
	public:
		Beta() : Component("Beta","rad") {};
		double get() { return m_fdm->GetAuxiliary()->Getbeta(); }
		void set(double val) { m_fdm->GetAuxiliary()->Setalpha(val); }
		double getIC() { return m_fdm->GetIC()->GetBetaRadIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetBetaDegIC(val); }
		double getDeriv() { return m_fdm->GetAuxiliary()->Getbdot(); }
	};
	class Phi : public Component
	{
	public:
		Phi() : Component("Phi","rad") {};
		double get() { return m_fdm->GetPropagate()->GetEuler(1); }
		void set(double val) { std::cerr << "Phi::set not implemented." << std::endl; }
		double getIC() { return m_fdm->GetIC()->GetPhiRadIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetThetaRadIC(val); }
		double getDeriv() { return m_fdm->GetAuxiliary()->GetEulerRates(1); }
	};
	class P : public Component
	{
	public:
		P() : Component("P","rad/s") {};
		double get() { return m_fdm->GetPropagate()->GetPQR(1); }
		void set(double val) { m_fdm->GetPropagate()->SetPQR(1,val); }
		double getIC() { return m_fdm->GetIC()->GetPRadpsIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetPRadpsIC(val); }
		double getDeriv() { return m_fdm->GetPropagate()->GetPQRdot(1); }
	};
	class R : public Component
	{
	public:
		R() : Component("R","rad/s") {};
		double get() { return m_fdm->GetPropagate()->GetPQR(3); }
		void set(double val) { m_fdm->GetPropagate()->SetPQR(3,val); }
		double getIC() { return m_fdm->GetIC()->GetRRadpsIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetRRadpsIC(val); }
		double getDeriv() { return m_fdm->GetPropagate()->GetPQRdot(3); }
	};
	class Psi : public Component
	{
	public:
		Psi() : Component("Psi","rad") {};
		double get() { return m_fdm->GetPropagate()->GetEuler(3); }
		void set(double val) { std::cerr << "Psi::set not implemented." << std::endl; }
		double getIC() { return m_fdm->GetIC()->GetPsiRadIC(); }
		void setIC(double val) { m_fdm->GetIC()->SetPsiRadIC(val); }
		double getDeriv() { return m_fdm->GetAuxiliary()->GetEulerRates(3); }
	};
	class ThrottleCmd : public Component
	{
	public:
		ThrottleCmd() : Component("ThrottleCmd","norm") {};
		double get() { return m_fdm->GetFCS()->GetThrottleCmd(0); }
		void set(double val) {
			for (int i=0;i<m_fdm->GetPropulsion()->GetNumEngines();i++)
				m_fdm->GetFCS()->SetThrottleCmd(i,val);
		}
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class ThrottlePos : public Component
	{
	public:
		ThrottlePos() : Component("ThrottlePos","norm") {};
		double get() { return m_fdm->GetFCS()->GetThrottlePos(0); }
		void set(double val) {
			for (int i=0;i<m_fdm->GetPropulsion()->GetNumEngines();i++)
				m_fdm->GetFCS()->SetThrottlePos(i,val);
		}
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class DaCmd : public Component
	{
	public:
		DaCmd() : Component("DaCmd","norm") {};
		double get() { return m_fdm->GetFCS()->GetDaCmd(); }
		void set(double val) { m_fdm->GetFCS()->SetDaCmd(val); }
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class DaPos : public Component
	{
	public:
		DaPos() : Component("DaPos","norm") {};
		double get() { return m_fdm->GetFCS()->GetDaLPos(); }
		void set(double val)
		{
			m_fdm->GetFCS()->SetDaLPos(ofRad,val);
			m_fdm->GetFCS()->SetDaRPos(ofRad,val); // TODO: check if this is neg.
		}
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class DeCmd : public Component
	{
	public:
		DeCmd() : Component("DeCmd","norm") {};
		double get() { return m_fdm->GetFCS()->GetDeCmd(); }
		void set(double val) { m_fdm->GetFCS()->SetDeCmd(val); }
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class DePos : public Component
	{
	public:
		DePos() : Component("DePos","norm") {};
		double get() { return m_fdm->GetFCS()->GetDePos(); }
		void set(double val) { m_fdm->GetFCS()->SetDePos(ofRad,val); }
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class DrCmd : public Component
	{
	public:
		DrCmd() : Component("DrCmd","norm") {};
		double get() { return m_fdm->GetFCS()->GetDrCmd(); }
		void set(double val) { m_fdm->GetFCS()->SetDrCmd(val); }
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class DrPos : public Component
	{
	public:
		DrPos() : Component("DrPos","norm") {};
		double get() { return m_fdm->GetFCS()->GetDrPos(); }
		void set(double val) { m_fdm->GetFCS()->SetDrPos(ofRad,val); }
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class Rpm : public Component
	{
	public:
		Rpm() : Component("Rpm","rev/min") {};
		double get() { return m_fdm->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM(); }
		void set(double val)
		{ 
			for (int i=0;i<m_fdm->GetPropulsion()->GetNumEngines();i++)
				m_fdm->GetPropulsion()->GetEngine(i)->GetThruster()->SetRPM(val);
		}
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class Pitch : public Component
	{
	public:
		Pitch() : Component("Pitch","deg") {};
		double get() { return m_fdm->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch(); }
		void set(double val)
		{ 
			for (int i=0;i<m_fdm->GetPropulsion()->GetNumEngines();i++)
				m_fdm->GetPropulsion()->GetEngine(i)->GetThruster()->SetPitch(val);
		}
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class N1 : public Component
	{
	public:
		N1() : Component("N1","rev/min") {};
		double get() 
		{
			FGEngine * engine = m_fdm->GetPropulsion()->GetEngine(0);
			switch(engine->GetType())
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
			std::cerr << "N1:set not imlemented." << std::endl;
		}
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
	class N2 : public Component
	{
	public:
		N2() : Component("N2","rev/min") {};
		double get() 
		{
			FGEngine * engine = m_fdm->GetPropulsion()->GetEngine(0);
			switch(engine->GetType())
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
			std::cerr << "N2:set not imlemented." << std::endl;
		}
		double getIC() { return get(); }
		void setIC(double val) { set(val); }
	};
    FGStateSpace(FGFDMExec & fdm) : m_x(), m_u(), m_fdm(fdm) {};
    virtual ~FGStateSpace(){};
	void addX(Component * comp) {comp->setFdm(&m_fdm); m_x.push_back(comp); }
	void addU(Component * comp) {comp->setFdm(&m_fdm); m_u.push_back(comp); }
	const std::vector<Component *> & getX() { return m_x; };
	const std::vector<Component *> & getU() { return m_u; };
private:
	std::vector<Component *> m_x, m_u; 
	FGFDMExec & m_fdm;
};

} // JSBSim

#endif

// vim:ts=4:sw=4
