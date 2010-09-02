function [x,y,typ]=jsbsimComm(job,arg1,arg2)
//
// flightGearComm.sci
// Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
//
// flightGearComm.sci is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// flightGearComm.sci is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.
//

x=[];y=[];typ=[];

select job
	case 'plot' then
	 	standard_draw(arg1)
	case 'getinputs' then
	 	[x,y,typ]=standard_inputs(arg1)
	case 'getoutputs' then
	 	[x,y,typ]=standard_outputs(arg1)
	case 'getorigin' then
	 	[x,y]=standard_origin(arg1)
	case 'set' then
		x=arg1;
		graphics=arg1.graphics;exprs=graphics.exprs
		model=arg1.model;
		while %t do
			labels=[..
				'model name';..
				'aircraft path';..
				'engine path';..
				'systems path';..
				'velocity, fps';..
				'alpah,rad';..
				'pitch, rad';..
				'pitchRate, rad/s';..
				'altitude ASL, ft';..
				'sideSlip, rad';..
				'roll, rad';..
				'rollRate, rad/s';..
				'yawRate, rad/s';..
				'heading, rad';..
				'rpm'];
			[ok,ModelName,AircraftPath,EnginePath,SystemsPath,..
			velocityTrue,alpha,pitch,pitchRate,altitude,sideSlip,..
			roll,rollRate,yawRate,heading,rpm,exprs]=..
				getvalue('Set JSBSim Parameters',labels,..
				list('str',-1,'str',-1,'str',-1,'str',-1,..
					'vec',1,'vec',1,'vec',1,'vec',1,..
					'vec',1,'vec',1,'vec',1,'vec',1,..
					'vec',1,'vec',1,'vec',1),exprs);
			if ~ok then break,end
			[model,graphics,ok]=check_io(model,graphics,[4],[11;8],[],[])
			if ok then
				model.state=[velocityTrue;alpha;pitch;pitchRate;altitude;..
					sideSlip;roll;rollRate;yawRate;heading;rpm];
				graphics.exprs=exprs;
				x.graphics=graphics;
				x.model=model;
				break
			end
		end
	case 'define' then
		// set model properties
		model=scicos_model()
		model.sim=list('sci_jsbsimComm',4)
		model.in=4
		model.out=[11;8]
		model.blocktype='c'
		model.dep_ut=[%f %t]

		// jsbsim parameters
		ModelName="EasyStar";
		AircraftPath="~/Projects/oooark/data/";
		EnginePath="~/Projects/oooark/data/EasyStar/Engines";
		SystemsPath="~/Projects/oooark/data/EasyStar/Engines";
			
		// intial state
		velocityTrue=40; // ft/s
		alpha=0; // rad
		pitch=0; // rad
		pitchRate=0;  // rad/s
		altitude=1000; // ft ASL, don't want to start in ground
		sideSlip=0; // rad
		roll=0; // rad
		rollRate=0; // rad/s
		yawRate=0; // rad/s
		heading=0; // rad
		rpm=0; // rpm

		// save state
		model.state=[velocityTrue;alpha;pitch;pitchRate;altitude;..
			sideSlip;roll;rollRate;yawRate;heading;rpm];

			// initialize strings for gui
		exprs=[strcat(ModelName),strcat(AircraftPath),strcat(EnginePath),strcat(SystemsPath),..
		strcat(sci2exp(velocityTrue)),strcat(sci2exp(alpha)),..
		strcat(sci2exp(pitch)),strcat(sci2exp(pitchRate)),..
		strcat(sci2exp(altitude)),strcat(sci2exp(sideSlip)),..
		strcat(sci2exp(roll)),strcat(sci2exp(rollRate)),..
		strcat(sci2exp(yawRate)),strcat(sci2exp(heading)),strcat(sci2exp(rpm))];

		// setup icon
	  	gr_i=['xstringb(orig(1),orig(2),''JSBSimComm'',sz(1),sz(2),''fill'');']
	  	x=standard_define([5 2],model,exprs,gr_i)
	end
endfunction

// vim:ts=4:sw=4
