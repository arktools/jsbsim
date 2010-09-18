function [x,y,typ]=ardupilotmegaHil(job,arg1,arg2)
//
// ardupilotmegaHil.sci
// Copyright (C) James Goppert 2010 <jgoppert@users.sourceforge.net>
//
// ardupilotmegaHil.sci is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// ardupilotmegaHil.sci is distributed in the hope that it will be useful, but
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
			labels=['device';'baud rate'];
			[ok,device,baudRate,exprs]=..
				getvalue('Set ArduPilotMega HIL Parameters',labels,..
				list('str',-1,'vec',1),exprs);
			if ~ok then break,end
			[model,graphics,ok]=check_io(model,graphics,[13],[4],[1],[])
			if ok then
				model.ipar=[..
					length(evstr(device)),ascii(evstr(device)),0,..
					baudRate];
				graphics.exprs=exprs;
				x.graphics=graphics;
				x.model=model;
				break
			end
		end
	case 'define' then
		// set model properties
		model=scicos_model()
		model.sim=list('sci_ardupilotmegaHil',4)
		model.in=[13]
		model.out=[4]
		model.evtin=[1]
		model.blocktype='c'
		model.dep_ut=[%f %t]

		// jsbsim parameters
		device="""/dev/ttyUSB1""";
		baudRate=38400;
		model.ipar=[..
					length(evstr(device)),ascii(evstr(device)),0,..
					baudRate];

		// initialize strings for gui
		exprs=[strcat(device),strcat(sci2exp(baudRate))];

		// setup icon
	  	gr_i=['xstringb(orig(1),orig(2),''ArduPilotMega HIL'',sz(1),sz(2),''fill'');']
	  	x=standard_define([5 2],model,exprs,gr_i)
	end
endfunction

// vim:ts=4:sw=4
