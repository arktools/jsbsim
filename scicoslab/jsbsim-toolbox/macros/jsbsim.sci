function jsbsim(str,root)
	[lhs,rhs] = argn()
	if (rhs == 1) then
		root=jsbsimRoot	
	elseif (rhs ~= 2) then
		error('wrong number of args, usage: jsbsim(str,root)')
	end
	host(jsbsimRoot+'/src/JSBSim --root='+root+' '+str) 
endfunction
