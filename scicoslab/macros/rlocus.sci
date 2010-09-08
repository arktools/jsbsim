function rlocus(G_open,kmin,kmax,axis)
mode(1)  
// figure setup
scf(1)

// closed loop
closedLoopPoles=[];
for kval=logspace(kmin,kmax)
	//solve close loop char. eq
	ce = denom(G_open)+kval*numer(G_open);
	closedLoopPoles = [closedLoopPoles,roots(ce)]; 
end
real(closedLoopPoles)
plot(real(closedLoopPoles),imag(closedLoopPoles),'r');
plot(real(closedLoopPoles),-imag(closedLoopPoles),'g');

// open loop
openLoopZeros = roots(numer(G_open));
plot(real(openLoopZeros),imag(openLoopZeros),'bo');
openLoopPoles = roots(denom(G_open));
plot(real(openLoopPoles),imag(openLoopPoles),'bx');
//mtlb_axis(axis);

endfunction
