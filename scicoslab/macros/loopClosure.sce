clc; clear;
Gc_phi = 1;
Gc_psi = 1;

exec EasyStar_lin.sce;
plant = EasyStar.sys([7,11],4); // phi, psi | rudder
[ap,bp,cp,dp] = abcd(plant);

// actuator
actuator=tf2ss(20*%s/(20+%s));
sys1 = plant*actuator; // series 2 * series 1
[a1,b1,c1,d1] = abcd(sys1);

// phi loop
k_phi = .00000001;
ac1 = a1 + b1*[k_phi 0]*c1; // close phi loop
spec(ac1)
phi_closed = syslin('c',ac1,b1,c1(1,:),0); // siso for phi

// psi loop
k_psi = .000001;
sys2 = phi_closed*k_psi; // series 2 * series 1
[a2,b2,c2,d2] = abcd(sys2);
ac2 = a2 - b2*k_psi*c2;
spec(ac2)
psi_closed = syslin('c',ac2,b2,c2,0);

scf(1); clf(1);
f=gcf(); f.figure_name = "phi, psi | rudder";
subplot(1,3,1)
bode(sys1)
legend('phi','psi');
subplot(1,3,2)
evans(sys1(1,1))
title('phi')
subplot(1,3,3)
evans(sys1(2,1))
title('psi')

scf(2); clf(2);
f=gcf(); f.figure_name = "phi closed | rudder";
bode(phi_closed(:,1))

scf(3); clf(3);
f=gcf(); f.figure_name = "psi closed | rudder";
bode(psi_closed(:,1))
