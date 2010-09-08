clc; clear;
mode(-1)

exec EasyStar_lin.sce;

// rudder
plant = EasyStar.sys; // beta, phi, p, r, psi | rudder
[ap,bp,cp,dp] = abcd(plant);
lat.indx=[7,8,9,10,11];
lat.indy=[7,8,9,10,11];
lat.indu=[4];
lat.sys=syslin('c',ap(lat.indx,lat.indx),bp(lat.indx,lat.indu),cp(lat.indy,lat.indx),dp(lat.indx,lat.indu));

// actuator
actuator=tf2ss(20*%s/(20+%s));
plant_act = lat.sys*actuator; // series 2 * series 1

// yaw damper
gc_yawdamp = tf2ss(1); // pid compensator
yawdamp_open = plant_act*gc_yawdamp;
[a,b,c,d] = abcd(yawdamp_open);
yawdamp_closed = syslin('c',a+b*c(4,:),b,c,d); // simo
disp('yaw damper controller')
disp('phase margin:')
disp(p_margin(yawdamp_closed(4,:)))
disp('gain margin:')
disp(g_margin(yawdamp_closed(4,:)))

// phi loop
gc_phi = tf2ss(1); // pid compensator
phi_open = yawdamp_closed*gc_phi;
[a,b,c,d] = abcd(phi_open);
phi_closed = syslin('c',a+b*c(2,:),b,c,d); // simo
disp('phi controller')
disp('phase margin:')
disp(p_margin(phi_closed(2,:)))
disp('gain margin:')
disp(g_margin(phi_closed(2,:)))


// psi loop
gc_psi = tf2ss(1); // pid compensator
psi_open = phi_closed*gc_psi;
[a,b,c,d] = abcd(psi_open);
psi_closed = syslin('c',a+b*c(5,:),b,c,d);
disp('psi controller')
disp('phase margin:')
disp(p_margin(psi_closed(5,:)))
disp('gain margin:')
disp(g_margin(psi_closed(5,:)))

scf(1); clf(1);
f=gcf(); f.figure_name = "plant w/ actuator | rudder";
bode(plant_act)
legend('phi','psi')

scf(2); clf(2);
f=gcf(); f.figure_name = "yaw damper open/closed | rudder";
subplot(1,2,3);
bode(yawdamp_open(2,:))
subplot(1,2,3);
bode(yawdamp_closed(2,:))
subplot(1,3,3);
evans(yawdamp_closed(2,:))
legend('phi','psi')

scf(3); clf(3);
f=gcf(); f.figure_name = "phi open/closed | rudder";
subplot(1,2,1);
bode(phi_open(1,:))
subplot(1,2,2);
bode(phi_closed(1,:))
legend('phi','psi')

scf(4); clf(4);
f=gcf(); f.figure_name = "psi open/closed | rudder";
subplot(1,2,1);
bode(psi_open(3,:))
subplot(1,2,2);
bode(psi_closed(3,:))
legend('phi','psi')
