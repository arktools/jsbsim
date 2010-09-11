clc; clear;
mode(-1)

exec EasyStar_lin.sce;

plant.x.vt=1;
plant.x.alpha=2;
plant.x.theta=3;
plant.x.q=4;
plant.x.rpm=5;
plant.x.alt=6;
plant.x.beta=7;
plant.x.phi=8;
plant.x.p=9;
plant.x.r=10;
plant.x.psi=11;
plant.x.lon=12;
plant.x.lat=13;

plant.y.vt=1;
plant.y.alpha=2;
plant.y.theta=3;
plant.y.q=4;
plant.y.rpm=5;
plant.y.alt=6;
plant.y.beta=7;
plant.y.phi=8;
plant.y.p=9;
plant.y.r=10;
plant.y.psi=11;
plant.y.lon=12;
plant.y.lat=13;

plant.u.th=1;
plant.u.da=2;
plant.u.de=3;
plant.u.dr=4;

// complete plant
plant.sys = minreal(EasyStar.sys);
[ap,bp,cp,dp] = abcd(plant.sys);

// lateral subsystem
latSub.indx=[plant.x.beta, plant.x.phi, plant.x.p, plant.x.r, plant.x.psi];
latSub.indy=[plant.y.beta, plant.y.phi, plant.y.p, plant.y.r, plant.y.psi];
latSub.indu=[plant.u.da, plant.u.dr];
latSub.sys=syslin('c',ap(latSub.indx,latSub.indx),bp(latSub.indx,latSub.indu),..
  cp(latSub.indy,latSub.indx),dp(latSub.indx,latSub.indu));
  
latSub.x.beta=1;
latSub.x.phi=2;
latSub.x.p=3;
latSub.x.r=4;

latSub.y.beta=1;
latSub.y.phi=2;
latSub.y.p=3;
latSub.y.r=4;

latSub.u.da=1;
latSub.u.dr=2;

// longitudinal subsystem
lonSub.indx=[plant.x.vt, plant.x.alpha, plant.x.theta, plant.x.q, plant.x.rpm, plant.x.alt];
lonSub.indy=[plant.y.vt, plant.y.alpha, plant.y.theta, plant.y.q, plant.y.rpm, plant.y.alt];
lonSub.indu=[plant.u.th, plant.u.de];
lonSub.sys=syslin('c',ap(lonSub.indx,lonSub.indx),bp(lonSub.indx,lonSub.indu),..
  cp(lonSub.indy,lonSub.indx),dp(lonSub.indx,lonSub.indu));
  
lonSub.x.vt=1;
lonSub.x.alpha=2;
lonSub.x.theta=3;
lonSub.x.q=4;
lonSub.x.rpm=5;
lonSub.x.alt=6;

lonSub.y.vt=1;
lonSub.y.alpha=2;
lonSub.y.theta=3;
lonSub.y.q=4;
lonSub.y.rpm=5;
lonSub.y.alt=6;

lonSub.u.th=1;
lonSub.u.de=2;



// rudder
//plant = minss(EasyStar.sys); // beta, phi, p, r, psi | rudder


//lat.sys=syslin('c',ap(lat.indx,lat.indx),bp(lat.indx,lat.indu),cp(lat.indy,lat.indx),dp(lat.indx,lat.indu));

// actuator
//actuator=tf2ss(20*%s/(20+%s));
//plant_act = plant*actuator; // series 2 * series 1

// yaw damper
gc_yawdamp = tf2ss(10*(%s+20)/%s); // pid compensator
yawdamp_open = latSub.sys(:,latSub.u.dr)*gc_yawdamp;
[a,b,c,d] = abcd(yawdamp_open);
yawdamp_closed = syslin('c',a+b*c(latSub.y.r,:),b,c,d); // simo
disp('yaw damper controller')
disp('phase margin:')
disp(p_margin(yawdamp_open(latSub.y.r,:)))
disp('gain margin:')
disp(g_margin(yawdamp_open(latSub.y.r,:)))

// phi
gc_phi = tf2ss((%s+1)/%s); // pid compensator
phi_open = latSub.sys(:,latSub.u.dr)*gc_phi;
[a,b,c,d] = abcd(phi_open);
phi_closed = syslin('c',a+b*c(latSub.y.phi,:),b,c,d); // simo
disp('yaw damper controller')
disp('phase margin:')
disp(p_margin(yawdamp_open(latSub.y.phi,:)))
disp('gain margin:')
disp(g_margin(yawdamp_open(latSub.y.phi,:)))

//gc_phi = tf2ss(1); // pid compensator
//phi_open = yawdamp_closed*gc_phi;
//[a,b,c,d] = abcd(phi_open);
//phi_closed = syslin('c',a+b*c(2,:),b,c,d); // simo
//disp('phi controller')
//disp('phase margin:')
//disp(p_margin(phi_closed(2,:)))
//disp('gain margin:')
//disp(g_margin(phi_closed(2,:)))


//psi loop
//gc_psi = tf2ss(1); pid compensator
//psi_open = phi_closed*gc_psi;
//[a,b,c,d] = abcd(psi_open);
//psi_closed = syslin('c',a+b*c(5,:),b,c,d);
//disp('psi controller')
//disp('phase margin:')
//disp(p_margin(psi_closed(5,:)))
//disp('gain margin:')
//disp(g_margin(psi_closed(5,:)))

scf(1); clf(1);
f=gcf(); f.figure_name = "yaw rate -> rudder | open bode /closed bode /closed evans";
subplot(1,3,1)
bode(yawdamp_open(latSub.y.r,:));
subplot(1,3,2)
bode(yawdamp_closed(latSub.y.r,:))
subplot(1,3,3)
e=gce(); p1=e.children(1); p2=e.children(2);
p1.foreground=color("purple"); p2.foreground=color("navy blue");
cmap=xget("colormap"); cmap(8,:)=0; xset("colormap",cmap); // fix white
evans(minss(yawdamp_closed(latSub.y.r,:),.1e-2),100)
mtlb_axis([-30,40,-30,30])

scf(2); clf(2);
f=gcf(); f.figure_name = "phi -> rudder | open bode /closed bode /closed evans";
subplot(1,3,1)
bode(phi_open(latSub.y.phi,:));
subplot(1,3,2)
bode(phi_closed(latSub.y.phi,:))
subplot(1,3,3)
e=gce(); p1=e.children(1); p2=e.children(2);
p1.foreground=color("purple"); p2.foreground=color("navy blue");
cmap=xget("colormap"); cmap(8,:)=0; xset("colormap",cmap); // fix white
evans(minss(phi_closed(latSub.y.phi,:),.1e-2),10)
mtlb_axis([-30,40,-30,30])

scf(3); clf(3);


//scf(3); clf(3);
//f=gcf(); f.figure_name = "phi open/closed | rudder";
//subplot(1,2,1);
//bode(phi_open(1,:))
//subplot(1,2,2);
//bode(phi_closed(1,:))
//legend('phi','psi')

//scf(4); clf(4);
//f=gcf(); f.figure_name = "psi open/closed | rudder";
//subplot(1,2,1);
//bode(psi_open(3,:))
//subplot(1,2,2);
//bode(psi_closed(3,:))
//legend('phi','psi')
