clc; clear

exec EasyStar_lin.sce;

load JSBSimCommPorts.cos
sys_airframe = lincos(scs_m,x0,u0);
commandedStates=[5;1;9;10;7];

disp(norm(sys_airframe.A-sys.A))
sys_airframe = sys_airframe(commandedStates,:);

load BacksidePIDAutopilot.cos
servoPlantNum=529; // watch this, can change when saving
controllerPlantnum = 394; // watch this, can change when saving


sys_servos = lincos(scs_m.objs(servoPlantNum).model.rpar,u0,u0);
sys_controller = lincos(scs_m.objs(controllerPlantnum).model.rpar,..
  zeros(10,1),.. // set low pass filter poles and integratotr poles to zero
  zeros(5,1)); // set initial command errors to zero
sys2 = sys_airframe*sys_servos;

//[a,b,c,d] = abcd(sys2);
//ac1 = a - b*c;
//s1 = syslin('c',ac1,b,c,d);
t=linspace(0,10);

n=4;
//s2 = s1(n,n)/(1+s1(n,n));

scf(1); clf(1);
plot(t,csim("step",t,sys2(n,n)));
