load BacksidePIDAutopilot.cos
servoPlantNum=529; // watch this, can change when saving
controllerPlantnum = 394; // watch this, can change when saving
exec EasyStar_lin.sce;
sys_airframe = sys;
sys_servos = lincos(scs_m.objs(servoPlantNum).model.rpar,u0,u0);
u0controller = [x0;x0(5);x0(1);x0(10);x0(9)];
x0controller = [zeros(10,1)]; // setting integrator/low pass states to zero
sys_controller = lincos(scs_m.objs(controllerPlantnum).model.rpar,..
  x0controller,u0controller);
sys = sys_airframe*sys_servos*sys_controller;
sysc = sys/(1+sys);

plot(linspace(0,100),csim("step",linspace(0,100),sysc(5,14)))
