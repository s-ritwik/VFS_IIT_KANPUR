clear all;
clc;
rho=0.8789;
R= 1;
c=1/11;
twist=0;
rpm=1336;
omega= 2*pi*rpm/60;
Nb=3;
GW= 150/4; %weight per rotor
trans_loss=1.1;
electrical_loss=1.05;
nondp= rho*(pi*R^2)*(omega*R)^3;
motor_efficiency=0.85;
nondt=rho*(pi*R^2)*(omega*R)^2;
theta_0= (0:1:15)*pi/180;
air=2;

for k=1: length(theta_0)    
[Thrust(k),Power(k),torque_h(k),theta_h(k),err(k),FM(k),BL(k),power_h_mech(k),CP(k),CT(k)] = Rotor_opt(R,c,twist,rpm,Nb,air,GW,trans_loss,nondp,motor_efficiency,nondt,theta_0(k),electrical_loss,rho);
end
plot(CT,CP);
hold on;
xlabel("CT");
ylabel("CP");
%legend("NACA 4412","NACA 653618","NACA")