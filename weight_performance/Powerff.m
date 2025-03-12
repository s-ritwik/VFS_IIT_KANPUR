function [Pi,P0,Pp,Pt] = Powerff(f,Rfr,T,rho,sigma,Vtip,Vforward,nondp)
K=1.15; %tip loss factor
myu = Vforward/Vtip; %advance ratio
Cd0=0.028;
Ph0 = 0.125*sigma*Cd0*(1+K*myu^3)*nondp; % Hover Profile Power
P0 = 2*((1+4.6*myu^2)*0.125*sigma*Cd0); %Forward Flight Profile Power(front+back)

Pp =2* 0.5*rho*Vforward^3*f; %Forward Flight Parasite Power

Vi = sqrt(T/(2*rho*pi*Rfr^2));
% Vit^4+2*Vvertical*Vit^3+(Vforward^2+Vvertical^2)*Vit^2-Vi^4=0;
Vvertical = 0;
eqn=[1 2*Vvertical (Vforward^2+Vvertical^2) 0 -Vi^4]; %solving polynomial equation
root = roots(eqn);
Vit=root(real(root)>0&imag(root)==0);
Pi = 2*(K)*(T*Vit); %Forward FLight induced Power(front +back)

Pt = P0 + Pi + Pp;   %Total Power of front + back in forward flight

end