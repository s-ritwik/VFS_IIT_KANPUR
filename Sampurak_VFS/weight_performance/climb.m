function [Pc,Pd]=climb(T,rho,R,Ph,Vc,Vd)
A=pi*(R^2);%disk area
Vh = sqrt(T/(2*rho*A));
K=1.15;
K1=-1.125;
K2=-1.372;
K3=-1.718;
K4=-0.655;
% i=1;
% for k= 0:1:20 %climb velocity in m/s
%     Vc(i)= k;
%     Vi_Vh(i)= sqrt((Vc(i)/(2*Vh))^2 +1)-Vc(i)/(2*Vh);
%     Vi(i)=Vi_Vh(i)*Vh;
%     Vc_Vh(i) =Vc(i)/Vh;
%     P_Ph(i)= Vc(i)/(2*Vh)+sqrt((Vc(i)/(2*Vh))^2 +1);
%     P(i)=P_Ph(i)*Ph;
%     i=i+1;
% end
% plot(Vc_Vh,Vi_Vh);
% xlabel("V_c/V_h");
% ylabel("V_i/V_h");
% title("induced speed vs climb speed");
% figure;
% plot(Vc_Vh,P_Ph);
% xlabel("V_c/V_h");
% ylabel("P/P_h");
% title("Power in climb to hover");
Pc=(Vc/(2*Vh)+sqrt((Vc/(2*Vh))^2 +1))*Ph;
%Pd=2*(Vd/(2*Vh)-sqrt((Vd/(2*Vh))^2 -1))*Ph;
% j=1;
%for l= 0:1:20 %climb velocity in m/s
%     Vd(j)= l;
%     Vi_Vh(j)= sqrt((Vd(j)/(2*Vh))^2 +1)-Vd(j)/(2*Vh);
%     Vi(j)=Vi_Vh(j)*Vh;
%     Vd_Vh(j) =Vd(j)/Vh;
%     P_Ph(j)= Vd(j)/(2*Vh)+sqrt((Vd(j)/(2*Vh))^2 +1);
%     P(j)=P_Ph(j)*Ph;
%     j=j+1;
Vi_descent=(K+K1*(Vd/Vh)+K2*(Vd/Vh)^2+K3*(Vd/Vh)^3+K4*(Vd/Vh)^4)*Vh;
Pd=(T*(Vd+Vi_descent));
end
