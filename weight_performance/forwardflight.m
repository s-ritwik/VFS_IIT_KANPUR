function [Prange,Vrange,Pendu,Vendu]=forwardflight(R,Vtip,Thrust,nondp,solidity,height)
%--------------------calculation of forward fight using momentum theory----
P0 = 1.01325*(10^5);%atmospheric pressure                  
T0 = 288.16+15;%30 degrees(15 deg extra taken here for ISA)                              
rho0 = 1.225;%density at sea level
[rho,P,T]=atmos(height,P0,T0,rho0);
K=1.15;%induce power factor
A=pi*(R^2);%disk area
f=0.2*A;%flat plate area
%-----------power and velocity for best range-----------------------------
% Vrange=((GW/(2*rho*A))^0.5)*((4*K/(F/A))^0.25);
% mu_range=Vrange/Vtip;
% Cp_range=(K*Ct^2)/(2*mu_range) + 0.5*mu_range^3*(F/A);
% Prange=Cp_range*nondp*1.15;%addition 15% for losses
% %-----------power and velocity for best endurance-------------------------
% Vendu=((GW/(2*rho*A))^0.5)*((4*K/(3*F/A))^0.25);
% mu_endu=Vendu/Vtip;
% Cp_endu=(K*Ct^2)/(2*mu_endu) + 0.5*mu_endu^3*(F/A);
% Pendu=Cp_endu*nondp*1.15;%additional 15% for losses
i=1;
for k=0:10:400 %velocity in kmph
    Vforward(i) = k*1000/3600;% in m/s
    [Pfi(i),Pf0(i),Pfp(i) ,Pft(i)] = Powerff(f,R,Thrust,rho,solidity,Vtip,Vforward(i),nondp);
    myu(i) = Vforward(i)/Vtip;
    i=i+1;
end
%Power consumption due to transmission loss will be ~15 % of the total power.
Pft_ACC = Pft*1.10;
%plotting
%---------------------------TOTAL POWER------------------------------------ 
% plot(Vforward(1:i-1)*18/5,Pft_ACC(1:i-1)/745.7,'-k*','LineWidth',1);
% plot(Vforward(1:i-1)*18/5,Pfi(1:i-1),'-bs','LineWidth',1);
% hold on;
% plot(Vforward(1:i-1)*18/5,Pfp(1:i-1),'-ko','LineWidth',1);
% plot(Vforward(1:i-1)*18/5,Pf0(1:i-1),'-gx','LineWidth',1);
% plot(Vforward(1:i-1)*18/5,Pft(1:i-1),'-r*','LineWidth',1);
% plot(Vforward(1:i-1)*18/5,Pft_ACC(1:i-1),'-c*','LineWidth',1)
% legend('Induced Power','Parasite Power','Profile Power','Total Power','Total Power with losses');
% title('Total Power - Forward Flight');
% xlabel('Forward Velocity (Kmph)') % x-axis label
% ylabel('Power (W)') % y-axis label
% grid on;
for j=1:i
slope(j)=(Pft_ACC(j))/(Vforward(j)*18/5);
next = slope(j)*Vforward(j+1)*18/5;
    if next < (Pft_ACC(j+1))
        finalslope = slope(j);
        break;
    end
end
for j=1:i
    check1=finalslope*Vforward(j)*18/5;
    compare = @(n1,n2,n_dcp) round(n1*10^n_dcp)==round(n2*10^n_dcp);
    if compare(check1,(Pft_ACC(j)),4)
        Vrange = Vforward(j)*18/5;
        Prange = check1;
        break;
    end
end

Pendu=min((Pft_ACC(1:i-1)));
for j=1:i
    if Pendu == (Pft_ACC(j))
       Vendu = Vforward(j)*18/5;
       break;
    end
end
end

