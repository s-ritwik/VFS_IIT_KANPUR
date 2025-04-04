clc;    
clear;
clear all;
%% --------------------------------Mission requirements--------------------------------------------------------------%
h=10; %m
range = .5; %km
climb_alt=.1;%climb altitude in km
mpayload = 185; %(kg)payload 
L_by_D=8; % estimated L/D ratio
%% --------------------------------Atmosphere data-------------------------------------------------------------------%
P0 = 1.01325*(10^5);%atmospheric pressure                  
T0 = 288.16+15;%30 degrees(15 deg extra taken here for IRA)                              
rho0 = 1.225;%density
[rho,P,T]=atmos(h,P0,T0,rho0);
g = 9.81; %m/s^2
gama = 1.4;
R = 287;%universal gas constant(KJ/(kgmol-k))
a = sqrt(gama*R*T);%speed of sound
Cd=0.011;% drag coefficient
cl_alpha=5.73;%lift coefficent
%% --------------------------------Baseline Parameters---------------------------------------------------------------%
R=1.1%0.8:0.05:1.15;%(in meters)
RPM= 4000%2500:500:6000;
%%
Nb = 3;%no of blades 
N_rotors=8; %no of motors/rotors
N_rotors_cruise=8; %no of motors/rotors active in cruise fixed wing mode

Vc=0.76;%climb speed in m/s
Vd=0.5;% descent speed in m/s
V_cruise=40;
V_cruise_5=40;

% --------------section 9 where you have to maximise T9---------------------------
T9=0.5*60*60; %1 hr
V_endu_cruise=30;

AR = 12;% aspect ratio of rotor
theta0  =(10:0.01:30)*pi/180;% collective
thetatw = -16;% twist rate
%Vtip_fr=(900:50:1300).*(2*pi*R_fr/60);%(2*pi*1400/60)*R_fr;% tip velocity in m/s
%rpm=(Vtip_fr*60/(2*pi*R_fr));%blade rpm
S_fr =12; %no of cells in series in a battery(44.4v/3.7v)
S_br =12; %no of cells in series(same voltage considered for back rotor)
% ----------------------wing-----------------------------------
AR_wing=12;% aspect ratio of wing
taper_ratio=0.4; 
tc_ratio = 0.25; % Thickness to chord ratio
Cl_design=0.8;% Cl_design
%% --------------------------------Losses & Efficiencies-------------------------------------------------------------%
trans_loss=1.05;%transmission losses(2%)
electrical_loss=1.02;%electrical losses(2%)
motor_efficiency=0.85;%motor efficiency(85%)
%GR=5;%Gear ratio
%rpm_m=GR*rpm;
%motor_efficiency=(-5.655e-8)*(rpm_m)^2 + 0.0003249*(rpm_m) + 0.3692;%motor efficiency(function of motor rpm)
mu=1.09;
%% --------------------------------Fixed weights----------------------------------------------------------------------
%mmotor=1.74*4;%we are using 4 motors(Tmotor U15 2 KV80),total power=34.3 KW
%mesc=0.558*4;%we are using 4 esc,(FLAME 200A 14S) 1 for each motor
%% --------------------------------Constants----------------------------------------------------------------------

kg_to_lb = 2.20462; % kg to pounds
m_to_ft = 3.28084; % meters to feet


%%                                 Design code
for i=1:size(R,2)
    
    for j=1:size(RPM,2)
        
        solution_check=1;
        Vtip=(RPM(j)).*(2*pi.*R(i)/60);
        fprintf('%4.3f RPM- %4.3f radius-%4.3f vtip:%4.3f \n',Nb, RPM(j), R(i), Vtip);
%-------ROTOR PARAMETERS--------------------------------------------------------------------------------------------------%
        c(i,j) =R(i)/AR;%chord
        omega(i,j) = Vtip/R(i);%angular velocity
        solidity(i,j) =(Nb*c(i,j)*(R(i)-0.2*R(i)))/(pi*R(i).^2);%solidity     
%-------DISTANCE BETWEEN ROTORS------------------------------------------------------------------------------------
        L = R(i)+R(i)+(2*R(i)/3);%center to center distance,clearance taken as diameter/3
%-------WEIGHT INITIALIZATION-----------------------------------------------------------------------------------------%
        m=1;%first index initialization
        n=2;% second index initialization
        GW(m) =1;%(KG) 
        mempty =24;%empty weight(kg)
        GW(n) = mempty+mpayload;%total gross weight
        pre_weight=GW(n);%intial gross weight assumption
        A=pi*(R(i)^2);%area of disk(for both front and back it will be same)
        nondt=rho*A*(omega(i,j)*R(i))^2;%non-dimentional making terms for thrust
        nondp=rho*A*(omega(i,j)*R(i))^3;%non-dimentional making terms for pressure
        error=100;%arbitrary value of error for initialization
        batt_reserve=0.85;% 15% battery reserve
        No_of_battery=2;% no of batteries each side
        fprintf(".");
        Nominal_volt=3.6*S_fr; %Nominal voltage of the entire one battery(cells in series)
        while( error>0.2 )
            count_k=0;
%-----------Thrust and Power calculation using momentum theory(hover)-----------------------------------------------------
            Ct=GW(n)*9.81/nondt;%considering thrust equal to weight for hover
            Cpi=1.15*(Ct^1.5)/sqrt(2);%induced power coefficient
            Cpp=solidity(i,j)*Cd/8;%profile power coefficient
            collective(i,j)=((6*Ct/(solidity(i,j)*cl_alpha)) + 1.5*sqrt(Ct/2))*180/pi;%theta in degrees
%-----------Thrust and Power calculation using
%BEMT(hover)--------------------------inclding binary search for
%optimisation
            % Initialize the binary search boundaries
            k_min = 1;
            k_max = length(theta0);
            found = false;
            
            while k_min <= k_max
                k_mid = floor((k_min + k_max) / 2);
                % fprintf("kmin= %4.3f kmid= %4.3f kmax= %4.3f \n", k_min,k_mid,k_max);
                 % fprintf("theta: %4.3f \n",theta0); 
                %_____________________________AIR=1 for naca4412 AND AIR=2
                %FOR NACA 653618
                % Call the Rotor_opt function

                %____________________________ROTOR OPT__________
                try

                [thrust_1, power_1, torque_1, theta_1, err_1, FM, BL, mech_power,CP,CT] = ...
                    Rotor_opt(R(i), c(i,j), thetatw, RPM(j), Nb, 3, GW(n), trans_loss, nondp, motor_efficiency, nondt, theta0(k_mid), electrical_loss, rho,N_rotors);
                    %2 is for airfoil
                
                %------------BEMT------------------------------------
                 % [thrust_1,power_1,torque_1,theta_1,err_1,FM,BL,mech_power]=BEMT(R_fr(i),Ct,Nb_fr,c_fr(i,j),Vtip_fr(j),a,GW(n),trans_loss,nondp,motor_efficiency,nondt,theta0(k_mid),electrical_loss);
                catch
                    continue
                end


                % Check the error condition
                if err_1 > 5 
                    % Condition is satisfied, so move left to find a smaller k
                    k_max = k_mid - 1;
                    thrust_h(i,j) = thrust_1;
                    power_h(i,j) = power_1;
                    torque_h(i,j) = torque_1;
                    C_P(i,j) = CP;
                    C_T(i,j)= CT;
                    theta_h(i,j) = theta0(k_mid);
                    err(i,j) = err_1;
                    power_mech(i,j) = mech_power;
                    count_k = k_mid;
                    found = true;
                else
                    % Condition is not satisfied, so move right to find a larger k
                    k_min = k_mid + 1;
                    % thrust_h(i,j) = 0;
                    % power_h(i,j) = 0;
                    % torque_h(i,j) = 0;
                    % theta_h(i,j) = 0;
                    % Profile_power(i,j) = 0;
                    % Induced_power(i,j) = 0;
                end
            end
            k_min2 = 1000;
            k_max2 = RPM(j);
            found2 = false;
            while k_min2 <= k_max2
                
                k_mid2 = floor((k_min2 + k_max2) / 2);
                omega2(i,j)=k_mid2*2*pi/60;
                nondt2=rho*A*(omega2(i,j)*R(i))^2;%non-dimentional making terms for thrust
                nondp2=rho*A*(omega2(i,j)*R(i))^3;%non-dimentional making terms for pressure
                % for cruise fixed wing
                [thrust_2, power_2, torque_2, theta_2, err_2, FM2, BL2, mech_power2,CP2,CT2] = ...
                    Rotor_opt(R(i), c(i,j), thetatw, k_mid2, Nb, 3, GW(n)/L_by_D/N_rotors_cruise, trans_loss, nondp2, motor_efficiency, nondt2, theta_h(i,j), electrical_loss, rho,1);
                if err_2 > 5
                    k_max2 = k_mid2 - 1;
                    err2(i,j)= err_2;
                    Cruise_RPM(i,j)=k_mid2;
                    power_cruise_hover(i,j)=power_2;% 0.85 is aerodynamic efficiency
                else
                    k_min2 = k_mid2 + 1;

                end
                % fprintf("1");
            end
            if ~found
                fprintf("solution not possible");
                solution_check = 0;
                break;
            else
                % fprintf("done");
            end
            
%-----------Auxillary power----------------------------------------------------------------------------------------
            Power_servo=100;% assumption
            Power_camera=25;%25W
            aux_power=Power_servo+Power_camera;
            aux_power=5000; %assumpption 
%-----------Total power and thrust in hover--------------------------------------------------------------------------- 
            Power_total_hover(i,j)=power_h(i,j)+aux_power;
            thrust_total(i,j)=thrust_h(i,j);
            Pclimb(i,j)=0;Pdescent(i,j)=0;

            %----Climb power------------------------for vc and vd
            try
            [Pclimb(i,j),Pdescent(i,j)]=climb(thrust_h(i,j)/N_rotors,rho,R(i),Power_total_hover(i,j)/N_rotors,Vc,Vd);
            catch
                Pclimb(i,j)=0;
                Pdescent(i,j)=0
                continue;  % Skip to next (j) iteration

            end
            %------------calculation using MT(forward-flight)---------------------   
            try
                [Prange(i,j), Vrange(i,j), Pendu(i,j), Vendu(i,j)] = forwardflight(R(i), Vtip, thrust_h(i,j)/N_rotors, nondp, solidity(i,j), h);
            catch
                Prange(i,j) = 0;
                Vrange(i,j) = 0;
                Pendu(i,j) = 0;
                Vendu(i,j) = 0;
                continue;  % Skip to next (j) iteration

            end
            %----Cruise power------------------------for vc and vd
            try
            [Pcruise(i,j),P_rand(i,j)]=climb(thrust_h(i,j)/N_rotors_cruise/L_by_D,rho,R(i),power_cruise_hover(i,j),V_cruise_5,Vd);
            catch
                Pcruise(i,j)=0;
                P_rand(i,j)=0
                continue;  % Skip to next (j) iteration

            end
            Power_cruise(i,j)= GW(n)/L_by_D*V_cruise_5/motor_efficiency*trans_loss*electrical_loss/0.7;
            power_cruise(i,j)= Pcruise(i,j)*N_rotors_cruise;
%-----------Total energy and battery capacity required based on mission

%-----------Section 2 : 15 seconds hover HIGE (i havent considered HIG yet)--------------------------------------------------------------
            e_section_2(i,j)= Power_total_hover(i,j)*15/motor_efficiency*trans_loss*electrical_loss; %watt (every power is in watt)

%-----------Section 3 : vertical Climb from 0m to 60m @ 0.76m/s : ~79 seconds of climb--------------------------------------------------------------
            e_section_3(i,j)=N_rotors*Pclimb(i,j)*60/Vc/motor_efficiency*trans_loss*electrical_loss;

%-----------Section 4 : 10 seconds hover HOGE--------------------------------------------------------------
            e_section_4(i,j)=Power_total_hover(i,j)*10/motor_efficiency*trans_loss*electrical_loss;

%-----------Section 5 : 9deg climb--------------------------------------------------------------
            %Power=D*V_cruise_5 + W*sin(9)*V_cruise_5=L/L_D*V +Wsin(9)*V
            %where L=Wcos(9)
            power_5(i,j)=(GW(n)*cos(degtorad(9))/L_by_D +GW(n)*sin(degtorad(9)))*V_cruise_5/motor_efficiency*trans_loss*electrical_loss;
            time_5(i,j)=(300-60)/V_cruise_5/sin(degtorad(9));
            e_section_5(i,j)=power_5(i,j)*time_5(i,j);


%-----------Section 6 : Cruise @300m (approx 30km)--------------------------------------------------------------
            e_section_6(i,j)=GW(n)*9.8/L_by_D*30000/motor_efficiency*trans_loss*electrical_loss; %Joules
            
%-----------Section 7 : V descent @-7.6m/s from 300m to 30m (NO POWER)--------------------------------------------------------------
            e_section_7(i,j)=0;

%-----------Section 8 : 30 seconds hover @30m--------------------------------------------------------------
            e_section_8(i,j)=Power_total_hover(i,j)*30/motor_efficiency*trans_loss*electrical_loss;

%-----------Section 9 : best endurance loiter @30m  the goal is to maximuise this time--------------------------------------------------------------
%           Basically all the energy that you have left should be utitlised
%           here to hover for max T9 seconds.

            e_section_9(i,j)=GW(n)*9.8/L_by_D*V_endu_cruise*T9/motor_efficiency*trans_loss*electrical_loss; %Joules
            

%-----------Section 10 : 9deg climb from 30m to 300m--------------------------------------------------------------
            e_section_10(i,j)=e_section_5(i,j);


%-----------Section 11 : Cruise back approx 30km--------------------------------------------------------------
            e_section_11(i,j)=GW(n)*9.8/L_by_D*30000/motor_efficiency*trans_loss*electrical_loss; %Joules


%-----------Section 12 : 5 degree descent to 60m--------------------------------------------------------------
            e_section_12(i,j)=0;

%-----------Section 13 : HOGE @60m for 10 seconds--------------------------------------------------------------
            e_section_13(i,j)=Power_total_hover(i,j)*10/motor_efficiency*trans_loss*electrical_loss;


%-----------Section 14 : Vertical descent @-0.5m/s from 60m to HIGE--------------------------------------------------------------
            e_section_14(i,j)= N_rotors*Pdescent(i,j)*60/Vd/motor_efficiency*trans_loss*electrical_loss;

%-----------Section 15 : HIGE for 15 seconds--------------------------------------------------------------
            e_section_15(i,j)= Power_total_hover(i,j)*15/motor_efficiency*trans_loss*electrical_loss;

            energy_init(i,j) = e_section_2(i,j) + e_section_3(i,j) + e_section_4(i,j) + e_section_5(i,j) + ...
              e_section_6(i,j) + e_section_7(i,j) + e_section_8(i,j) + e_section_9(i,j) + ...
              e_section_10(i,j) + e_section_11(i,j) + e_section_12(i,j) + e_section_13(i,j) + ...
              e_section_14(i,j) + e_section_15(i,j);
            energy(i,j)=energy_init(i,j)/3600/batt_reserve; % Watt-hr energy after considering reserve;
            % energy(i,j) =(Power_total_hover(i,j)*(endurance)+Pclimb(i,j)*0.5/60+Pdescent(i,j)*0.5/60)/batt_reserve;% energy supplied  by battery equals total power required for hover multiplied by the hover duration(in watt-hour)
            energy_MJ(i,j) = energy(i,j)*(3600)*10^(-6);%energy in mega -joules(MJ)
            C(i,j) =(((energy(i,j)/(No_of_battery*Nominal_volt))*1000)); %total battery capacity, all 2 batteries combined, in mAh=E(energy)/V(voltage), 3.6 volt for 1 cell;(17% for battery reserve)
            flight_time_min(i,j)= (batt_reserve*No_of_battery*Nominal_volt*(C(i,j))/ Power_total_hover(i,j))*60*10^(-3);%flight time in min
            % C_h(i,j)= ((Power_total(i,j)*(15/60))/(2*S_fr*3.6))*1000;
            % C_ver_climb(i,j)=(Pclimb(i,j)*(climb_alt/(Vc*3.6))/(2*S_fr*3.6))*1000;
            % C_ver_descent(i,j)=(Pdescent(i,j)*(climb_alt/(Vd*3.6))/(2*S_fr*3.6))*1000;
            % C_ff_endu(i,j)= (Pendu(i,j)*(range/Vendu(i,j))/(2*S_fr*3.6))*1000;
            % C_ff_range(i,j)=(Prange(i,j)*(range/Vrange(i,j))/(2*S_fr*3.6))*1000;
            % C_aux=(energy_aux/(2*S_fr*3.6))*1000;
            % C_total(i,j)=(C_h(i,j)+ C_ver_climb(i,j)+ C_ver_descent(i,j)+ C_ff_endu(i,j)+C_ff_range(i,j)+C_aux)/batt_reserve;
            % flight_time_min(i,j)=(batt_reserve)*(No_of_battery)*(Nominal_volt)*(60)*(10^-3)*((C_h(i,j)/Power_total(i,j))+ C_ver_climb(i,j)/Pclimb(i,j) + C_ver_descent(i,j)/Pdescent(i,j) +
            % C_ff_endu(i,j)/Pendu(i,j) + C_ff_range(i,j)/Prange(i,j)+ C_aux/aux_power);



%-----------WEIGHT ESTIMATION------------------------------------------------------------------------------------------
%-----------ROTOR GROUP----------------------------------------------------------------------------------------------%
            mrotor_pounds=(0.02638*(Nb^0.6826)*((c(i,j)*3.28)^0.9952)*((R(i)*3.28)^1.3507)*((Vtip*3.28)^0.6563)*(mu^2.5231))/Nb;% mass of rotor blades(in pounds)
            %mrotor_fr_prouty_pounds=0.026*(Nb_fr^0.66)*(c_fr(i,j)*3.28)*((R_fr(i)*3.28)^1.3)*((Vtip_fr(j)*3.28)^0.67);%prouty blade estimation
            mrotor=mrotor_pounds*0.4535;% mass of rotor in kg
            % mrotor=0.52;
            mhub_pounds=0.0135*(mempty*2.2)*(R(i)*3.28)^0.42;% mass of hub and hinge in pounds
            mhub=mhub_pounds*0.4535;%mass ofhub&hinge in kg
            mhub=0;
            rpm(i,j)=RPM(j);
            Kv(i,j) = rpm(i,j)/(2*S_fr*3.6);% rpm per volt of one side
            mmotor = 0.5;%HK5-4030-355kv or Scorpion IM-8012-115kv
            mmotor= ((10^4.0499)*(Kv(i,j)^-0.5329))*10^-3;
            Imax_fr(i,j)=(Power_total_hover(i,j)/N_rotors)/(2*S_fr*3.6);% maximum current of one side
            mesc_fr =2*0.8421*Imax_fr(i,j)*10^-3;%mass of esc(Thunder 300A 24S)
            m_rotor_group=N_rotors*(mrotor+mhub+mmotor+mesc_fr);%total front mass
%-----------FUSELAGE------------------------------------------------------------------------------------------------%
            nult=2.5; %ultimate load factor from vibhram
            Lf=(L)*3.28+4;% total length of fuselage in ft
            Sf=30;%fuselage wetted area in ft^2(22.38)
            Iramp = 1;% raming factor, 1 for no ramp
            mfuselage_pounds = 10.13*((0.001*GW(n)*2.20)^0.5719)*(nult^0.2238)*(Lf^0.5558)*(Sf^0.1534)*(Iramp^0.5242);% mass of fuselage using RTL method in pounds
            %mfuselage_pounds = 6.9*((GW(n)*2.2/1000)^0.49)*(Lf^0.61)*(Sf^0.25);
            mfuselage=mfuselage_pounds*0.4535*3;% conversion from pound to kg
%-----------TRANSMISSION-------------------------------------------------------------------------------------------------------
            HP_mr=(power_h(i,j)/2)/746;%maximum drive system horse power(1.2 times take off horse power)
            a_mr=1;%adjustment factor
            %rpm(i,j)=Vtip_fr(j)*60/(2*pi*R_fr(i));
            z_mr=1;% number of stages in drive system
            kt=1.3;%configuration factor
            k_star=0.35;%weight coefficient value of CH-47C
            nmgb=1;% number of main gear boxes
            a_q=1;%coefficient reflecting excess torque
            %m_transmission_pounds=250*a_mr*((HP_mr/rpm(i,j))*(z_mr^0.25)*kt)^0.67;%weight of drive system in pounds(boeing vertol)
            %m_transmission=k_star*nmgb*(a_q*torque_h(i,j)/9.8)^0.8;% tishenko estimation of drive system(considering main gear box only)
            m_transmission=0.2*N_rotors;%hard coded data from sabal 10kg m_transmission_pounds*0.4535;%conversion from pound to kg
%-----------LANDING GEAR--------------------------------------------------------------------------------------------
            %mlg = 0.015 * GW(n);%boeing vertol formula
            mlg = 0.010 * GW(n);% tishenko formula
%-----------CONTROLS AND ELECTRICALS-------------------------------------------------------------------------------
            Fcb=2;%1= mechanical type, 2=boosted type
            Fcp=1;%Flight control ballastic tolerance 1=no, 2=yes
            kmrc=26;
            %mcontrols_pounds=0.1657*(Fcb^1.3696)*((c_fr(i,j)*3.28)^0.4481)*(Fcp^0.4469)*((GW(n)*2.20)^0.6865);% weight of controls using RTL method in pounds
            %mcontrols_pounds=36*Nb_fr*((c_fr*3.28)^2.2)*((Vtip_fr*3.28/1000)^3.2);% prouty formula
            mcontrols_pounds=kmrc*((c(i,j)*3.28)*((R(i)*3.28)*Nb*(mrotor*2.2)*10^-3)^0.5)^1.1;% weight of rotor controls plus main actuators(boeing vertol formula)
            %mcontrols_pounds=30*((10^-3)*GW(n)*2.2/2)^0.84;% boeing vertol formula
            %mcontrols_pounds=20*(c_fr(i,j)*3.28*(R_fr(i)*3.28*mrotor_fr_pounds*10^-3)^0.5)^1.1;
            mcontrols=mcontrols_pounds*0.4535;%conversion from pound to kg
            melec=0.02*mempty;% electrical weights
%-----------Tilt Mechanism-----------------------------------------------------------------------------------------------------%
            

%-----------Wings Weight ref:https://core.ac.uk/download/pdf/12983145.pdf-----------------------------------------------------------------------------------------------------%
            % Configuration is tandem wing 
            
            S=1/2*GW(n)*g*2/(Cl_design*V_cruise^2*rho); %wing area for a particular Cl
            b=sqrt(AR_wing*S);
            root_chord=S/(1+taper_ratio)*2/b;

            Ngust = (1 + 6.3 * AR_wing * S * V_cruise * m_to_ft^3) / (GW(n) * kg_to_lb) / (2 + AR_wing)/6;
            Nmanu = max(2.5, 2.1 + 10900 / (4530 + GW(n) * kg_to_lb));
            Ngust_ult = 1.5 * Ngust;
            Nmanu_ult = 1.65 * Nmanu;
            Nult = max(Nmanu_ult, Ngust_ult);
            % Wing Weight (converted to metric)
            m_wing = 0.6*(4.22 * S * m_to_ft^2 + 1.642 * (10^-6) * Nult * (b * m_to_ft)^3 * (1 + 2 * taper_ratio) * ...
                      (GW(n) * (mempty) * kg_to_lb^2)^0.5 / (tc_ratio * S * m_to_ft^2 * (1 + taper_ratio))) / kg_to_lb*.8;
            
            % Fuselage Weight (converted to metric)
            % W_fuselage = 0.0737 * (2 * (D_fuselage * m_to_ft) * (v_cruise * m_to_ft)^0.338 * (L_fuselage * m_to_ft)^0.857 * ...
            %               (GW(n) * kg_to_lb * Nult)^0.286)^1.1 / kg_to_lb;
%-----------HYDROGEN FUEL CELL-----------------------------------------------------------------------------------------------------%
            sed = 300*3600*1e-6; %MJ/Kg(specific energy density of an efficient Hydrogen cell)
            m_hydrogen =energy_MJ(i,j) / sed;%kg
            mfuel_system=30;%Power_total_hover(i,j)/0.15/1000;
            mfuel_cell(i,j)= mfuel_system+ m_hydrogen;
%-----------FIXED--------------------------------------------------------------------------------------------------
            m_avionics=0.05*mempty;% mass of avionics
            manti_ice=0;%8*(GW(n)/1000);%mass of anti ice equipments
            m_instruments=0.4535*3.5*(GW(n)*2.2/1000)^1.3;%mass of instruments
            mfixed=m_avionics+manti_ice+m_instruments;% includes mass of avionics and mass of ribs+rods+payload support, anti ice and equipments
%-----------NEW EMPTY WEIGHT---------------------------------------------------------------------------------------
            mempty = m_rotor_group + mfuselage + mcontrols + melec + mlg + mfixed + m_transmission+ m_wing ;
%-----------NEW GROSS WEIGHT----------------------------------------------------------------------------------------
            mgross(i,j) = mempty+mfuel_cell(i,j)+mpayload ;%new gross weight
            m=m+1;
            n=n+1;
            GW(n) = mgross(i,j);
            fprintf("current total mass :%4.3f \n",GW(n));
            error= abs(GW(n)-GW(m));
            % if(error<=0.01)
            %     power_final(i,j)=power_1(count); 
            %     thrust_final(i,j)=
        end
        if (solution_check==1)
            V1(i,j) = R(i);% radius
            V2(i,j) = Vtip;%tip speed
            V3(i,j) = thrust_total(i,j);%total thrust
            V4(i,j) = err(i,j);%error margin
            V5(i,j) = torque_h(i,j);% total torque
            V6(i,j) = rpm(i,j);%rpm of blade
            V7(i,j)= Vtip/sqrt(1.4*287*T);%mach number
            O1(i,j) = Power_total_hover(i,j);%total hover power
            O2(i,j) = mgross(i,j);%gross weight
            O3(i,j) = energy(i,j);%total energy
            O4(i,j) = mfuel_cell(i,j);%total battery mass
            O5(i,j) = theta_h(i,j)*180/pi;%collective in degrees
            O6(i,j) = thrust_total(i,j)./power_h(i,j);%power loading(N/W)
            O7(i,j) = (thrust_total(i,j)/N_rotors)./(pi.*V1(i,j).^2);%disk loading(N/m^2)
            O8(i,j) = power_mech(i,j);
        end
        
        save=false;
            if save==true
                       % Create a table with the variables
            data = table({'Number of Blades', Nb;'NumberOfRotors', N_rotors;  'Radius', V1(i,j); 'TipSpeed', V2(i,j); 
              'TotalThrust', V3(i,j); 'Extra thrust at hover', V4(i,j); 
              'TotalTorque_per_motor Nm', V5(i,j); 'RPM', V6(i,j); 'MachNumber', V7(i,j); 
              'TotalHoverPower Watt', O1(i,j); 'GrossWeight', O2(i,j); 
              'TotalEnergy', O3(i,j); 'TotalBatteryMass', O4(i,j); 
              'CollectiveInDegrees', O5(i,j); 'PowerLoading', O6(i,j); 
              'DiskLoading', O7(i,j); 'MechanicalPower', O8(i,j); 
              'BatteryMass', mfuel_cell; 'EmptyMass', mempty; 
              'FuselageMass', mfuselage;'ClimbSpeed', Vc; 'DescentSpeed', Vd;
              'CruiseVelocity', V_cruise; 'CruiseVelocity_Climb', V_cruise_5; 
              'AspectRatio', AR; 'TwistRate', thetatw; 'Max endurance Velocity', Vendu;'T9 time to be maximised for loiter',T9});
            % Prompt user for confirmation before saving
            
            prompt = 'Do you want to save the data to a file? Y/N: ';
            str = input(prompt, 's');
            
            
            if strcmpi(str, 'Y')
                % Ask for file name
                [file, path] = uiputfile('*.csv', 'Save data as');
                if ischar(file)
                    % Save the table to a CSV file
                    writetable(data, fullfile(path, file));
                    fprintf('Data saved to %s\n', fullfile(path, file));
                else
                    fprintf('File save canceled.\n');
                end
            else
                fprintf('Data not saved.\n');
            end
            end


    end
end

%% ------------------------------------Design Trade study--------------------
% set(groot,'defaultLineLineWidth',3);
% set(groot,'defaultAxesFontSize',16);
% surf(V1,V2,O7,O6);
% hold on;
% xlabel("Blade radius(m)");
% ylabel("Tip speed in(m/s)");
% zlabel("Disk loading(N/m^2)");
% %shading interp;
% grid on;
% cb=colorbar;
% cb.Label.String="Power Loading(N/W)";
% hold on;
% plot(V6,O1);
% xlabel("rpm");
% ylabel("Power(W)");
% hold on;
% for z=1:size(R_fr,2)
% plot(V3(z,:),V4(z,:));
% hold on;
% end
% Create meshgrid
% [R_fr_grid, RPM_grid] = meshgrid(R_fr, RPM);
% 
% % Plot the first graph (Total Power)
% figure;
% surf(R_fr_grid, RPM_grid, O1');
% title('Total Power');
% xlabel('R_fr');
% ylabel('RPM');
% zlabel('Total Power');
% grid on;
% 
% % Plot the second graph (Gross Weight)
% figure;
% surf(R_fr_grid, RPM_grid, O2');
% title('Gross Weight');
% xlabel('R_fr');
% ylabel('RPM');
% zlabel('Gross Weight');
% grid on;
% 
% % Plot the third graph (Torque Required)
% figure;
% surf(R_fr_grid, RPM_grid, V5');
% title('Torque Required');
% xlabel('R_fr');
% ylabel('RPM');
% zlabel('Torque Required');
% grid on;
% 
% % Plot the fourth graph (Power Loading)
% figure;
% surf(R_fr_grid, RPM_grid, O5');
% title('Power Loading');
% xlabel('R_fr');
% ylabel('RPM');
% zlabel('Power Loading');
% grid on;
% 
% % Plot the fifth graph (Disc Loading)
% figure;
% surf(R_fr_grid, RPM_grid, O6');
% title('Disc Loading');
% xlabel('R_fr');
% ylabel('RPM');
% zlabel('Disc Loading');
% grid on;
%% Plots

% % Create mesh grids for R and RPM
% [R_mesh, RPM_mesh] = meshgrid(R, RPM);
% 
% % Plot 1: Gross Weight (mgross)
% figure;
% surf(R_mesh, RPM_mesh, mgross');
% xlabel('Rotor Radius (m)');
% ylabel('Rotor RPM');
% zlabel('Gross Weight (kg)');
% title('Gross Weight vs. Rotor Radius and RPM');
% colorbar;
% shading interp;
% grid on;
% 
% [R_mesh1, RPM_mesh1] = meshgrid(R, RPM);
% 
% % Plot 2: Battery Mass (mbattery)
% figure;
% surf(R_mesh1, RPM_mesh1, mfuel_cell');
% xlabel('Rotor Radius (m)');
% ylabel('Rotor RPM');
% zlabel('Battery Mass (kg)');
% title('Battery Mass vs. Rotor Radius and RPM');
% colorbar;
% shading interp;
% grid on;
% 
% % Plot 3: Hover Power (Power_total_hover)
% [R_mesh2, RPM_mesh2] = meshgrid(R, RPM);
% 
% figure;
% surf(R_mesh2, RPM_mesh2, Power_total_hover');
% xlabel('Rotor Radius (m)');
% ylabel('Rotor RPM');
% zlabel('Total Hover Power (W)');
% title('Hover Power vs. Rotor Radius and RPM');
% colorbar;
% shading interp;
% grid on;
% % torque
% [R_mesh3, RPM_mesh3] = meshgrid(R, RPM);
% 
% figure;
% surf(R_mesh3, RPM_mesh3, V5');
% xlabel('Rotor Radius (m)');
% ylabel('Rotor RPM');
% zlabel('Torque required by Motor (Nm)');
% title('Torque vs. Rotor Radius and RPM');
% colorbar;
% shading interp;
% grid on;
% % theta0
% [R_mesh4, RPM_mesh4] = meshgrid(R, RPM);
% 
% figure;
% surf(R_mesh4, RPM_mesh4, O5');
% xlabel('Rotor Radius (m)');
% ylabel('Rotor RPM');
% zlabel('Collective (degrees)');
% title('Collective vs. Rotor Radius and RPM');
% colorbar;
% shading interp;
% grid on;
% 
%% disk loading
% [R_mesh5, RPM_mesh5] = meshgrid(R, RPM);
% 
% figure;
% surf(R_mesh5, RPM_mesh5, O7');
% xlabel('Rotor Radius (m)');
% ylabel('Rotor RPM');
% zlabel('Disk Loading (Kg/m^2)');
% title('Disk Loading vs. Rotor Radius and RPM');
% colorbar;
% shading interp;
% grid on;
