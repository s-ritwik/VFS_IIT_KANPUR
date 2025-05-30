clc;    
clear;
clear all;
%% --------------------------------Mission requirements--------------------------------------------------------------%
h=300; %m
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
%% --------------------------------Baseline Parameters: Indipendant variables---------------------------------------------------------------%
R=1;%(in meters) % FIXEDDDD__________________
RPM= 3000%:250:3500;
V_cruise=55%40:10:80;
% --------------section 9 where you have to maximise T9---------------------------
T9= 60%:20:120; %1 hr
T9=T9.*60; %into seconds
thetatw = -20:1:-18;% twist rate

%% ------------------------------- UAV Parameter---------------------------------------------------------------------
Nb = 3;%no of blades 
N_rotors=8; %no of motors/rotors
N_rotors_cruise=2; %no of motors/rotors active in cruise fixed wing mode
Vc=0.76;%climb speed in m/s
Vd=0.5;% descent speed in m/s
V_cruise_climb=6; %section 5 climb vel 
%% -------------- rotor parameters ----------------------------------------
AR = 12;% aspect ratio of rotor
theta0  =(10:0.01:50)*pi/180;% collective
% thetatw = -18;% twist rate
S_battery =12; %no of cells in series in a battery(44.4v/3.7v)
%% ----------------------wing-----------------------------------
wing_AR=8;% aspect ratio of wing
wing_taper_ratio=1; 
tc_ratio = 0.25; % Thickness to chord ratio
wing_Cl_design=0.5;% Cl_design
%% --------------------------------Losses & Efficiencies-------------------------------------------------------------%
trans_loss=1.03;%transmission losses(2%)
electrical_loss=1.02;%electrical losses(2%)
motor_efficiency=0.85;%motor efficiency(85%)
mu=1.09;
%% --------------------------------Fixed weights----------------------------------------------------------------------
%mmotor=1.74*4;%we are using 4 motors(Tmotor U15 2 KV80),total power=34.3 KW
%mesc=0.558*4;%we are using 4 esc,(FLAME 200A 14S) 1 for each motor
%% --------------------------------Constants----------------------------------------------------------------------
kg_to_lb = 2.20462; % kg to pounds
m_to_ft = 3.28084; % meters to feet
g=9.81;
save_variable=false;
%% ---------------------------------------Design code---------------------------------------------------------
counter = 0; % keep outside loop
for k=1:size(V_cruise,2)
    for i=1:size(T9,2)
        
        for j=1:size(RPM,2)
            for l=1:size(thetatw,2)
                V_endu_cruise=V_cruise(k)*0.85;
                wing_S(i,j,k,l)=2;
                solution_check=1;
                Vtip=(RPM(j)).*(2*pi.*R/60);
                fprintf('T9:%4.3f mins, RPM- %4.3f twist:%4.3f vtip:%4.3f V_cruise:%4.3f\n',T9(i)/60, RPM(j), thetatw(l), Vtip,V_cruise(k));
        %-------ROTOR PARAMETERS--------------------------------------------------------------------------------------------------%
                c(i,j,k,l) =R/AR;%chord
                omega(i,j,k,l) = Vtip/R;%angular velocity
                solidity(i,j,k,l) =(Nb*c(i,j,k,l)*(R-0.2*R))/(pi*R.^2);%solidity     
        %-------DISTANCE BETWEEN ROTORS------------------------------------------------------------------------------------
                L = R+R+(2*R/3);%center to center distance,clearance taken as diameter/3
        %-------WEIGHT INITIALIZATION-----------------------------------------------------------------------------------------%
                m=1;%first index initialization
                n=2;% second index initialization
                GW(m) =600;%(KG) 
                mempty =400;%empty weight(kg)
                GW(n) = mempty+mpayload;%total gross weight
                pre_weight=GW(n);%intial gross weight assumption
                A=pi*(R^2);%area of disk(for both front and back it will be same)
                nondt=rho*A*(omega(i,j,k,l)*R)^2;%non-dimentional making terms for thrust
                nondp=rho*A*(omega(i,j,k,l)*R)^3;%non-dimentional making terms for pressure
                error=100;%arbitrary value of error for initialization
                batt_reserve=0.85;% 15% battery reserve
                No_of_battery=2;% no of batteries each side
                % fprintf(".");
                Nominal_volt=3.6*S_battery; %Nominal voltage of the entire one battery(cells in series)
                gibrish=0; % to terminate loop when negative power arises
                while( error>0.2 & n<25 & gibrish <1)
                    % gibrish
                    count_k=0;
        %-----------Thrust and Power calculation using momentum theory(hover)-----------------------------------------------------
                    Ct=GW(n)*9.81/nondt;%considering thrust equal to weight for hover
                    Cpi=1.15*(Ct^1.5)/sqrt(2);%induced power coefficient
                    Cpp=solidity(i,j,k,l)*Cd/8;%profile power coefficient
                    collective(i,j,k,l)=((6*Ct/(solidity(i,j,k,l)*cl_alpha)) + 1.5*sqrt(Ct/2))*180/pi;%theta in degrees
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
                        % Call the Rotor_opt_sampurak function
        
                        %____________________________ROTOR OPT__________
                        try
        
                        [thrust_1, power_1, torque_1, theta_1, err_1, FM, BL, mech_power,CP,CT] = ...
                            Rotor_opt_sampurak(R, c(i,j,k,l), thetatw(l), RPM(j), Nb, 3, GW(n), trans_loss, nondp, motor_efficiency, nondt, theta0(k_mid), electrical_loss, rho,N_rotors);
                            %2 is for airfoil
                        
                        %------------BEMT------------------------------------
                         % [thrust_1,power_1,torque_1,theta_1,err_1,FM,BL,mech_power]=BEMT(R_fr(i),Ct,Nb_fr,c_fr(i,j,k,l),Vtip_fr(j),a,GW(n),trans_loss,nondp,motor_efficiency,nondt,theta0(k_mid),electrical_loss);
                        catch
                            continue
                        end
        
                        % if power_1 <0
                        %     fprintf("skipping iteration")
                        %     continue;
                        % end
                        % Check the error condition
                        if err_1 > 5 
                            % Condition is satisfied, so move left to find a smaller k
                            k_max = k_mid - 1;
                            thrust_h(i,j,k,l) = thrust_1;
                            power_h(i,j,k,l) = power_1;
                            torque_h(i,j,k,l) = torque_1;
                            C_P(i,j,k,l) = CP;
                            C_T(i,j,k,l)= CT;
                            theta_h(i,j,k,l) = theta0(k_mid);
                            err(i,j,k,l) = err_1;
                            power_mech(i,j,k,l) = mech_power;
                            count_k = k_mid;
                            found = true;
                        else
                            % Condition is not satisfied, so move right to find a larger k
                            k_min = k_mid + 1;
                            % thrust_h(i,j,k,l) = 0;
                            % power_h(i,j,k,l) = 0;
                            % torque_h(i,j,k,l) = 0;
                            % theta_h(i,j,k,l) = 0;
                            % Profile_power(i,j,k,l) = 0;
                            % Induced_power(i,j,k,l) = 0;
                        end
                    end
        %% -----------------------------Cruise performance of the rotor--------------------------------
                    if(power_h(i,j,k,l)<0)
                        gibrish=1;
                        k_max = k_mid - 1;
                        thrust_h(i,j,k,l) = 0;
                        power_h(i,j,k,l) = 0;
                        torque_h(i,j,k,l) = 0;
                        C_P(i,j,k,l) = 0;
                        C_T(i,j,k,l)= 0;
                        theta_h(i,j,k,l) = theta0(k_mid);
                        err(i,j,k,l) = 0;
                        power_mech(i,j,k,l) = 0;
                        count_k = k_mid;
                        found = false;
                        GW(n)=0;
                        Power_total_hover(i,j,k,l)=0;
                        power_cruise(i,j,k,l)=0;
                    end
                    k_min2 = 1000;
                    k_max2 = RPM(j);
                    found2 = false;
                    while k_min2 <= k_max2
        
                        k_mid2 = floor((k_min2 + k_max2) / 2);
                        omega2(i,j,k,l)=k_mid2*2*pi/60;
                        nondt2=rho*A*(omega2(i,j,k,l)*R)^2;%non-dimentional making terms for thrust
                        nondp2=rho*A*(omega2(i,j,k,l)*R)^3;%non-dimentional making terms for pressure
                        % for cruise fixed wing
                        [thrust_2, power_2, torque_2, theta_2, err_2, FM2, BL2, mech_power2,CP2,CT2] = ...
                            Rotor_opt_sampurak(R, c(i,j,k,l), thetatw(l), k_mid2, Nb, 3, GW(n)/L_by_D/N_rotors_cruise, trans_loss, nondp2, motor_efficiency, nondt2, theta_h(i,j,k,l), electrical_loss, rho,1);
                        if err_2 > 5
                            k_max2 = k_mid2 - 1;
                            err2(i,j,k,l)= err_2;
                            Cruise_RPM(i,j,k,l)=k_mid2;
                            power_cruise_hover(i,j,k,l)=power_2;% 0.85 is aerodynamic efficiency
                        else
                            k_min2 = k_mid2 + 1;
        
                        end
                        % fprintf("1");
                    end
        %% ----------- 9 degree climb performance--------------------
                    % k_min3 = 1000;
                    % k_max3 = RPM(j);
                    % found3 = false;
                    % thrust_section_5(i,j,k,l)= GW(n)*g*(cos(degtorad(9))/L_by_D+ sin(degtorad(9)));
                    % 
                    % while k_min3 <= k_max3
                    % 
                    %     k_mid3 = floor((k_min3 + k_max3) / 2);
                    %     omega3(i,j,k,l)=k_mid3*2*pi/60;
                    %     nondt3=rho*A*(omega3(i,j,k,l)*R)^2;%non-dimentional making terms for thrust
                    %     nondp3=rho*A*(omega3(i,j,k,l)*R)^3;%non-dimentional making terms for pressure
                    %     % for cruise fixed wing
                    %     [thrust_3, power_3, torque_3, theta_3, err_3, FM3, BL3, mech_power3,CP3,CT3] = ...
                    %         Rotor_opt_sampurak(R, c(i,j,k,l), thetatw(l), k_mid3, Nb, 3, thrust_section_5(i,j,k,l)/g/N_rotors_cruise, trans_loss, nondp3, motor_efficiency, nondt3, theta_h(i,j,k,l), electrical_loss, rho,1);
                    %     if err_3 > 5
                    %         k_max3 = k_mid3 - 1;
                    %         err3(i,j,k,l)= err_3;
                    %         Cruise_Climb_RPM(i,j,k,l)=k_mid3;
                    %         power_cruise_climb(i,j,k,l)=power_3;% 0.85 is aerodynamic efficiency
                    %     else
                    %         k_min3 = k_mid3 + 1;
                    % 
                    %     end
                    %     % fprintf("1");
                    % end
                    % ------------------------------------------------------------------------------------
                    % fprintf("\n");
                    if ~found
                        fprintf("solution not possible");
                        solution_check = 0;
                        break;
                    else
                        % fprintf("done");
                    end
                    % fprintf("Into power");
        %-----------Auxillary power----------------------------------------------------------------------------------------
                    Power_servo=100;% assumption
                    Power_camera=25;%25W
                    aux_power=Power_servo+Power_camera;
                    aux_power=5000; %assumpption 
        %-----------Total power and thrust in hover--------------------------------------------------------------------------- 
                    Power_total_hover(i,j,k,l)=power_h(i,j,k,l)+aux_power;
                    thrust_total(i,j,k,l)=thrust_h(i,j,k,l);
                    Pclimb(i,j,k,l)=0;Pdescent(i,j,k,l)=0;
        
                    %----Climb power------------------------for vc and vd
                    try
                    [Pclimb(i,j,k,l),Pdescent(i,j,k,l)]=climb(thrust_h(i,j,k,l)/N_rotors,rho,R,Power_total_hover(i,j,k,l)/N_rotors,Vc,Vd);
                    catch
                        Pclimb(i,j,k,l)=0;
                        Pdescent(i,j,k,l)=0;
                        fprintf(" Vertical climb failed");
        
                        continue;  % Skip to next (j) iteration
                        
                    end
                    %------------calculation using MT(forward-flight)---------------------   
                    % try
                    %     [Prange(i,j,k,l), Vrange(i,j,k,l), Pendu(i,j,k,l), Vendu(i,j,k,l)] = forwardflight(R, Vtip, thrust_h(i,j,k,l)/N_rotors, nondp, solidity(i,j,k,l), h);
                    % catch
                    %     Prange(i,j,k,l) = 0;
                    %     Vrange(i,j,k,l) = 0;
                    %     Pendu(i,j,k,l) = 0;
                    %     Vendu(i,j,k,l) = 0;
                    %     fprintf(" Forwar");
                    %     continue;  % Skip to next (j) iteration
                    % 
                    % end
                    %----Cruise power------------------------for vc and vd
                    try
                    [Pcruise(i,j,k,l),P_rand(i,j,k,l)]=climb(thrust_h(i,j,k,l)/N_rotors_cruise/L_by_D,rho,R,power_cruise_hover(i,j,k,l),V_cruise(k),Vd);
                    catch
                        Pcruise(i,j,k,l)=0;
                        P_rand(i,j,k,l)=0;
                        fprintf(" cruise flight BEMT failed");
                        continue;  % Skip to next (j) iteration
        
                    end
                    % print("1");
                    Thrust_cruise(i,j,k,l)=GW(n)/L_by_D*g;
                    % Pcruise(i,j,k,l)= 0.5*Thrust_cruise(i,j,k,l)/N_rotors_cruise*V_cruise(k)*(1+sqrt(2*Thrust_cruise(i,j,k,l)/(rho*A*V_cruise(k)^2)));
                    power_cruise(i,j,k,l)= Pcruise(i,j,k,l)*N_rotors_cruise;
        %-----------Total energy and battery capacity required based on mission
        
        %-----------Section 2 : 15 seconds hover HIGE (i havent considered HIG yet)--------------------------------------------------------------
                    e_section_2(i,j,k,l)= Power_total_hover(i,j,k,l)*15/1.5; %watt (every power is in watt)
                    time_section_2(i,j,k,l)= 15;
        %-----------Section 3 : vertical Climb from 0m to 60m @ 0.76m/s : ~79 seconds of climb--------------------------------------------------------------
                    p_section_3(i,j,k,l)= N_rotors*Pclimb(i,j,k,l);
                    e_section_3(i,j,k,l)=N_rotors*Pclimb(i,j,k,l)*60/Vc/motor_efficiency;
                    time_section_3(i,j,k,l)=60/Vc;
        %-----------Section 4 : 10 seconds hover HOGE--------------------------------------------------------------
                    e_section_4(i,j,k,l)=Power_total_hover(i,j,k,l)*10;
                    time_section_4(i,j,k,l)=10;
        %-----------Section 5 : 9deg climb--------------------------------------------------------------
                    %Power=D*V_cruise_5 + W*sin(9)*V_cruise_5=L/L_D*V +Wsin(9)*V
                    %where L=Wcos(9)
                    try
                        power_5(i,j,k,l)=power_cruise(i,j,k,l)+ V_cruise_climb*GW(n)*g; % by using excess power formula
                        % power_5(i,j,k,l)= power_5(i,j,k,l)*N_rotors_cruise;
                    catch
                        fprintf(" Climb performance failed")
                        power_5(i,j,k,l)=N_rotors_cruise* 0.5*thrust_section_5(i,j,k,l)/N_rotors_cruise*V_cruise_climb*(1+sqrt(2*thrust_section_5(i,j,k,l)/N_rotors_cruise/(rho*A*V_cruise_climb^2)));
                    end
                    time_section_5(i,j,k,l)=(300-60)/V_cruise_climb;
                    e_section_5(i,j,k,l)= power_5(i,j,k,l)*time_section_5(i,j,k,l);
        
        
        %-----------Section 6 : Cruise @300m (approx 30km)--------------------------------------------------------------
                    time_section_6(i,j,k,l)= 30000/V_cruise(k);
                    e_section_6(i,j,k,l)= power_cruise(i,j,k,l)*time_section_6(i,j,k,l);
                    
        %-----------Section 7 : V descent @-7.6m/s from 300m to 30m (NO POWER)--------------------------------------------------------------
                    e_section_7(i,j,k,l)=0;
                    time_section_7(i,j,k,l)=(300-30)/7.6;
        %-----------Section 8 : 30 seconds hover @30m--------------------------------------------------------------
                    e_section_8(i,j,k,l)=Power_total_hover(i,j,k,l)*30;
                    time_section_8(i,j,k,l)=30;
        %-----------Section 9 : best endurance loiter @30m  the goal is to maximuise this time--------------------------------------------------------------
        %           Basically all the energy that you have left should be utitlised
        %           here to hover for max T9 seconds.
                    P_cruise_endurance(i,j,k,l)=climb(thrust_h(i,j,k,l)/N_rotors_cruise/L_by_D,rho,R,power_cruise_hover(i,j,k,l),V_endu_cruise,Vd);
                    eta = thrust_h(i,j,k,l)*V_endu_cruise/(P_cruise_endurance(i,j,k,l)*L_by_D);
                    % P_cruise_endurance(i,j,k,l)= Thrust_cruise(i,j,k,l)/N_rotors_cruise*V_endu_cruise*(1+sqrt(2*Thrust_cruise(i,j,k,l)/(rho*A*V_endu_cruise^2)));
                    power_endu(i,j,k,l)= P_cruise_endurance(i,j,k,l)*N_rotors_cruise;
                    e_section_9(i,j,k,l)=power_endu(i,j,k,l)*T9(i); %Joules
                    Cl_req(i,j,k,l)= GW(n)/2*g/(0.5*rho*V_endu_cruise^2*wing_S(i,j,k,l));
        
        %-----------Section 10 : 9deg climb from 30m to 300m--------------------------------------------------------------
                    e_section_10(i,j,k,l)=e_section_5(i,j,k,l);
                    time_section_10(i,j,k,l)= time_section_5(i,j,k,l);
                    
        %-----------Section 11 : Cruise back approx 30km--------------------------------------------------------------
                    e_section_11(i,j,k,l)=e_section_6(i,j,k,l); %Joules
                    time_section_11(i,j,k,l)=time_section_6(i,j,k,l);
        
        %-----------Section 12 : 5 degree descent to 60m--------------------------------------------------------------
                    e_section_12(i,j,k,l)=0;
                    time_section_12(i,j,k,l)=(300-60)/7.6;
        %-----------Section 13 : HOGE @60m for 10 seconds--------------------------------------------------------------
                    e_section_13(i,j,k,l)=Power_total_hover(i,j,k,l)*10;
                    time_section_13(i,j,k,l)= 10;
        
        %-----------Section 14 : Vertical descent @-0.5m/s from 60m to HIGE--------------------------------------------------------------
                    e_section_14(i,j,k,l)= N_rotors*Pdescent(i,j,k,l)*60/Vd/motor_efficiency;
                    time_section_14(i,j,k,l)=60/Vd;
        %-----------Section 15 : HIGE for 15 seconds--------------------------------------------------------------
                    e_section_15(i,j,k,l)= Power_total_hover(i,j,k,l)*15*0.7;
                    time_section_15(i,j,k,l)=15;
        %% -------------------- ENERGY CALCULATIONS------------------------------------------------------------------
                    % Total Energy
                    total_energy(i,j,k,l) = e_section_2(i,j,k,l) + e_section_3(i,j,k,l) + e_section_4(i,j,k,l) + e_section_5(i,j,k,l) + e_section_6(i,j,k,l) + e_section_7(i,j,k,l) + e_section_8(i,j,k,l) +...
                     e_section_9(i,j,k,l) + e_section_10(i,j,k,l) + e_section_11(i,j,k,l) + e_section_12(i,j,k,l) + e_section_13(i,j,k,l) + e_section_14(i,j,k,l) + e_section_15(i,j,k,l);   
        
                    % Endurance
                    endurance(i,j,k,l)=time_section_2(i,j,k,l)+time_section_3(i,j,k,l)+time_section_4(i,j,k,l)+time_section_5(i,j,k,l)+time_section_6(i,j,k,l)+...
                        time_section_7(i,j,k,l)+time_section_8(i,j,k,l)+time_section_10(i,j,k,l)+time_section_11(i,j,k,l)+time_section_12(i,j,k,l)+time_section_13(i,j,k,l)+...
                        time_section_14(i,j,k,l)+time_section_15(i,j,k,l)+T9(i);
        
                    % Battery powered ( upto 30kg)
                    energy_init_battery(i,j,k,l)= e_section_3(i,j,k,l) + e_section_4(i,j,k,l)+e_section_8(i,j,k,l)+e_section_13(i,j,k,l) + e_section_14(i,j,k,l) + e_section_15(i,j,k,l);
                    % Hydrogen powered
                    energy_init(i,j,k,l)=e_section_2(i,j,k,l) + e_section_5(i,j,k,l) + ...
                      e_section_6(i,j,k,l) + e_section_7(i,j,k,l) +  e_section_9(i,j,k,l) + ...
                      e_section_10(i,j,k,l) + e_section_11(i,j,k,l) + e_section_12(i,j,k,l) ;
                      
                    energy_hydrogen(i,j,k,l)=energy_init(i,j,k,l)/3600/batt_reserve; % Watt-hr energy after considering reserve;
                    energy_battery(i,j,k,l)=energy_init_battery(i,j,k,l)/3600/batt_reserve;
                    energy(i,j,k,l) =(Power_total_hover(i,j,k,l)*(endurance(i,j,k,l))+Pclimb(i,j,k,l)*0.5/60+Pdescent(i,j,k,l)*0.5/60)/batt_reserve;% energy supplied  by battery equals total power required for hover multiplied by the hover duration(in watt-hour)
                    energy_MJ_hydrogen(i,j,k,l) = energy_hydrogen(i,j,k,l)*(3600)*10^(-6);%energy in mega -joules(MJ)
        
        
        %-----------WEIGHT ESTIMATION------------------------------------------------------------------------------------------
        %-----------ROTOR GROUP----------------------------------------------------------------------------------------------%
                    mrotor_pounds=(0.02638*(Nb^0.6826)*((c(i,j,k,l)*3.28)^0.9952)*((R*3.28)^1.3507)*((Vtip*3.28)^0.6563)*(mu^2.5231))/Nb;% mass of rotor blades(in pounds)
                    %mrotor_fr_prouty_pounds=0.026*(Nb_fr^0.66)*(c_fr(i,j,k,l)*3.28)*((R_fr(i)*3.28)^1.3)*((Vtip_fr(j)*3.28)^0.67);%prouty blade estimation
                    mrotor=mrotor_pounds*0.4535;% mass of rotor in kg
                    % mrotor=0.52;
                    mhub_pounds=0.0135*(mempty*2.2)*(R*3.28)^0.42;% mass of hub and hinge in pounds
                    mhub=mhub_pounds*0.4535;%mass ofhub&hinge in kg
                    mhub=0;
                    rpm(i,j,k,l)=RPM(j);
                    Kv(i,j,k,l) = rpm(i,j,k,l)/(2*S_battery*3.6);% rpm per volt of one side
                    mmotor = 0.5;%HK5-4030-355kv or Scorpion IM-8012-115kv
                    mmotor= ((10^4.0499)*(Kv(i,j,k,l)^-0.5329))*10^-3;
                    Imax_fr(i,j,k,l)=(Power_total_hover(i,j,k,l)/N_rotors)/(2*S_battery*3.6);% maximum current of one side
                    mesc_fr =2*0.8421*Imax_fr(i,j,k,l)*10^-3;%mass of esc(Thunder 300A 24S)
                    m_rotor_group=N_rotors*(mrotor+mhub+mmotor+mesc_fr);%total front mass
        %-----------FUSELAGE------------------------------------------------------------------------------------------------%
                    nult=2.5; %ultimate load factor
                    Lf=(L)*3.28+4;% total length of fuselage in ft
                    Sf=30;%fuselage wetted area in ft^2(22.38)
                    Iramp = 1;% raming factor, 1 for no ramp
                    mfuselage_pounds = 10.13*((0.001*GW(n)*2.20)^0.5719)*(nult^0.2238)*(Lf^0.5558)*(Sf^0.1534)*(Iramp^0.5242);% mass of fuselage using RTL method in pounds
                    %mfuselage_pounds = 6.9*((GW(n)*2.2/1000)^0.49)*(Lf^0.61)*(Sf^0.25);
                    mfuselage=mfuselage_pounds*0.4535*3;% conversion from pound to kg
        %-----------TRANSMISSION-------------------------------------------------------------------------------------------------------
                    HP_mr=(power_h(i,j,k,l)/2)/746;%maximum drive system horse power(1.2 times take off horse power)
                    a_mr=1;%adjustment factor
                    %rpm(i,j,k,l)=Vtip_fr(j)*60/(2*pi*R_fr(i));
                    z_mr=1;% number of stages in drive system
                    kt=1.3;%configuration factor
                    k_star=0.35;%weight coefficient value of CH-47C
                    nmgb=1;% number of main gear boxes
                    a_q=1;%coefficient reflecting excess torque
                    %m_transmission_pounds=250*a_mr*((HP_mr/rpm(i,j,k,l))*(z_mr^0.25)*kt)^0.67;%weight of drive system in pounds(boeing vertol)
                    %m_transmission=k_star*nmgb*(a_q*torque_h(i,j,k,l)/9.8)^0.8;% tishenko estimation of drive system(considering main gear box only)
                    m_transmission=0.2*N_rotors;%hard coded data from sabal 10kg m_transmission_pounds*0.4535;%conversion from pound to kg
        %-----------LANDING GEAR--------------------------------------------------------------------------------------------
                    %mlg = 0.015 * GW(n);%boeing vertol formula
                    mlg = 0.010 * GW(n);% tishenko formula
        %-----------CONTROLS AND ELECTRICALS-------------------------------------------------------------------------------
                    Fcb=2;%1= mechanical type, 2=boosted type
                    Fcp=1;%Flight control ballastic tolerance 1=no, 2=yes
                    kmrc=26;
                    %mcontrols_pounds=0.1657*(Fcb^1.3696)*((c_fr(i,j,k,l)*3.28)^0.4481)*(Fcp^0.4469)*((GW(n)*2.20)^0.6865);% weight of controls using RTL method in pounds
                    %mcontrols_pounds=36*Nb_fr*((c_fr*3.28)^2.2)*((Vtip_fr*3.28/1000)^3.2);% prouty formula
                    mcontrols_pounds=kmrc*((c(i,j,k,l)*3.28)*((R*3.28)*Nb*(mrotor*2.2)*10^-3)^0.5)^1.1;% weight of rotor controls plus main actuators(boeing vertol formula)
                    %mcontrols_pounds=30*((10^-3)*GW(n)*2.2/2)^0.84;% boeing vertol formula
                    %mcontrols_pounds=20*(c_fr(i,j,k,l)*3.28*(R_fr(i)*3.28*mrotor_fr_pounds*10^-3)^0.5)^1.1;
                    mcontrols=mcontrols_pounds*0.4535;%conversion from pound to kg
                    melec=0.02*mempty;% electrical weights
        %-----------Tilt Mechanism-----------------------------------------------------------------------------------------------------%
                    
        
        %-----------Wings Weight ref:https://core.ac.uk/download/pdf/12983145.pdf-----------------------------------------------------------------------------------------------------%
                    % Configuration is tandem wing 
                    
                    wing_S(i,j,k,l)=1/2*GW(n)*g*2/(wing_Cl_design*V_cruise(k)^2*rho); %wing area for a particular Cl
                    wing_b(i,j,k,l)=sqrt(wing_AR*wing_S(i,j,k,l));
                    wing_root_chord=wing_S(i,j,k,l)/(1+wing_taper_ratio)*2/wing_b(i,j,k,l);
        
                    Ngust = (1 + 6.3 * wing_AR * wing_S(i,j,k,l) * V_cruise(k) * m_to_ft^3) / (GW(n) * kg_to_lb) / (2 + wing_AR)/6;
                    Nmanu = max(2.5, 2.1 + 10900 / (4530 + GW(n) * kg_to_lb));
                    Ngust_ult = 1.5 * Ngust;
                    Nmanu_ult = 1.65 * Nmanu;
                    Nult = max(Nmanu_ult, Ngust_ult);
                    % Wing Weight (converted to metric)
                    m_wing = 0.6*(4.22 * wing_S(i,j,k,l) * m_to_ft^2 + 1.642 * (10^-6) * Nult * (wing_b(i,j,k,l) * m_to_ft)^3 * (1 + 2 * wing_taper_ratio) * ...
                              (GW(n) * (mempty) * kg_to_lb^2)^0.5 / (tc_ratio * wing_S(i,j,k,l) * m_to_ft^2 * (1 + wing_taper_ratio))) / kg_to_lb*.8;
                    
                    % Fuselage Weight (converted to metric)
                    % W_fuselage = 0.0737 * (2 * (D_fuselage * m_to_ft) * (v_cruise * m_to_ft)^0.338 * (L_fuselage * m_to_ft)^0.857 * ...
                    %               (GW(n) * kg_to_lb * Nult)^0.286)^1.1 / kg_to_lb;
        %-----------HYDROGEN FUEL CELL-----------------------------------------------------------------------------------------------------%
                    sed_hydrogen = 16000; %Watt hr/Kg(specific energy density of an efficient Hydrogen cell)
                    sed_battery= 350; % Watt hr
                    m_hydrogen(i,j,k,l) =energy_hydrogen(i,j,k,l) / sed_hydrogen/0.15;%kg
                    m_battery(i,j,k,l)= energy_battery(i,j,k,l) / sed_battery;
                    mfuel_system(i,j,k,l)=132;%power_cruise(i,j,k,l)/1000*2.5;
                    mhydrogen_fuel_cell(i,j,k,l)= mfuel_system(i,j,k,l)+ m_hydrogen(i,j,k,l);
                    mfuel_cell(i,j,k,l)= mhydrogen_fuel_cell(i,j,k,l)+m_battery(i,j,k,l);
                    % mfuel_cell(i,j,k,l) = total_energy/(1000*3600*0.348061)
                    
                    
    
        %-----------FIXED--------------------------------------------------------------------------------------------------
                    m_avionics=0.05*mempty;% mass of avionics
                    manti_ice=0;%8*(GW(n)/1000);%mass of anti ice equipments
                    m_instruments=0.4535*3.5*(GW(n)*2.2/1000)^1.3;%mass of instruments
                    mfixed=m_avionics+manti_ice+m_instruments;% includes mass of avionics and mass of ribs+rods+payload support, anti ice and equipments
        %-----------NEW EMPTY WEIGHT---------------------------------------------------------------------------------------
                    mempty = m_rotor_group + mfuselage + mcontrols + melec + mlg + mfixed + m_transmission+ m_wing ;
        %-----------NEW GROSS WEIGHT----------------------------------------------------------------------------------------
                    mgross(i,j,k,l) = mempty+mfuel_cell(i,j,k,l)+mpayload ;%new gross weight
                    m=m+1;
                    n=n+1;
                    GW(n) = mgross(i,j,k,l);
                    fprintf("current total mass :%4.3f \n",GW(n));
                    error= abs(GW(n)-GW(m));
                    % if(error<=0.01)
                    %     power_final(i,j,k,l)=power_1(count); 
                    %     thrust_final(i,j,k,l)=
                end
                if (solution_check==1)
                    V1(i,j,k,l) = R;% radius
                    V2(i,j,k,l) = Vtip;%tip speed
                    V3(i,j,k,l) = thrust_total(i,j,k,l);%total thrust
                    V4(i,j,k,l) = err(i,j,k,l);%error margin
                    V5(i,j,k,l) = torque_h(i,j,k,l);% total torque
                    V6(i,j,k,l) = rpm(i,j,k,l);%rpm of blade
                    V7(i,j,k,l)= Vtip/sqrt(1.4*287*T);%mach number
                    O1(i,j,k,l) = Power_total_hover(i,j,k,l);%total hover power
                    O2(i,j,k,l) = mgross(i,j,k,l);%gross weight
                    O3(i,j,k,l) = energy_hydrogen(i,j,k,l);%total energy
                    O4(i,j,k,l) = mfuel_cell(i,j,k,l);%total battery mass
                    O5(i,j,k,l) = theta_h(i,j,k,l)*180/pi;%collective in degrees
                    O6(i,j,k,l) = thrust_total(i,j,k,l)./power_h(i,j,k,l);%power loading(N/W)
                    O7(i,j,k,l) = (thrust_total(i,j,k,l)/N_rotors)./(pi.*V1(i,j,k,l).^2);%disk loading(N/m^2)
                    O8(i,j,k,l) = power_mech(i,j,k,l);
                end
                fprintf("Total mass :%4.3f \n",GW(n));
                save_file=false;
                if save_file==true
                    fprintf("saving")
                     % Create a table with the variables
                    data = table({'Number of Blades', Nb;'NumberOfRotors', N_rotors;  'Radius', V1(i,j,k,l); 'TipSpeed', V2(i,j,k,l); 
                      'TotalThrust', V3(i,j,k,l); 'Extra thrust at hover', V4(i,j,k,l); 
                      'TotalTorque_per_motor Nm', V5(i,j,k,l); 'RPM', V6(i,j,k,l); 'MachNumber', V7(i,j,k,l); 
                      'TotalHoverPower Watt', O1(i,j,k,l); 'GrossWeight', O2(i,j,k,l); 
                      'TotalEnergy', O3(i,j,k,l); 'TotalBatteryMass', O4(i,j,k,l); 
                      'CollectiveInDegrees', O5(i,j,k,l); 'PowerLoading', O6(i,j,k,l); 
                      'DiskLoading', O7(i,j,k,l); 'MechanicalPower', O8(i,j,k,l); 
                      'BatteryMass', mfuel_cell; 'EmptyMass', mempty; 
                      'FuselageMass', mfuselage;'ClimbSpeed', Vc; 'DescentSpeed', Vd;
                      'CruiseVelocity', V_cruise(k); 'CruiseVelocity_Climb', V_cruise_climb; 
                      'AspectRatio', AR; 'TwistRate', thetatw(l); 'Max endurance Velocity', Vendu;'T9 time to be maximised for loiter',T9(i)});
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
    end
end
%% -----------------------------------Saving Variable data------------------------------------
% if save_variable
%     % Check if T9(i) and V_endu_cruise exist
%     if exist('T9(i)', 'var') && exist('V_endu_cruise', 'var')
%         % Ensure both are scalars
%         if isscalar(T9(i)) && isscalar(V_endu_cruise)
%             % Replace '.' with '_' in file-safe names
%             T9_str = strrep(sprintf('%.2f', T9(i)/60), '.', '_');
%             Vendu_str = strrep(sprintf('%.2f', V_endu_cruise), '.', '_');
%             %filename = sprintf('T_%s.mat', T9_str, Vendu_str);
%             filename = sprintf('Design Point %.0f.mat', counter);   
%             counter = counter+1;
% 
%             save(filename);  % Saves all workspace variables
%             fprintf('Variables saved to %s\n', filename);
%         else
%             error('T9 and V_endu_cruise must be scalars to name the file.');
%         end
%     else
%         error('T9 and/or V_endu_cruise are not defined.');
%     end
% end


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
