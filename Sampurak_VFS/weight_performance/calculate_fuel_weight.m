function [fuel_weight, engine_weight] = calculate_fuel_weight(power_required, time_hours, index,energy,Pclimb,Pdescent,Pff)
    % Check if the correct index is provided
    if index == 1 % AR682R UAV ENGINES
        % Data from the table
        power = [85.0, 76.5, 68.0, 59.5, 51.0, 42.5]; % in bhp
        sfc = [0.54, 0.54, 0.55, 0.55, 0.56, 0.56]; % in lb/bhp/hr
        fuel_usage_us_gph = [7.6, 6.8, 6.1, 5.4, 4.7, 3.9]; % in gallons per hour (US)
        
        % Interpolation to find SFC and fuel usage for the given power
        sfc_interp = interp1(power, sfc, power_required, 'linear');
        fuel_usage_us_gph_interp = interp1(power, fuel_usage_us_gph, power_required, 'linear');
        
        % Convert US gallons to pounds (assuming AVGAS density 6.01 lb/US gallon)
        fuel_usage_lb_per_hr = fuel_usage_us_gph_interp * 6.01;
        
        % Calculate total fuel weight required
        fuel_weight_lb = fuel_usage_lb_per_hr * time_hours; % in pounds
        engine_weight = 56.5;
        % Convert pounds to kilograms (1 pound = 0.453592 kg)
        fuel_weight = fuel_weight_lb * 0.453592;
        
    elseif index == 2 % ROTAX 914 UL
        engine_weight = 78.3;
        % fuel_weight = 20 * time_hours * 0.72;
        RPM =@(x) 55*(x-40)+2500;
        FUEL=@(y) (RPM(y)-2500)/203.077+11.25;
            
        fuel_hover = FUEL(power_required);
        fuel_climb=FUEL(Pclimb);
        fuel_descent=FUEL(Pdescent);
        fuel_ff=FUEL(Pff);
        vfor=15;
        range=12;%kms

        time_range=range*1000/vfor/60/60;

        % Calculate total fuel consumption in gallons
        total_fuel_liters = fuel_hover * 15/60 +2*fuel_climb*3/60+2*fuel_descent*5/60+fuel_ff*time_range;
    
    
        % Calculate fuel weight in kilograms using fuel density
        fuel_weight = total_fuel_liters * 0.72*3*1.05; % kg
        
    elseif index == 3 % Hirth
        engine_weight = 36+7;
        
        % % Data from the table
        % power_kW = [14.14, 16.65, 18.66, 22.69, 25.27, 30.70, 39.54, 44.67, 48.36, 49.05, 52.30, 50.75];
        % power_hp = power_kW * 1.34102; % Convert kW to hp
        % fuel_consumption_lbs_per_hp_hr = [0.991, 0.973, 1.002, 0.846, 0.834, 0.920, 0.837, 0.797, 0.750, 0.746, 0.730, 0.779];
        % 
        % % Interpolate the specific fuel consumption for the given power requirement
        % specific_fuel_consumption = interp1(power_hp, fuel_consumption_lbs_per_hp_hr, power_required, 'linear', 'extrap');
        % 
        % % Calculate the total fuel consumption in lbs
        % total_fuel_lbs = power_required * specific_fuel_consumption * time_hours;
        % 
        % % Convert the total fuel consumption to kilograms (1 lb = 0.453592 kg)
        % fuel_weight = total_fuel_lbs * 0.453592;
        % fuel_weight=(2.1350+0.29*power_required)*time_hours;
        fuel_weight=(2.1350)*time_hours+0.29*energy*1.341/1000;
    elseif index==4 %https://uavenginesltd.co.uk/wp-content/uploads/2019/03/ar682r.pdf
        engine_weight=56.5;
        % density_fuel = 0.72; % kg/L

        % Data from the provided table
        % power_bhp = [85.0, 76.5, 68.0, 59.5, 51.0, 42.5];
        % sfc = [0.54, 0.54, 0.55, 0.55, 0.56, 0.56]; % lb/bhp/hr
        % 
        % % Interpolate SFC for the given power
        % sfc_required = interp1(power_bhp, sfc, power_required, 'linear');
        % 
        % % Calculate fuel consumption in pounds (lb)
        % fuel_consumption_lb = power_required * sfc_required * time_hours;
        % 
        % % Convert fuel consumption from pounds to kilograms
        % fuel_weight = fuel_consumption_lb * 0.453592; % 1 lb = 0.453592 kg
        density_fuel = 0.72; % kg/L
        gallon_to_liter = 3.78541; % 1 US gallon = 3.78541 liters
    
        % Data from the provided table
        power_bhp = [85.0, 76.5, 68.0, 59.5, 51.0, 42.5];
        fuel_usage_us_gal_hr = [7.6, 6.8, 6.1, 5.4, 4.7, 3.9]; % US gallons per hour
    
        % Interpolate fuel usage for the given power
        fuel_hover = interp1(power_bhp, fuel_usage_us_gal_hr, power_required, 'linear','extrap');
        fuel_climb=interp1(power_bhp, fuel_usage_us_gal_hr, Pclimb, 'linear','extrap');
        fuel_descent=interp1(power_bhp, fuel_usage_us_gal_hr, Pdescent, 'linear','extrap');
        fuel_ff=interp1(power_bhp, fuel_usage_us_gal_hr, Pff, 'linear','extrap');
        vfor=15;
        range=12;%kms

        time_range=range*1000/vfor/60/60;

        % Calculate total fuel consumption in gallons
        total_fuel_gallons = fuel_hover * 15/60 +2*fuel_climb*3/60+2*fuel_descent*5/60+fuel_ff*time_range;
    
        % Convert fuel consumption from gallons to liters
        total_fuel_liters = total_fuel_gallons * gallon_to_liter;
    
        % Calculate fuel weight in kilograms using fuel density
        fuel_weight = total_fuel_liters * density_fuel*3; % kg
    else
        error('Invalid index. Currently, only index 1, 2, and 3 are supported.');
    end
end
