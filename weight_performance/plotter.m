%% LOAD DATASET ACCCORDINGLY T_60 means , T9 = 60 mins ie 60 mins endurance for section 9
% load('T_60.mat');
%%
% Meshgrid of R and RPM
[R_mesh, RPM_mesh] = meshgrid(R, RPM);  % R: (1x4), RPM: (1x4)
v_labels = squeeze(V_cruise);          % V_cruise: (1x5), get as column vector
colors = lines(length(V_cruise));      % Distinct colors for each surface
%% - line plot 
% Power hover
i = 1;

% Extract RPM and V_cruise
rpm_values = squeeze(RPM);         % (1x4)
v_cruise_values = squeeze(V_cruise); % (1x5)

% Prepare color set
colors = lines(length(v_cruise_values));

% Plot setup
figure; hold on;

for k = 1:length(v_cruise_values)
    % Extract Power_total_hover for fixed R(i), all RPMs, and V_cruise(k)
    y_vals = squeeze(Power_total_hover(i,:,k));  % (1x4)
    plot(rpm_values, y_vals, '-o', 'LineWidth', 2, 'Color', colors(k,:), ...
         'DisplayName', sprintf('V_{cruise} = %.1f m/s', v_cruise_values(k)));
end

xlabel('Rotor RPM');
ylabel('Total Hover Power (W)');
title(sprintf('Hover Power vs RPM at R = %.2f m', R(i)));
legend('Location', 'northwest');
grid on;

% Power cruise

% Extract RPM and V_cruise
rpm_values = squeeze(RPM);         % (1x4)
v_cruise_values = squeeze(V_cruise); % (1x5)
% Prepare color set
colors = lines(length(v_cruise_values));

% Plot setup
figure; hold on;

for k = 1:length(v_cruise_values)
    % Extract Power_total_hover for fixed R(i), all RPMs, and V_cruise(k)
    y_vals = squeeze(power_cruise(i,:,k));  % (1x4)
    plot(rpm_values, y_vals, '-o', 'LineWidth', 2, 'Color', colors(k,:), ...
         'DisplayName', sprintf('V_{cruise} = %.1f m/s', v_cruise_values(k)));
end

xlabel('Rotor RPM');
ylabel('Total Cruise Power (W)');
title(sprintf('Cruise Power vs RPM at R = %.2f m', R(i)));
legend('Location', 'northwest');
grid on;

%% === Plot 1: Gross Weight ===
figure; hold on;
colors = lines(length(v_labels));  % Or use your own color map

for k = 1:length(v_labels)
    Z = squeeze(mgross(:,:,k))';
    h = surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));  % Fixed RGB color for each surface
end

xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Gross Weight (kg)');
zlim([500, 800]);
title('Gross Weight vs R & RPM for Various V_{cruise}');

% Create manual legend
legend_entries = arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false);
legend(legend_entries, 'Location', 'northeastoutside');

grid on; view(135, 30); box on;


% Load dataset
load('T_60.mat');

%% --- 1. Fuel Cell Mass (mfuel_cell) ---
figure; hold on;
for k = 1:length(v_labels)
    Z = squeeze(mfuel_cell(:,:,k))';
    surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));
end
xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Fuel Cell Mass (kg)');
zlim([150,400]);
title('Fuel Cell Mass vs R & RPM for Various V_{cruise}');
legend(arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false), ...
       'Location', 'northeastoutside');
grid on; view(135, 30); box on;

%% --- 2. Total Hover Power (Power_total_hover) ---
figure; hold on;
for k = 1:length(v_labels)
    Z = squeeze(Power_total_hover(:,:,k))';
    surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));
end
xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Hover Power (W)');zlim([19,22])
zlim([100000,250000]);
title('Total Hover Power vs R & RPM for Various V_{cruise}');
legend(arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false), ...
       'Location', 'northeastoutside');
grid on; view(135, 30); box on;

%% --- 3. Cruise Power (Pcruise) ---
figure; hold on;
for k = 1:length(v_labels)
    Z = squeeze(power_cruise(:,:,k))';
    surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));
end
xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Cruise Power (W)');
% zlim([95000,220000]);
title('Cruise Power vs R & RPM for Various V_{cruise}');
legend(arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false), ...
       'Location', 'northeastoutside');
grid on; view(135, 30); box on;

%% --- 4. Disk Loading (O7) ---
figure; hold on;
for k = 1:length(v_labels)
    Z = squeeze(O7(:,:,k))';
    surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));
end
xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Disk Loading (kg/m^2)');
zlim([200,520]);
title('Disk Loading vs R & RPM for Various V_{cruise}');
legend(arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false), ...
       'Location', 'northeastoutside');
grid on; view(135, 30); box on;

%% --- 5. Power Loading (O8) ---
figure; hold on;
for k = 1:length(v_labels)
    Z = squeeze(O6(:,:,k))';
    surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));
end
xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Power Loading (kg/W)');
zlim([0.03,0.06]);
title('Power Loading vs R & RPM for Various V_{cruise}');
legend(arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false), ...
       'Location', 'northeastoutside');
grid on; view(135, 30); box on;

%% --- 6. Collective Angle (O5) ---
figure; hold on;
for k = 1:length(v_labels)
    Z = squeeze(O5(:,:,k))';
    surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));
end
xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Collective (\theta_0 in deg)');
zlim([18,30]);
title('Collective vs R & RPM for Various V_{cruise}');
legend(arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false), ...
       'Location', 'northeastoutside');
grid on; view(135, 30); box on;

%% --- 7. Wing Span (wing_b) ---
figure; hold on;
for k = 1:length(v_labels)
    Z = squeeze(wing_b(:,:,k))';
    surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));
end
xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Wing Span (m)');
zlim([2,8]);
title('Wing Span vs R & RPM for Various V_{cruise}');
legend(arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false), ...
       'Location', 'northeastoutside');
grid on; view(135, 30); box on;

%% --- 8. Wing Area (wing_S) ---
figure; hold on;
for k = 1:length(v_labels)
    Z = squeeze(wing_S(:,:,k))';
    surf(R_mesh, RPM_mesh, Z, ...
        'FaceAlpha', 0.6, ...
        'EdgeColor', 'none', ...
        'FaceColor', colors(k,:));
end
xlabel('Rotor Radius (m)');
ylabel('Rotor RPM');
zlabel('Wing Area (m^2)');
title('Wing Area vs R & RPM for Various V_{cruise}');
legend(arrayfun(@(v) sprintf('V_{cruise} = %.1f m/s', v), v_labels, 'UniformOutput', false), ...
       'Location', 'northeastoutside');
grid on; view(135, 30); box on;

