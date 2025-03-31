% Experimental Data (from user)
Cq_exp = [1.23E-4, 1.4E-4, 1.72E-4, 1.903E-4, 2.06E-4, 2.3350E-4, ...
          2.8299E-4, 3.5786E-4, 3.87E-4, 4.378E-4, 5.304E-4];
Ct_exp = [8.205E-4, 0.0011, 0.0017, 0.0020, 0.0023, 0.0026, ...
          0.00324, 0.0040, 0.0045, 0.0049, 0.00579];

% Check if C_P and C_T exist
if ~exist('C_P', 'var') || ~exist('C_T', 'var')
    error('C_P and C_T must be defined in the workspace with 100 values each.');
end

% Plotting
figure;
hold on;
plot(C_P, C_T, 'b-', 'LineWidth', 1.5);               % Theoretical (100-point) curve
plot(Cq_exp, Ct_exp, 'ro', 'MarkerSize', 5, 'LineWidth', 1.5); % Experimental points

xlabel('C_q','FontSize',15,'FontWeight','bold');
ylabel('C_t','FontSize',15,'FontWeight','bold');
title('Comparison of Experimental and Theoretical C_t vs C_q','FontSize',20);
legend('Theoretical (C_P vs C_T)', 'Experimental (Cq vs Ct)', 'Location', 'Best');
set(gca, 'FontSize', 14);  % Increases tick label font size

grid on;
hold off;
