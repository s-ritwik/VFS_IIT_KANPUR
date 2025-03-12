Instructions:
First, clone this repo locally on your system, [Watch this Video for a quick tutorial](https://www.youtube.com/watch?v=EhxPBMQFCaI)
Add all your work here, and keep committing(and explaining) so everyone is up to date :)

# [Weight_performance](./weight_performance)
It contains the MATLAB scripts to estimate the weight using empirical (historical) formulas and simultaneously estimate performance parameters for our Aircraft.

## Key Files

- [main.m](./weight_performance/main.m)  
  *Description: This script serves as the main entry point for running the weight and performance estimation calculations. Just play around with Radius, RPM and different hyperparameters for the aircraft and try to get the most optimal results.*

- [Rotor_opt.mlx](./weight_performance/Rotor_opt.mlx)  
  *Description: This script is BEMT with Prandtl tip loss function which calculates the forces and moments of our blade *

- [NAC0012.mlx](./weight_performance/NAC0012.mlx)  
  *Description: Contains normal airfoil analysis Cl,Cd and alpha used by [Rotor_opt.mlx](./weight_performance/Rotor_opt.mlx) to perform calculations*
