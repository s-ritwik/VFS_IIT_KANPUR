function [rho,P,T] = atmos(z,P0,T0,den0)

z1 = z;                                 % z is geomentric altitude m
r = 6356766;                            % r is radius of the earth in m.
R = 287;                                % R is universal gas constant in joule per kg-kelvin (J/(kg*K) 
g_0 = 9.81;                             % g_0 is gravity at sea level in meter per square sec (m/s2)

    if ((z1 >=0) && (z1 <= 11000))                     % Gradient layer        0-11 km
          h = (r*z1)/(r + z1);%geopotential altitute
          h1 = 0;% sea level 
          a = -0.0065;%defined constant for the layer 1
          T = T0 + a*(h - h1);% temperature at altitude h
          P = P0*((T/T0)^(-g_0/(a*R))) ;% pressure for altitude h
          rho = den0*(T/T0)^((-g_0/(a*R)) - 1) ;%density for altitude h  
    end
end