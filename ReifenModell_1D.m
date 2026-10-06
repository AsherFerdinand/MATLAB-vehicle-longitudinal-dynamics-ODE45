function [RPara_VA,RPara_HA] = ReifenModell_1D

% Front axle /////////////////////////////////////////////////////////////////////

% Reference values for wheel loads

RPara_VA.Fz_1        = 4000;        % [N]   Lower reference value for the wheel load

RPara_VA.Fz_2        = 8000;        % [N]   Upper reference value for the wheel load

% Parameters for the tire-road friction function (TMeasy)  -------

% Valid values for wheel load Fz_1:

RPara_VA.lambda_Mx_1 = 0.11;        % [-]   Longitudinal slip at maximum tire-road friction

RPara_VA.mu_Mx_1     = 1.1;         % [-]   Maximum longitudinal tire-road friction coefficient

RPara_VA.lambda_Gx_1 = 0.8;         % [-]   Longitudinal slip at transition to full sliding

RPara_VA.mu_Gx_1     = 0.9;         % [-]   Longitudinal sliding friction coefficient

RPara_VA.dF0_x_1     = 120000;      % [N]   Tire longitudinal stiffness (tread)

% Valid values for wheel load Fz_2:

RPara_VA.lambda_Mx_2 = 0.10;        % [-]   Longitudinal slip at maximum tire-road friction

RPara_VA.mu_Mx_2     = 1.0;         % [-]   Maximum longitudinal tire-road friction coefficient

RPara_VA.lambda_Gx_2 = 0.7;         % [-]   Longitudinal slip at transition to full sliding

RPara_VA.mu_Gx_2     = 0.9;         % [-]   Longitudinal sliding friction coefficient

RPara_VA.dF0_x_2     = 160000;      % [N]   Tire longitudinal stiffness (tread)

% Parameters for the transient tire model ----------

RPara_VA.cRx         = 200000;       % [N/m]     Longitudinal stiffness of the tire structure

RPara_VA.dRx         = 500;          % [N/(m/s)] Longitudinal damping of the tire structure

RPara_VA.rdyn        = 0.3433;       % [m]       Dynamic radius

RPara_VA.rstat       = 0.3261;       % [m]       Static radius


% Rear axle /////////////////////////////////////////////////////////////////////

% Reference values for wheel loads

RPara_HA.Fz_1        = 4000;        % [N]   Lower reference value for the wheel load

RPara_HA.Fz_2        = 8000;        % [N]   Upper reference value for the wheel load

% Parameters for the tire-road friction function (TMeasy)  -------

% Valid values for wheel load Fz_1:

RPara_HA.lambda_Mx_1 = 0.11;        % [-]   Longitudinal slip at maximum tire-road friction

RPara_HA.mu_Mx_1     = 1.1;         % [-]   Maximum longitudinal tire-road friction coefficient

RPara_HA.lambda_Gx_1 = 0.8;         % [-]   Longitudinal slip at transition to full sliding

RPara_HA.mu_Gx_1     = 0.9;         % [-]   Longitudinal sliding friction coefficient

RPara_HA.dF0_x_1     = 120000;      % [N]   Tire longitudinal stiffness (tread)

% Valid values for wheel load Fz_2:

RPara_HA.lambda_Mx_2 = 0.10;        % [-]   Longitudinal slip at maximum tire-road friction

RPara_HA.mu_Mx_2     = 1.0;         % [-]   Maximum longitudinal tire-road friction coefficient

RPara_HA.lambda_Gx_2 = 0.7;         % [-]   Longitudinal slip at transition to full sliding

RPara_HA.mu_Gx_2     = 0.9;         % [-]   Longitudinal sliding friction coefficient

RPara_HA.dF0_x_2     = 160000;      % [N]   Tire longitudinal stiffness (tread)

% Parameters for the transient tire model ----------

RPara_HA.cRx         = 200000;       % [N/m]     Longitudinal stiffness of the tire structure

RPara_HA.dRx         = 500;          % [N/(m/s)] Longitudinal damping of the tire structure

RPara_HA.rdyn        = 0.3433;       % [m]       Dynamic radius

RPara_HA.rstat       = 0.3261;       % [m]       Static radius