% Compilation of inputs for sizing the actuation system and its dynamics
% Configuration A means: all area is used for acting
% Configuration B means: only low airspeed area is used for acting
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
% Input data --------------------------------------------------------------
% Sizing criteria ---------------------------------------------------------
loadFlowMargin = 1.1; % QNL = loadFlowMargin*QNL
sizingCriteriaSingleTandem = 4/5;
A_sizingCriteria = 4/5; % Max hinge respect stall hinge
B_sizingCriteria = 2/3; % Max hinge respect stall hinge
ram2body_pin2pin_stiffness_ratio = 0.9;
volumeRatio = 1.5; % TotalVolume = VolumeRatio*Area*actuatoStroke
%Hm0 = 1.2724*(70e3/sizingCriteriaSingleTandem); % Equivalent hinge moment to accomplish stiffness req
% Load and rate requirements ----------------------------------------------
Hm0 = 0;
Hm1 = 70e3; % Maximum hinge moment [N*m]
Hm2 = 24.5e3; % Hinge moment at maximum surface rate [N*m]
dotdelta0_deg = 0; % Surface rate at Hm1 [deg/s]
dotdelta1_deg = 20; % Surface rate at Hm1 [deg/s]
dotdelta2_deg = 80; % Surface rate at Hm2 = Maximum surface rate [deg/s]
A_rateLimit_deg = 80; % Maximum commanded surface rate [deg/s]
B_rateLimit_deg = 80; % Maximum commanded surface rate [deg/s]
% Stiffness requirement
stiffnessReq = 5e7; % N/m
% Geometric and kinetic data ---------------------------------------------- 
arm = 0.15; % actuation attacking arm [m]
deltaRange_deg = 60; % Surface range [deg]
inertia_kgm2 = 270; % Inertia respect rotation point [Kg*m^2]
% Hydraulic data ----------------------------------------------------------
PS_psi = 5000; % PressureSupply [psi]
Bulk_psi = 1e5; % Effective bulk modulus [psi]
density = 867; % Kg/m^3 at 15 degC
densityTempRatio = 1.3; % Kg/m^3/degC
cd = 0.7; % Dimensionless
viscosity = 1.4e-2; % Pa*s
internalLeak_lpm = 2; % Leak at supply pressure
% Valve data
valveStroke_mm = 1; % mm
radialClearance_mm = 5e-3; % mm
% Save data ---------------------------------------------------------------
save('actuationInputDataListDualTandem.mat',...
    'Hm0','Hm1','Hm2','arm','PS_psi',...
    'dotdelta0_deg','dotdelta1_deg','dotdelta2_deg',...
    'A_rateLimit_deg','B_rateLimit_deg','deltaRange_deg',...
    'inertia_kgm2','Bulk_psi','volumeRatio','internalLeak_lpm',...
    'ram2body_pin2pin_stiffness_ratio','stiffnessReq',...
    'A_sizingCriteria','B_sizingCriteria','loadFlowMargin',...
    'density','densityTempRatio','cd','viscosity',...
    'valveStroke_mm','radialClearance_mm');

