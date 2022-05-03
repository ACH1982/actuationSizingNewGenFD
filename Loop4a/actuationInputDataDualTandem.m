% Compilation of inputs for sizing the actuation system and its dynamics
% Configuration A means: all area is used for acting
% Configuration B means: only low airspeed area is used for acting
clear all
% Input data --------------------------------------------------------------
loadFlowMargin = 1.1; % QNL = loadFlowMargin*QNL
sizingCriteriaSingleTandem = 4/5;
%Hm0 = 1.2724*(70e3/sizingCriteriaSingleTandem); % Equivalent hinge moment to accomplish stiffness req
Hm0 = 0;
Hm1 = 70e3; % Maximum hinge moment [N*m]
Hm2 = 24.5e3; % Hinge moment at maximum surface rate [N*m]
arm = 0.15; % actuation attacking arm [m]
dotdelta0_deg = 0; % Surface rate at Hm1 [deg/s]
dotdelta1_deg = 20; % Surface rate at Hm1 [deg/s]
dotdelta2_deg = 80; % Surface rate at Hm2 = Maximum surface rate [deg/s]
stiffnessReq = 5e7; % N/m
A_rateLimit_deg = 80; % Maximum commanded surface rate [deg/s]
B_rateLimit_deg = 80; % Maximum commanded surface rate [deg/s]
deltaRange_deg = 60; % Surface range [deg]
A_sizingCriteria = 4/5; % Max hinge respect stall hinge
B_sizingCriteria = 2/3; % Max hinge respect stall hinge
PS_psi = 5000; % PressureSupply [psi]
inertia_kgm2 = 270; % Inertia respect rotation point [Kg*m^2]
Bulk_psi = 1e5; % Effective bulk modulus [psi]
volumeRatio = 1.5; % TotalVolume = VolumeRatio*Area*actuatoStroke
internalLeak_lpm = 3; % Leak at supply pressure
ram2body_pin2pin_stiffness_ratio = 0.9;
% Save data ---------------------------------------------------------------
save('actuationInputDataListDualTandem.mat',...
    'Hm0','Hm1','Hm2','arm','PS_psi',...
    'dotdelta0_deg','dotdelta1_deg','dotdelta2_deg',...
    'A_rateLimit_deg','B_rateLimit_deg','deltaRange_deg',...
    'inertia_kgm2','Bulk_psi','volumeRatio','internalLeak_lpm',...
    'ram2body_pin2pin_stiffness_ratio','stiffnessReq',...
    'A_sizingCriteria','B_sizingCriteria','loadFlowMargin');

