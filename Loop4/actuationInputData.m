% Compilation of inputs for sizing the actuation system and its dynamics
clear all
% Input data --------------------------------------------------------------
sizingCriteria = 4/5; % Max hinge respect stall hinge
loadFlowMargin = 1.1; % QNL = loadFlowMargin*QNL
Hm0 = 0*70e3;
%Hm0 = 1.2724*(70e3/sizingCriteria); % Equivalent hinge moment for stiffness req [N*m]
Hm1 = 70e3; % Maximum hinge moment [N*m]
Hm2 = 24.5e3; % Hinge moment at maximum surface rate [N*m]
arm = 0.15; % actuation attacking arm [m]
dotdelta1_deg = 0*20; % Surface rate at Hm1 [deg/s]
dotdelta2_deg = 80; % Surface rate at Hm2 = Maximum surface rate [deg/s]
rateLimit_deg = 80; % Maximum commanded surface rate [deg/s]
deltaRange_deg = 60; % Surface range [deg]
stiffnessReq = 5e7; % Stiffness requirement N/m
PS_psi = 5000; % PressureSupply [psi]
inertia_kgm2 = 270; % Inertia respect rotation point [Kg*m^2]
Bulk_psi = 1e5; % Effective bulk modulus [psi]
volumeRatio = 1.5; % TotalVolume = VolumeRatio*Area*actuatoStroke
internalLeak_lpm = 1; % Leak at supply pressure
ram2body_pin2pin_stiffness_ratio = 0.9; % reduction of stiffness
% Save data ---------------------------------------------------------------
save('actuationInputDataList.mat',...
    'Hm0','Hm1','Hm2','arm',...
    'dotdelta1_deg','dotdelta2_deg',...
    'rateLimit_deg','deltaRange_deg',...
    'PS_psi','inertia_kgm2','Bulk_psi',...
    'volumeRatio','internalLeak_lpm','sizingCriteria','stiffnessReq',...
    'ram2body_pin2pin_stiffness_ratio','loadFlowMargin');
