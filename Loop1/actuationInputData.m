% Compilation of inputs for sizing the actuation system and its dynamics
clear all
% Input data --------------------------------------------------------------
Hm1 = 70e3; % Maximum hinge moment [N*m]
Hm2 = 24.5e3; % Hinge moment at maximum surface rate [N*m]
arm = 0.1; % actuation attacking arm [m]
dotdelta1_deg = 20; % Surface rate at Hm1 [deg/s]
dotdelta2_deg = 80; % Surface rate at Hm2 = Maximum surface rate [deg/s]
rateLimit_deg = 80; % Maximum commanded surface rate [deg/s]
deltaRange_deg = 60; % Surface range [deg]
PS_psi = 5000; % PressureSupply [psi]
inertia_kgm2 = 270; % Inertia respect rotation point [Kg*m^2]
Bulk_psi = 1e5; % Effective bulk modulus [psi]
volumeRatio = 1.5; % TotalVolume = VolumeRatio*Area*actuatoStroke
internalLeak_lpm = 1; % Leak at supply pressure
% Save data ---------------------------------------------------------------
save('actuationInputDataList.mat',...
    'Hm1','Hm2','arm',...
    'dotdelta1_deg','dotdelta2_deg',...
    'rateLimit_deg','deltaRange_deg',...
    'PS_psi','inertia_kgm2','Bulk_psi',...
    'volumeRatio','internalLeak_lpm');