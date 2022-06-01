% Program for estimating dynamic behaviour of the actuation system
clear all
%--------------------------------------------------------------------------
% Input data and parameters------------------------------------------------
load('actuationInputDataListDualTandem.mat')
% Conversion---------------------------------------------------------------
load('actuationUnitsConversion.mat')
% Sizing data--------------------------------------------------------------
load('actuationSizingDataDualTandem.mat');
load('valveSizingDataDualTandem.mat')
% -------------------------------------------------------------------------
% Estimated parameters - first level --------------------------------------
A_totalFlowPressCoeff_m3sPa = A_Kc0_m3s_Pa + ...
    (internalLeak_lpm * lpm2m3s) / (PS_psi * psi2Pa);
B_totalFlowPressCoeff_m3sPa = B_Kc0_m3s_Pa + ...
    (internalLeak_lpm * lpm2m3s) / (PS_psi * psi2Pa);
eqMass = inertia_kgm2 / (arm^2);
Bulk_Pa = Bulk_psi*psi2Pa;
A_stiffness = (4*Bulk_Pa*A_area^2) / A_totalVol;
B_stiffness = (4*Bulk_Pa*B_area^2) / B_totalVol;
% -------------------------------------------------------------------------
% Estimated parameters - second level--------------------------------------
A_natOmega = sqrt(A_stiffness / eqMass);
B_natOmega = sqrt(B_stiffness / eqMass);
A_natFreq_ConfA = A_natOmega / (2*pi);
B_natFreq_ConfB = B_natOmega / (2*pi);
A_dampRatio = 0.1+(A_totalFlowPressCoeff_m3sPa/A_area)*sqrt(eqMass*Bulk_Pa/A_totalVol);
B_dampRatio = 0.1+(B_totalFlowPressCoeff_m3sPa/B_area)*sqrt(eqMass*Bulk_Pa/B_totalVol);
% -------------------------------------------------------------------------
% Transfer functions estimation -------------------------------------------
% % Position - Valve position dynamics-------------------------------------
A_gainPosCurrent = [A_Kq0_m3s_mm / A_area];
B_gainPosCurrent = [B_Kq0_m3s_mm / B_area];
A_numPosCurrent = [1*A_gainPosCurrent];
B_numPosCurrent = [1*B_gainPosCurrent];
A_denPosCurrent = [1/A_natOmega^2 2*A_dampRatio/A_natOmega 1 0]; 
B_denPosCurrent = [1/B_natOmega^2 2*B_dampRatio/B_natOmega 1 0]; 
A_posCurrent_TF = tf(A_numPosCurrent,A_denPosCurrent);
B_posCurrent_TF = tf(B_numPosCurrent,B_denPosCurrent);
% Position - Load dynamics-------------------------------------------------
% Numerator, gain and TF for (position / load) transfer
A_gainPosLoad = -A_totalFlowPressCoeff_m3sPa / A_area^2; 
B_gainPosLoad = -B_totalFlowPressCoeff_m3sPa / B_area^2; 
A_numPosLoad = [A_gainPosLoad/(2*A_dampRatio*A_natOmega) A_gainPosLoad];
B_numPosLoad = [B_gainPosLoad/(2*B_dampRatio*B_natOmega) B_gainPosLoad];
A_denPosLoad = A_denPosCurrent;
B_denPosLoad = B_denPosCurrent;
A_posLoad_TF = tf(A_numPosLoad,A_denPosLoad);
B_posLoad_TF = tf(B_numPosLoad,B_denPosLoad);
% % -----------------------------------------------------------------------
% % Plot-------------------------------------------------------------------
% -------------------------------------------------------------------------
freq_init = 5; % Hz
freq_end = 30; % Hz
freq_step = 0.05; % Hz
w_init = freq_init*2*pi; % rad/s
w_end = freq_end*2*pi; % rad/s
w_step = freq_step*2*pi;
w_range = w_init:w_step:w_end;

[A_magPosLoadRaw,A_phasePosLoadRaw,A_omegaPosLoad] = ...
    bode(A_posLoad_TF,w_range);
[B_magPosLoadRaw,B_phasePosLoadRaw,B_omegaPosLoad] = ...
    bode(B_posLoad_TF,w_range);
A_freqLoadPos = A_omegaPosLoad/2/pi;
B_freqLoadPos = B_omegaPosLoad/2/pi;
for j=1:length(A_omegaPosLoad)
    A_magLoadPos(j,1) = 1/A_magPosLoadRaw(1,1,j);
    B_magLoadPos(j,1) = 1/B_magPosLoadRaw(1,1,j);
    A_phaseLoadPos(j,1) = A_phasePosLoadRaw(1,1,j);
    B_phaseLoadPos(j,1) = B_phasePosLoadRaw(1,1,j);
end
% % Equivalent stiffness considering terminals provide a stifffness which is
% % one order of magnitude greater (this reduces the stiffness in 10% aprox)
% magLoadPos_ConfA_1 = ram2body_pin2pin_stiffness_ratio*magLoadPos_ConfA_1;
% magLoadPos_ConfB_1 = ram2body_pin2pin_stiffness_ratio*magLoadPos_ConfB_1;
% 
% magLoadPos_ConfA_2 = ram2body_pin2pin_stiffness_ratio*magLoadPos_ConfA_2;
% magLoadPos_ConfB_2 = ram2body_pin2pin_stiffness_ratio*magLoadPos_ConfB_2;
% 
% % stiffnessPoint_A = [natFreq,(stiffness*1e-3)/sqrt(2)];
% % stiffnessPoint_B = [natFreq,(stiffness*1e-3)];

semilogx(A_omegaPosLoad/2/pi,A_magLoadPos*1e-3);
% semilogx(A_omegaPosLoad/2/pi,A_magLoadPos*1e-3,'b--',...
%     B_omegaPosLoad/2/pi,B_magLoadPos*1e-3,'r--');
grid on
xlabel('Frequency (Hz)')
ylabel('Stiffness (N/mm)')
text(6,4e6,num2str(A_stiffness*1e-6))
%text(6,3e6,num2str(B_stiffness*1e-6))
%legend('High speed configuration','Low speed configuration')
