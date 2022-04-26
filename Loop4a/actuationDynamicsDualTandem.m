% Program for estimating dynamic behaviour of the actuation system
clear all
%--------------------------------------------------------------------------
% Input data and parameters------------------------------------------------
load('actuationInputDataListDualTandem.mat')
% Conversion---------------------------------------------------------------
load('actuationUnitsConversion.mat')
% Sizing data--------------------------------------------------------------
load('actuationSizingDataDualTandem.mat');
% -------------------------------------------------------------------------
% Estimated parameters - first level --------------------------------------
% 1 mA of valve stroke is considered for Kp (pressGain) 
% and Kq (flowGain) estimation
pressGain_MPa_mA = 1.3*(PS_psi*psi2Pa*1e-6)/0.05;
pressGain_Pa_mA = 1.3*(PS_psi*psi2Pa)/0.05; 
flowGain_ConfA_lpm_mA = QNL_ConfA_m3s*m3s2lpm;
flowGain_ConfB_lpm_mA = QNL_ConfB_m3s*m3s2lpm;

flowGain_ConfA_m3s_mA = flowGain_ConfA_lpm_mA * lpm2m3s;
flowGain_ConfB_m3s_mA = flowGain_ConfB_lpm_mA * lpm2m3s;

flowPressCoeff_ConfA_m3sPa = flowGain_ConfA_m3s_mA / pressGain_Pa_mA;
flowPressCoeff_ConfB_m3sPa = flowGain_ConfB_m3s_mA / pressGain_Pa_mA;

totalFlowPressCoeff_ConfA_m3sPa = flowPressCoeff_ConfA_m3sPa + ...
    (internalLeak_lpm * lpm2m3s) / (PS_psi * psi2Pa);
totalFlowPressCoeff_ConfB_m3sPa = flowPressCoeff_ConfB_m3sPa + ...
    (internalLeak_lpm * lpm2m3s) / (PS_psi * psi2Pa);

eqMass = inertia_kgm2 / (arm^2);
Bulk_Pa = Bulk_psi*psi2Pa;
stiffness_ConfA = (4*Bulk_Pa*(area_ConfA)^2) / totalVol_ConfA;
stiffness_ConfB = (4*Bulk_Pa*(area_ConfB)^2) / totalVol_ConfB;
% -------------------------------------------------------------------------
% Estimated parameters - second level--------------------------------------
natOmega_ConfA = sqrt(stiffness_ConfA / eqMass);
natOmega_ConfB = sqrt(stiffness_ConfB / eqMass);
natFreq_ConfA = natOmega_ConfA / (2*pi);
natFreq_ConfB = natOmega_ConfB / (2*pi);
% dampRatioRaw = (totalFlowPressCoeff_m3sPa/area)*sqrt(eqMass*Bulk_Pa/totalVol);
dampRatio_1 = 0.4;
dampRatio_2 = 0.1;
% the totalFlowPressCoeff should be redefined if the dampRatio is
% established in a specific value: dampRatio based on totalFlowPressCoeff +
% 0.2 --> totalFlowPressCoeff should be computed from this dampRatio
totalFlowPressCoeff_ConfA_1_m3sPa = dampRatio_1*area_ConfA/sqrt(Bulk_Pa*eqMass/totalVol_ConfA);
totalFlowPressCoeff_ConfB_1_m3sPa = dampRatio_1*area_ConfB/sqrt(Bulk_Pa*eqMass/totalVol_ConfB);

totalFlowPressCoeff_ConfA_2_m3sPa = dampRatio_2*area_ConfA/sqrt(Bulk_Pa*eqMass/totalVol_ConfA);
totalFlowPressCoeff_ConfB_2_m3sPa = dampRatio_2*area_ConfB/sqrt(Bulk_Pa*eqMass/totalVol_ConfB);
% -------------------------------------------------------------------------
% Transfer functions estimation -------------------------------------------
% % Position - Valve position dynamics-------------------------------------
% denPosCurrent_1 = [1/natOmega^2 2*dampRatio_1/natOmega 1 0]; 
% denPosCurrent_2 = [1/natOmega^2 2*dampRatio_2/natOmega 1 0]; 
% % Numerator, gain and TF for (position / valvePosition) transfer
% gainPosCurrent = [flowGain_m3s_mA / area];
% numPosCurrent = [1*gainPosCurrent];
% posCurrent_TF_1 = tf(numPosCurrent,denPosCurrent_1);
% posCurrent_TF_2 = tf(numPosCurrent,denPosCurrent_2);
% Position - Load dynamics-------------------------------------------------
% Numerator, gain and TF for (position / load) transfer
gainPosLoad_ConfA_1 = -totalFlowPressCoeff_ConfA_1_m3sPa / area_ConfA^2; 
gainPosLoad_ConfB_1 = -totalFlowPressCoeff_ConfB_1_m3sPa / area_ConfB^2; 

gainPosLoad_ConfA_2 = -totalFlowPressCoeff_ConfA_2_m3sPa / area_ConfA^2; 
gainPosLoad_ConfB_2 = -totalFlowPressCoeff_ConfB_2_m3sPa / area_ConfB^2; 

numPosLoad_ConfA_1 = [1*gainPosLoad_ConfA_1/(2*dampRatio_1*natOmega_ConfA) 1*gainPosLoad_ConfA_1];
numPosLoad_ConfB_1 = [1*gainPosLoad_ConfB_1/(2*dampRatio_1*natOmega_ConfB) 1*gainPosLoad_ConfB_1];

numPosLoad_ConfA_2 = [1*gainPosLoad_ConfA_2/(2*dampRatio_2*natOmega_ConfA) 1*gainPosLoad_ConfA_2];
numPosLoad_ConfB_2 = [1*gainPosLoad_ConfB_2/(2*dampRatio_2*natOmega_ConfB) 1*gainPosLoad_ConfB_2];

denPosLoad_ConfA_1 = [1/natOmega_ConfA^2 2*dampRatio_1/natOmega_ConfA 1 0]; 
denPosLoad_ConfA_2 = [1/natOmega_ConfA^2 2*dampRatio_2/natOmega_ConfA 1 0]; 

denPosLoad_ConfB_1 = [1/natOmega_ConfB^2 2*dampRatio_1/natOmega_ConfB 1 0]; 
denPosLoad_ConfB_2 = [1/natOmega_ConfB^2 2*dampRatio_2/natOmega_ConfB 1 0]; 

posLoad_TF_ConfA_1 = tf(numPosLoad_ConfA_1,denPosLoad_ConfA_1);
posLoad_TF_ConfB_1 = tf(numPosLoad_ConfB_1,denPosLoad_ConfB_1);

posLoad_TF_ConfA_2 = tf(numPosLoad_ConfA_2,denPosLoad_ConfA_2);
posLoad_TF_ConfB_2 = tf(numPosLoad_ConfB_2,denPosLoad_ConfB_2);
% -------------------------------------------------------------------------
% Plot---------------------------------------------------------------------
freq_init = 10; % Hz
freq_end = 40; % Hz
freq_step = 0.5; % Hz
w_init = freq_init*2*pi; % rad/s
w_end = freq_end*2*pi; % rad/s
w_step = 0.25*freq_step*2*pi;
w_range = w_init:w_step:w_end;
% [magPosCurrentRaw_1,phasePosCurrentRaw_1,omegaPosCurrent_1] = ...
%     bode(posCurrent_TF_1,w_range);
% [magPosCurrentRaw_2,phasePosCurrentRaw_2,omegaPosCurrent_2] = ...
%     bode(posCurrent_TF_2,w_range);
% for i=1:length(omegaPosCurrent_1)
%     magPosCurrent_1(i,1) = magPosCurrentRaw_1(1,1,i);
%     phasePosCurrent_1(i,1) = phasePosCurrentRaw_1(1,1,i);
%     magPosCurrent_2(i,1) = magPosCurrentRaw_2(1,1,i);
%     phasePosCurrent_2(i,1) = phasePosCurrentRaw_2(1,1,i);
% end
[magPosLoadRaw_ConfA_1,phasePosLoadRaw_ConfA_1,omegaPosLoad_ConfA_1] = ...
    bode(posLoad_TF_ConfA_1,w_range);
[magPosLoadRaw_ConfB_1,phasePosLoadRaw_ConfB_1,omegaPosLoad_ConfB_1] = ...
    bode(posLoad_TF_ConfB_1,w_range);

[magPosLoadRaw_ConfA_2,phasePosLoadRaw_ConfA_2,omegaPosLoad_ConfA_2] = ...
    bode(posLoad_TF_ConfA_2,w_range);
[magPosLoadRaw_ConfB_2,phasePosLoadRaw_ConfB_2,omegaPosLoad_ConfB_2] = ...
    bode(posLoad_TF_ConfB_2,w_range);
freqLoadPos_ConfA_1 = omegaPosLoad_ConfA_1/2/pi;
freqLoadPos_ConfB_1 = omegaPosLoad_ConfB_1/2/pi;

freqLoadPos_ConfA_2 = omegaPosLoad_ConfA_2/2/pi;
freqLoadPos_ConfB_2 = omegaPosLoad_ConfB_2/2/pi;

for j=1:length(omegaPosLoad_ConfA_1)
    magLoadPos_ConfA_1(j,1) = 1/magPosLoadRaw_ConfA_1(1,1,j);
    magLoadPos_ConfB_1(j,1) = 1/magPosLoadRaw_ConfB_1(1,1,j);

    phaseLoadPos_ConfA_1(j,1) = phasePosLoadRaw_ConfA_1(1,1,j);
    phaseLoadPos_ConfB_1(j,1) = phasePosLoadRaw_ConfB_1(1,1,j);

    magLoadPos_ConfA_2(j,1) = 1/magPosLoadRaw_ConfA_2(1,1,j);
    magLoadPos_ConfB_2(j,1) = 1/magPosLoadRaw_ConfB_2(1,1,j);
    
    phaseLoadPos_ConfA_2(j,1) = phasePosLoadRaw_ConfA_2(1,1,j);
    phaseLoadPos_ConfB_2(j,1) = phasePosLoadRaw_ConfB_2(1,1,j);
end
% Equivalent stiffness considering terminals provide a stifffness which is
% one order of magnitude greater (this reduces the stiffness in 10% aprox)
magLoadPos_ConfA_1 = ram2body_pin2pin_stiffness_ratio*magLoadPos_ConfA_1;
magLoadPos_ConfB_1 = ram2body_pin2pin_stiffness_ratio*magLoadPos_ConfB_1;

magLoadPos_ConfA_2 = ram2body_pin2pin_stiffness_ratio*magLoadPos_ConfA_2;
magLoadPos_ConfB_2 = ram2body_pin2pin_stiffness_ratio*magLoadPos_ConfB_2;

% stiffnessPoint_A = [natFreq,(stiffness*1e-3)/sqrt(2)];
% stiffnessPoint_B = [natFreq,(stiffness*1e-3)];
% -------------------------------------------------------------------------
% Plot results ------------------------------------------------------------
% -------------------------------------------------------------------------
subplot(1,2,1)
semilogx(omegaPosLoad_ConfA_1/2/pi,magLoadPos_ConfA_1*1e-3,'b--',...
    omegaPosLoad_ConfA_2/2/pi,magLoadPos_ConfA_2*1e-3,'r--');
grid on
xlabel('Frequency (Hz)')
ylabel('Stiffness (N/mm)')
legend('Low speed conf (damping = 0.4)','Low speed conf (damping = 0.1)')
subplot(1,2,2)
semilogx(omegaPosLoad_ConfB_1/2/pi,magLoadPos_ConfB_1*1e-3,'b--',...
    omegaPosLoad_ConfB_2/2/pi,magLoadPos_ConfB_2*1e-3,'r--');
grid on
xlabel('Frequency (Hz)')
ylabel('Stiffness (N/mm)')
legend('High speed conf (damping = 0.4)','High speed conf (damping = 0.1)')
min(magLoadPos_ConfA_2*1e-3)

% save('freqLoadPos_1.mat','freqLoadPos_1')
% save('magLoadPos_1.mat','magLoadPos_1')
% save('freqLoadPos_2.mat','freqLoadPos_2')
% save('magLoadPos_2.mat','magLoadPos_2')
% csvwrite('freqLoadPos_1.txt',freqLoadPos_1);
% csvwrite('freqLoadPos_2.txt',freqLoadPos_2);
% csvwrite('magLoadPos_1.txt',magLoadPos_1);
% csvwrite('magLoadPos_2.txt',magLoadPos_2);



