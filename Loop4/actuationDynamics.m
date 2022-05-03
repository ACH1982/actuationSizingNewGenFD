% Program for estimating dynamic behaviour of the actuation system
clear all
%--------------------------------------------------------------------------
% Input data and parameters------------------------------------------------
load('actuationInputDataList.mat')
% Conversion---------------------------------------------------------------
load('actuationUnitsConversion.mat')
% Sizing data--------------------------------------------------------------
load('actuationSizingData.mat');
% -------------------------------------------------------------------------
% Estimated parameters - first level --------------------------------------
% 1 mA of valve stroke is considered for Kp (pressGain) 
% and Kq (flowGain) estimation
pressGain_MPa_mA = 1.3*(PS_psi*psi2Pa*1e-6)/0.05;
pressGain_Pa_mA = 1.3*(PS_psi*psi2Pa)/0.05; 
flowGain_lpm_mA = QNL_m3s*m3s2lpm;
flowGain_m3s_mA = flowGain_lpm_mA * lpm2m3s;
flowPressCoeff_m3sPa = flowGain_m3s_mA / pressGain_Pa_mA;
totalFlowPressCoeff_m3sPa = flowPressCoeff_m3sPa + ...
    (internalLeak_lpm * lpm2m3s) / (PS_psi * psi2Pa);
totalVol = deltaRange_deg*deg2rad * arm * area * volumeRatio;
eqMass = inertia_kgm2 / (arm^2);
Bulk_Pa = Bulk_psi*psi2Pa;
stiffness = (4*Bulk_Pa*(area)^2) / totalVol;
stiffness_Nmm = stiffness*1e-3;
% -------------------------------------------------------------------------
% Estimated parameters - second level--------------------------------------
natOmega = sqrt(stiffness / eqMass);
natFreq = natOmega / (2*pi);
dampRatioRaw = (totalFlowPressCoeff_m3sPa/area)*sqrt(eqMass*Bulk_Pa/totalVol);
dampRatioRaw = 0;
dampRatio_1 = dampRatioRaw + 0.4;
dampRatio_2 = dampRatioRaw + sqrt(2)/2;
% the totalFlowPressCoeff should be redefined if the dampRatio is
% established in a specific value: dampRatio based on totalFlowPressCoeff +
% 0.2 --> totalFlowPressCoeff should be computed from this dampRatio
totalFlowPressCoeff_1_m3sPa = dampRatio_1*area/sqrt(Bulk_Pa*eqMass/totalVol);
totalFlowPressCoeff_2_m3sPa = dampRatio_2*area/sqrt(Bulk_Pa*eqMass/totalVol);
% -------------------------------------------------------------------------
% Transfer functions estimation -------------------------------------------
denPosCurrent_1 = [1/natOmega^2 2*dampRatio_1/natOmega 1 0]; 
denPosCurrent_2 = [1/natOmega^2 2*dampRatio_2/natOmega 1 0]; 
denPosLoad_1 = [1/natOmega^2 2*dampRatio_1/natOmega 1 0]; 
denPosLoad_2 = [1/natOmega^2 2*dampRatio_2/natOmega 1 0]; 
% Numerator, gain and TF for (position / valvePosition) transfer-----------
gainPosCurrent = [flowGain_m3s_mA / area];
numPosCurrent = [1*gainPosCurrent];
posCurrent_TF_1 = tf(numPosCurrent,denPosCurrent_1);
posCurrent_TF_2 = tf(numPosCurrent,denPosCurrent_2);
% Numerator, gain and TF for (position / load) transfer
gainPosLoad_1 = -totalFlowPressCoeff_1_m3sPa / area^2; 
gainPosLoad_2 = -totalFlowPressCoeff_2_m3sPa / area^2; 
numPosLoad_1 = [1*gainPosLoad_1/(2*dampRatio_1*natOmega) 1*gainPosLoad_1];
numPosLoad_2 = [1*gainPosLoad_2/(2*dampRatio_2*natOmega) 1*gainPosLoad_2];
posLoad_TF_1 = tf(numPosLoad_1,denPosLoad_1);
posLoad_TF_2 = tf(numPosLoad_2,denPosLoad_2);
% -------------------------------------------------------------------------
% Plot---------------------------------------------------------------------
freq_init = 10; % Hz
freq_end = 40; % Hz
freq_step = 0.5; % Hz
w_init = freq_init*2*pi; % rad/s
w_end = freq_end*2*pi; % rad/s
w_step = 0.25*freq_step*2*pi;
w_range = w_init:w_step:w_end;
[magPosCurrentRaw_1,phasePosCurrentRaw_1,omegaPosCurrent_1] = ...
    bode(posCurrent_TF_1,w_range);
[magPosCurrentRaw_2,phasePosCurrentRaw_2,omegaPosCurrent_2] = ...
    bode(posCurrent_TF_2,w_range);
for i=1:length(omegaPosCurrent_1)
    magPosCurrent_1(i,1) = magPosCurrentRaw_1(1,1,i);
    phasePosCurrent_1(i,1) = phasePosCurrentRaw_1(1,1,i);
    magPosCurrent_2(i,1) = magPosCurrentRaw_2(1,1,i);
    phasePosCurrent_2(i,1) = phasePosCurrentRaw_2(1,1,i);
end
[magPosLoadRaw_1,phasePosLoadRaw_1,omegaPosLoad_1] = ...
    bode(posLoad_TF_1,w_range);
[magPosLoadRaw_2,phasePosLoadRaw_2,omegaPosLoad_2] = ...
    bode(posLoad_TF_2,w_range);
freqLoadPos_1 = omegaPosLoad_1/2/pi;
freqLoadPos_2 = omegaPosLoad_2/2/pi;
for j=1:length(omegaPosLoad_1)
    magLoadPos_1(j,1) = 1/magPosLoadRaw_1(1,1,j);
    phaseLoadPos_1(j,1) = phasePosLoadRaw_1(1,1,j);
    magLoadPos_2(j,1) = 1/magPosLoadRaw_2(1,1,j);
    phaseLoadPos_2(j,1) = phasePosLoadRaw_2(1,1,j);
end
% Equivalent stiffness considering terminals provide a stifffness which is
% one order of magnitude greater (this reduces the stiffness in 10% aprox)
magLoadPos_1 = ram2body_pin2pin_stiffness_ratio*magLoadPos_1;
magLoadPos_2 = ram2body_pin2pin_stiffness_ratio*magLoadPos_2;
stiffnessPoint_A = [natFreq,(stiffness*1e-3)/sqrt(2)];
stiffnessPoint_B = [natFreq,(stiffness*1e-3)];
% Minimum stiffness around natural frequency
for i=1:length(freqLoadPos_1)
    if (freqLoadPos_1(i) - natFreq*sqrt(1-2*dampRatio_1^2)) < 0.05;
        iResonance_1 = i;
    else
    end
end
for i=1:length(freqLoadPos_2)
    if (freqLoadPos_2(i) - natFreq*sqrt(1-2*dampRatio_2^2)) < 0.05;
        iResonance_2 = i;
    else
    end
end
% minImpedance_1 = magLoadPos_1(iResonance_1); % N/m
% minImpedance_2 = magLoadPos_2(iResonance_2); % N/m
% minImpedance = min(minImpedance_1,minImpedance_2);
% areaRatio = stiffnessReq / minImpedance;
% -------------------------------------------------------------------------
% Plot results ------------------------------------------------------------
% -------------------------------------------------------------------------
stiffnessLine_Nmm = stiffness_Nmm.*ones(length(omegaPosLoad_1));
verticalLine = 0:1e4:6e5;
natFreqLine = natFreq*ones(length(verticalLine));
semilogx(omegaPosLoad_1/2/pi,magLoadPos_1*1e-3,'r--',...
    omegaPosLoad_2/2/pi,magLoadPos_2*1e-3,'b--',...
    omegaPosLoad_2/2/pi,stiffnessLine_Nmm,'k--',...
    natFreqLine,verticalLine,'k--','MarkerSize',10);
%     natFreq*sqrt(1-2*dampRatio_1^2),minImpedance_1*1e-3,'rd',...
%     natFreq*sqrt(1-2*dampRatio_2^2),minImpedance_2*1e-3,'bo',...

grid on
ax = gca;
ax.FontSize = 14;
xlabel('Frequency (Hz)','Fontsize',16)
ylabel('Stiffness (N/mm)','Fontsize',16)
legend({'Damping = 0.4','Damping = 0.1','Stiffness = 1.98 e5','Natural frequency = 20.45'},...
    'Fontsize',14,'Location','northeast')
min(magLoadPos_2*1e-3)

% save('freqLoadPos_1.mat','freqLoadPos_1')
% save('magLoadPos_1.mat','magLoadPos_1')
% save('freqLoadPos_2.mat','freqLoadPos_2')
% save('magLoadPos_2.mat','magLoadPos_2')
% csvwrite('freqLoadPos_1.txt',freqLoadPos_1);
% csvwrite('freqLoadPos_2.txt',freqLoadPos_2);
% csvwrite('magLoadPos_1.txt',magLoadPos_1);
% csvwrite('magLoadPos_2.txt',magLoadPos_2);



