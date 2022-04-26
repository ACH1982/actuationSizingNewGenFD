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
area_1 = area;
% area_2 = 2.75*area_1;
pressGain_MPa_mA = 1.3*(PS_psi*psi2Pa*1e-6)/0.05;
pressGain_Pa_mA = 1.3*(PS_psi*psi2Pa)/0.05; 
flowGain_lpm_mA = QNL_m3s*m3s2lpm;
flowGain_m3s_mA = flowGain_lpm_mA * lpm2m3s;
flowPressCoeff_m3sPa = flowGain_m3s_mA / pressGain_Pa_mA;
totalFlowPressCoeff_m3sPa = flowPressCoeff_m3sPa + ...
    (internalLeak_lpm * lpm2m3s) / (PS_psi * psi2Pa);
totalVol_1 = deltaRange_deg*deg2rad * arm * area_1 * volumeRatio;
% totalVol_2 = deltaRange_deg*deg2rad * arm * area_2 * volumeRatio;
eqMass = inertia_kgm2 / (arm^2);
Bulk_Pa = Bulk_psi*psi2Pa;
stiffness_1 = (4*Bulk_Pa*(area_1)^2) / totalVol_1;
% stiffness_2 = (4*Bulk_Pa*(area_2)^2) / totalVol_2;
% -------------------------------------------------------------------------
% Estimated parameters - second level--------------------------------------
natOmega_1 = sqrt(stiffness_1 / eqMass);
% natOmega_2 = sqrt(stiffness_2 / eqMass);
natFreq_1 = natOmega_1 / (2*pi);
% natFreq_2 = natOmega_2 / (2*pi);
dampRatioRaw = (totalFlowPressCoeff_m3sPa/area)*sqrt(eqMass*Bulk_Pa/totalVol_1);
dampRatio_1 = dampRatioRaw + 0.3;
% dampRatio_2 = dampRatioRaw + 0.3;
% the totalFlowPressCoeff should be redefined if the dampRatio is
% established in a specific value: dampRatio based on totalFlowPressCoeff +
% 0.2 --> totalFlowPressCoeff should be computed from this dampRatio
totalFlowPressCoeff_1_m3sPa = dampRatio_1*area_1/sqrt(Bulk_Pa*eqMass/totalVol_1);
% totalFlowPressCoeff_2_m3sPa = dampRatio_2*area_2/sqrt(Bulk_Pa*eqMass/totalVol_2);
% -------------------------------------------------------------------------
% Transfer functions estimation -------------------------------------------
denPosCurrent_1 = [1/natOmega_1^2 2*dampRatio_1/natOmega_1 1 0]; 
% denPosCurrent_2 = [1/natOmega_2^2 2*dampRatio_2/natOmega_2 1 0]; 
denPosLoad_1 = [1/natOmega_1^2 2*dampRatio_1/natOmega_1 1 0]; 
% denPosLoad_2 = [1/natOmega_2^2 2*dampRatio_2/natOmega_2 1 0]; 
% Numerator, gain and TF for (position / valvePosition) transfer
gainPosCurrent_1 = [flowGain_m3s_mA / area_1];
% gainPosCurrent_2 = [flowGain_m3s_mA / area_2];
numPosCurrent_1 = [1*gainPosCurrent_1];
% numPosCurrent_2 = [1*gainPosCurrent_2];
posCurrent_TF_1 = tf(numPosCurrent_1,denPosCurrent_1);
% posCurrent_TF_2 = tf(numPosCurrent_2,denPosCurrent_2);
% Numerator, gain and TF for (position / load) transfer
gainPosLoad_1 = -totalFlowPressCoeff_1_m3sPa / area_1^2; 
% gainPosLoad_2 = -totalFlowPressCoeff_2_m3sPa / area_2^2; 
numPosLoad_1 = [1*gainPosLoad_1/(2*dampRatio_1*natOmega_1) 1*gainPosLoad_1];
% numPosLoad_2 = [1*gainPosLoad_2/(2*dampRatio_2*natOmega_2) 1*gainPosLoad_2];
posLoad_TF_1 = tf(numPosLoad_1,denPosLoad_1);
% posLoad_TF_2 = tf(numPosLoad_1,denPosLoad_2);
% -------------------------------------------------------------------------
% Plot---------------------------------------------------------------------
freq_init = 1; % Hz
freq_end = 40; % Hz
freq_step = 0.4;
w_init = freq_init*2*pi; % rad/s
w_end = freq_end*2*pi; % rad/s
w_step = freq_step*2*pi;
w_range = w_init:w_step:w_end;
[magPosCurrentRaw_1,phasePosCurrentRaw_1,omegaPosCurrent_1] = ...
    bode(posCurrent_TF_1,w_range);
% [magPosCurrentRaw_2,phasePosCurrentRaw_2,omegaPosCurrent_2] = ...
%     bode(posCurrent_TF_2,w_range);
for i=1:length(omegaPosCurrent_1)
    magPosCurrent_1(i) = magPosCurrentRaw_1(1,1,i);
    phasePosCurrent_1(i) = phasePosCurrentRaw_1(1,1,i);
    
%    magPosCurrent_2(i) = magPosCurrentRaw_2(1,1,i);
%    phasePosCurrent_2(i) = phasePosCurrentRaw_2(1,1,i);
end
[magPosLoadRaw_1,phasePosLoadRaw_1,omegaPosLoad_1] = bode(posLoad_TF_1,w_range);
%[magPosLoadRaw_2,phasePosLoadRaw_2,omegaPosLoad_2] = bode(posLoad_TF_2,w_range);
for j=1:length(omegaPosLoad_1)
    magLoadPos_1(j) = 1/magPosLoadRaw_1(1,1,j);
    phaseLoadPos_1(j) = phasePosLoadRaw_1(1,1,j);
    
%    magLoadPos_2(j) = 1/magPosLoadRaw_2(1,1,j);
%    phaseLoadPos_2(j) = phasePosLoadRaw_2(1,1,j);
end
%-------------------------------------------------------------------
vertLine_y_1 = 0:1e1:stiffness_1*1e-3;
% vertLine_y_2 = 0:1e1:stiffness_2*1e-3;
vertLine_x_1 = natFreq_1*ones(length(vertLine_y_1),1);
% vertLine_x_2 = natFreq_2*ones(length(vertLine_y_2),1);
horLine_x_1 = 1:0.1:natFreq_1;
% horLine_x_2 = 1:0.1:natFreq_2;
horLine_y_1 = (stiffness_1*1e-3)*ones(length(horLine_x_1),1);
% horLine_y_2 = (stiffness_2*1e-3)*ones(length(horLine_x_2),1);
% -------------------------------------------------------------------------
% Plot results ------------------------------------------------------------
% -------------------------------------------------------------------------
% semilogx(omegaPosLoad_1/2/pi,magLoadPos_1*1e-3,...
%     omegaPosLoad_2/2/pi,magLoadPos_2*1e-3,...
%     natFreq_1,stiffness_1*1e-3,'k*',...
%     natFreq_2,stiffness_2*1e-3,'ko',...
%     vertLine_x_1,vertLine_y_1,'k--',...
%     horLine_x_1,horLine_y_1,'k--',...
%     vertLine_x_2,vertLine_y_2,'k--',...
%     horLine_x_2,horLine_y_2,'k--','MarkerSize',10),grid
% grid on
% xlabel('Frequency (Hz)')
% ylabel('Stiffness (N/mm)')
% text = strcat('Stiffness = ',num2str(stiffness_1*1e-3),' N/mm');
% legend('Hinge/Rate reqs',...
%     'Resized owing to stiffness',...
%     'Stiffness at natural frequency (hinge/rate reqs)',...
%     'Resized stiffness at natural frequency','Location','north')
semilogx(omegaPosLoad_1/2/pi,magLoadPos_1*1e-3,...
    natFreq_1,stiffness_1*1e-3,'k*',...
    vertLine_x_1,vertLine_y_1,'k--',...
    horLine_x_1,horLine_y_1,'k--',...
    'MarkerSize',10),grid
grid on
xlabel('Frequency (Hz)')
ylabel('Stiffness (N/mm)')
text = strcat('Stiffness = ',num2str(stiffness_1*1e-3),' N/mm');
legend('Hinge/Rate reqs',...
    'Stiffness at natural frequency (hinge/rate reqs)',...
    'Location','north')