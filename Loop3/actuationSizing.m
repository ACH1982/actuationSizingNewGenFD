% Program for estimating the main characteristics of the actuation system
clear all
%--------------------------------------------------------------------------
% Input data and parameters------------------------------------------------
load('actuationInputDataList.mat')
% Conversion---------------------------------------------------------------
load('actuationUnitsConversion.mat')
%--------------------------------------------------------------------------
% Parameters for computation ----------------------------------------------
hingeRatio = Hm2 / Hm1;
loadFlowMargin = 1.0; % QNL = loadFlowMargin*QNL
textCriteria = strcat(' SizingCriteria=',num2str(round(sizingCriteria,2)),...
    ',',' PressureSupply= ',num2str(PS_psi),' psi');
%--------------------------------------------------------------------------
% Estimation---------------------------------------------------------------
% Arm / area / stroke -----------------------------------------------------
PS_Pa = PS_psi * psi2Pa;
armArea = (Hm1/PS_Pa) / sizingCriteria; % m^3
area = armArea / arm; % m^2
stroke_m = deltaRange_deg * deg2rad * arm; % m
stroke_mm = stroke_m * 1e3;
totalVol = volumeRatio * area * deltaRange_deg * deg2rad * arm;
% Valve size --------------------------------------------------------------
dotdelta1_rad = dotdelta1_deg * deg2rad; 
dotdelta2_rad = dotdelta2_deg * deg2rad;
QNL1_m3s = loadFlowMargin * armArea * dotdelta1_rad / sqrt(1 - sizingCriteria);
QNL2_m3s = loadFlowMargin * armArea * dotdelta2_rad / sqrt(1 - sizingCriteria*hingeRatio);
QNL_m3s = max(QNL1_m3s,QNL2_m3s);
QNL_lpm = QNL_m3s * m3s2lpm;
if QNL1_m3s > QNL2_m3s
    sizingCase = 0;
else
    sizingCase = 1;
end
% Pressure-flow curve------------------------------------------------------
PL_psi = 0:10:PS_psi;
rateLimit_rad = rateLimit_deg * pi/180;
QL_rateLimit = loadFlowMargin*rateLimit_rad*armArea;
QL_m3s = QNL_m3s * sqrt(1 - PL_psi./PS_psi);
QL_m3s_RL = min(QL_rateLimit,QNL_m3s * sqrt(1 - PL_psi/PS_psi));
QL_lpm = QL_m3s * m3s2lpm;
QL_lpm_RL = QL_m3s_RL * m3s2lpm;
% Load - velocity curve
velocityNL_ms = QNL_m3s / (loadFlowMargin * area);
velocityNL_mms = velocityNL_ms*1e3;
PL_Pa = PL_psi * psi2Pa;
load_N = PL_Pa * area;
velocity_ms = (QL_m3s / loadFlowMargin) / area; 
velocity_mms = 1e3 * velocity_ms; 
% Hinge-rate curve---------------------------------------------------------
Hm = PL_Pa * armArea;
dotdelta_rad = (QL_m3s / loadFlowMargin) / armArea; 
dotdelta_rad_RL = (QL_m3s_RL / loadFlowMargin) / armArea; 
dotdelta_deg = dotdelta_rad * rad2deg; 
dotdelta_deg_RL = dotdelta_rad_RL * rad2deg; 
% Power estimation---------------------------------------------------------
developedPower_W = QL_m3s .* PL_Pa;
developedPower_W_RL = QL_m3s_RL .* PL_Pa;
consumedPower_W = QL_m3s .* PS_Pa;
consumedPower_W_RL = QL_m3s_RL .* PS_Pa;
% Stiffness----------------------------------------------------------------
% Bulk_Pa = Bulk_psi * psi2Pa;
% deltaRange_rad = deltaRange_deg * pi/180;
% surfaceStiffness = (4 * Bulk_Pa ./ totalVol) * armArea^2;
%--------------------------------------------------------------------------
% Saved output to .mat ----------------------------------------------------
save('actuationSizingData.mat',...
    'area','arm','QNL_m3s','totalVol');
% -------------------------------------------------------------------------
% Plot results-------------------------------------------------------------
hingeSizingPoint_FlowPressure = [sizingCriteria*PS_psi,QNL_lpm*sqrt(1-sizingCriteria)];
hingeSizingPoint_RateHinge = [Hm1,(QNL_m3s*sqrt(1-sizingCriteria)/(armArea*loadFlowMargin))*rad2deg];
PowerSizingPoint = [sizingCriteria*PS_psi,QNL_m3s*sqrt(1-sizingCriteria)*sizingCriteria*PS_psi*psi2Pa];
if sizingCase == 0
    rateSizingPoint_FlowPressure = [Hm1/armArea*Pa2psi,dotdelta1_rad*armArea*m3s2lpm*loadFlowMargin];
    rateSizingPoint_RateHinge = [Hm1,(QNL_m3s*sqrt(1-sizingCriteria)/ (armArea*loadFlowMargin))*rad2deg];
else
    rateSizingPoint_FlowPressure = [Hm2/armArea*Pa2psi,dotdelta2_rad*armArea*m3s2lpm*loadFlowMargin];
    rateSizingPoint_RateHinge = [Hm2,dotdelta2_deg];
end
figure %-------------------------------------------------------------------
subplot(3,2,1)
plot(PL_psi,QL_lpm,'--',PL_psi,QL_lpm_RL,...
    rateSizingPoint_FlowPressure(1),rateSizingPoint_FlowPressure(2),'k*',...
    hingeSizingPoint_FlowPressure(1),hingeSizingPoint_FlowPressure(2),'ko',...
    'MarkerSize',10),grid
title(strcat('Flow-pressure curve,',textCriteria))
legend('No rate limit','Rate limit applied','Valve sizing point',...
    'Cylinder sizing point','Location','southwest')
xlabel('Pressure [psi]')
ylabel('Load-flow [lpm]')

subplot(3,2,3)
plot((Hm/arm)*1e-3,dotdelta_deg*deg2rad*arm*1e3,'--',(Hm/arm)*1e-3,dotdelta_deg_RL*deg2rad*arm*1e3,...
    (rateSizingPoint_RateHinge(1)/arm)*1e-3,rateSizingPoint_RateHinge(2)*deg2rad*arm*1e3,'k*',...
    (hingeSizingPoint_RateHinge(1)/arm)*1e-3,hingeSizingPoint_RateHinge(2)*deg2rad*arm*1e3,'ko','MarkerSize',10),grid
title(strcat('Velocity-force curve,',textCriteria))
legend('No rate limit','Rate limit applied','Valve sizing point',...
    'Cylinder sizing point','Location','southwest')
xlabel('Force [KN]')
ylabel('Velocity [mm/s]')

subplot(3,2,5)
plot(Hm*1e-3,dotdelta_deg,'--',Hm*1e-3,dotdelta_deg_RL,...
    rateSizingPoint_RateHinge(1)*1e-3,rateSizingPoint_RateHinge(2),'k*',...
    hingeSizingPoint_RateHinge(1)*1e-3,hingeSizingPoint_RateHinge(2),'ko','MarkerSize',10),grid
title(strcat('Rate-hinge curve,',textCriteria))
legend('No rate limit','Rate limit applied','Valve sizing point',...
    'Cylinder sizing point','Location','southwest')
xlabel('Hinge moment [KN*m]')
ylabel('Surface rate [deg/s]')

subplot(3,2,[2,6])
plot(PL_psi,developedPower_W*1e-3,'--',PL_psi,developedPower_W_RL*1e-3,...
    PL_psi,consumedPower_W*1e-3,'--',PL_psi,consumedPower_W_RL*1e-3,...
    PowerSizingPoint(1),PowerSizingPoint(2)*1e-3,'kd','MarkerSize',10),grid
title(strcat('Developed and consumed Power,',textCriteria))
legend('Developed no rate limit','Developed rate limit applied',...
    'Consumed no rate limit','Consumed rate limit applied','Sizing point',...
    'Location','northeast')
xlabel('Pressure [psi]')
ylabel('Power [KW]')


