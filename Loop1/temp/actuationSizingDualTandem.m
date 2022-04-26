% Program for estimating the main characteristics of the actuation system
clear all
%--------------------------------------------------------------------------
% Input data and parameters------------------------------------------------
load('actuationInputDataListDualTandem.mat')
% Conversion---------------------------------------------------------------
load('actuationUnitsConversion.mat')
%--------------------------------------------------------------------------
% Parameters for computation ----------------------------------------------
hingeRatio = Hm2 / Hm1;
loadFlowMargin = 1.0; % QNL = loadFlowMargin*QNL
sizingCriteria = 4/5; % Max hinge respect stall hinge
textCriteria = strcat(' SizingCriteria=',num2str(round(sizingCriteria,2)),...
    ',',' PressureSupply= ',num2str(PS_psi),' psi');
%--------------------------------------------------------------------------
% Estimation---------------------------------------------------------------
% Arm / area / stroke -----------------------------------------------------
PS_Pa = PS_psi * psi2Pa;
armArea_1 = (Hm1./PS_Pa) / sizingCriteria; % m^3
area_1 = armArea_1 ./ arm; % m^2
armArea_2 = (Hm2./PS_Pa) / sizingCriteria; % m^3
area_2 = armArea_2 ./ arm; % m^2
stroke_m = deltaRange_deg * deg2rad * arm; % m
stroke_mm = stroke_m * 1e3;
% Valve size --------------------------------------------------------------
dotdelta1_rad = dotdelta1_deg * deg2rad; 
dotdelta2_rad = dotdelta2_deg * deg2rad;
rateLimit1_deg = rateLimit_deg * (area_1/area_2)*(area_2/area_1);
rateLimit2_deg = rateLimit_deg;
rateLimit1_rad = rateLimit1_deg * pi/180;
rateLimit2_rad = rateLimit2_deg * pi/180;
QNL1_m3s = loadFlowMargin * armArea_1 * dotdelta1_rad / sqrt(1 - sizingCriteria);
QNL2_m3s = loadFlowMargin * armArea_2 * dotdelta2_rad / sqrt(1 - sizingCriteria);
QNL1_m3s_RL = armArea_1 * rateLimit1_deg * deg2rad;
QNL2_m3s_RL = armArea_2 * rateLimit2_deg * deg2rad;
QNL1_lpm_RL = QNL1_m3s_RL * m3s2lpm;
QNL2_lpm_RL = QNL2_m3s_RL * m3s2lpm;
% Pressure-flow curve------------------------------------------------------
PL1_psi = 0:10:PS_psi;
PL2_psi = 0:10:(Hm2/(armArea_2*sizingCriteria))*Pa2psi;
QL1_rateLimit = loadFlowMargin*rateLimit1_rad*armArea_1;
QL2_rateLimit = loadFlowMargin*rateLimit2_rad*armArea_2;
QL1_m3s = QNL1_m3s * sqrt(1 - PL1_psi./PS_psi);
QL2_m3s = QNL2_m3s * sqrt(1 - PL2_psi./PS_psi);
QL1_m3s_RL = min(QL1_rateLimit,QNL1_m3s * sqrt(1 - PL1_psi/PS_psi));
QL2_m3s_RL = min(QL2_rateLimit,QNL2_m3s * sqrt(1 - PL2_psi/PS_psi));
QL1_lpm = QL1_m3s * m3s2lpm;
QL2_lpm = QL2_m3s * m3s2lpm;
QL1_lpm_RL = QL1_m3s_RL * m3s2lpm;
QL2_lpm_RL = QL2_m3s_RL * m3s2lpm;
% Load - velocity curve
velocityNL1_ms = QNL1_m3s / (loadFlowMargin * area_1);
velocityNL2_ms = QNL2_m3s / (loadFlowMargin * area_2);
velocityNL1_mms = velocityNL1_ms*1e3;
velocityNL2_mms = velocityNL2_ms*1e3;
PL1_Pa = PL1_psi * psi2Pa;
PL2_Pa = PL2_psi * psi2Pa;
load1_KN = PL1_Pa * area_1 * 1e-3;
load2_KN = PL2_Pa * area_2 * 1e-3;
velocity1_ms = (QL1_m3s / loadFlowMargin) / area_1;
velocity2_ms = (QL2_m3s / loadFlowMargin) / area_2;
velocity1_ms_RL = (QL1_m3s_RL / loadFlowMargin) / area_1;
velocity2_ms_RL = (QL2_m3s_RL / loadFlowMargin) / area_2;
velocity1_mms = 1e3 * velocity1_ms; 
velocity2_mms = 1e3 * velocity2_ms;
velocity1_mms_RL = 1e3 * velocity1_ms_RL;
velocity2_mms_RL = 1e3 * velocity2_ms_RL;
% Hinge-rate curve---------------------------------------------------------
Hinge1_KNm = PL1_Pa * armArea_1 * 1e-3;
Hinge2_KNm = PL2_Pa * armArea_2 * 1e-3;
ddelta1_rad = (QL1_m3s / loadFlowMargin) / armArea_1; 
ddelta2_rad = (QL2_m3s / loadFlowMargin) / armArea_2; 
ddelta1_rad_RL = (QL1_m3s_RL / loadFlowMargin) / armArea_1; 
ddelta2_rad_RL = (QL2_m3s_RL / loadFlowMargin) / armArea_2; 
ddelta1_deg = ddelta1_rad * rad2deg; 
ddelta2_deg = ddelta2_rad * rad2deg; 
ddelta1_deg_RL = ddelta1_rad_RL * rad2deg; 
ddelta2_deg_RL = ddelta2_rad_RL * rad2deg; 



% -------------------------------------------------------------------------
% Plot data ---------------------------------------------------------------
% -------------------------------------------------------------------------
subplot(3,1,1)
plot(PL1_psi,QL1_lpm_RL,PL2_psi,QL2_lpm_RL),grid
xlabel('Pressure (psi)')
ylabel('Flow (lpm)')
legend('high speed','low speed','Location','southeast')
subplot(3,1,2)
plot(load1_KN,velocity1_mms_RL,load2_KN,velocity2_mms_RL),grid
xlabel('Load (KN)')
ylabel('Velocity (mm/s)')
legend('high speed','low speed','Location','northeast')
subplot(3,1,3)
plot(Hinge1_KNm,ddelta1_deg_RL,Hinge2_KNm,ddelta2_deg_RL),grid
xlabel('Hinge moment (KNm)')
ylabel('Rate (deg/s)')
legend('high speed','low speed','Location','northeast')




% % Power estimation---------------------------------------------------------
% developedPower_W = QL_m3s .* PL_Pa;
% developedPower_W_RL = QL_m3s_RL .* PL_Pa;
% consumedPower_W = QL_m3s .* PS_Pa;
% consumedPower_W_RL = QL_m3s_RL .* PS_Pa;
% % Stiffness----------------------------------------------------------------
% % Bulk_Pa = Bulk_psi * psi2Pa;
% % deltaRange_rad = deltaRange_deg * pi/180;
% % totalVol = volumeRatio * area * deltaRange_rad .* arm;
% % surfaceStiffness = (4 * Bulk_Pa ./ totalVol) * armArea^2;
% %--------------------------------------------------------------------------
% % Saved output to .mat ----------------------------------------------------
% a = 1;
% b = 1; 
% save('actuationSizingData.mat',...
%     'area',...
%     'arm',...
%     'QNL_m3s');
% % -------------------------------------------------------------------------
% % Plot results-------------------------------------------------------------
% hingeSizingPoint_FlowPressure = [sizingCriteria*PS_psi,QNL_lpm*sqrt(1-sizingCriteria)];
% hingeSizingPoint_RateHinge = [Hm1,(QNL_m3s*sqrt(1-sizingCriteria)/(armArea*loadFlowMargin))*rad2deg];
% PowerSizingPoint = [sizingCriteria*PS_psi,QNL_m3s*sqrt(1-sizingCriteria)*sizingCriteria*PS_psi*psi2Pa];
% if sizingCase == 0
%     rateSizingPoint_FlowPressure = [Hm1/armArea*Pa2psi,dotdelta1_rad*armArea*m3s2lpm*loadFlowMargin];
%     rateSizingPoint_RateHinge = [Hm1,(QNL_m3s*sqrt(1-sizingCriteria)/ (armArea*loadFlowMargin))*rad2deg];
% else
%     rateSizingPoint_FlowPressure = [Hm2/armArea*Pa2psi,dotdelta2_rad*armArea*m3s2lpm*loadFlowMargin];
%     rateSizingPoint_RateHinge = [Hm2,dotdelta2_deg];
% end
% figure %-------------------------------------------------------------------
% subplot(3,2,1)
% plot(PL_psi,QL_lpm,'--',PL_psi,QL_lpm_RL,...
%     rateSizingPoint_FlowPressure(1),rateSizingPoint_FlowPressure(2),'k*',...
%     hingeSizingPoint_FlowPressure(1),hingeSizingPoint_FlowPressure(2),'ko',...
%     'MarkerSize',10),grid
% title(strcat('Flow-pressure curve,',textCriteria))
% legend('No rate limit','Rate limit applied','Valve sizing point',...
%     'Cylinder sizing point','Location','southwest')
% xlabel('Pressure [psi]')
% ylabel('Load-flow [lpm]')
% 
% subplot(3,2,3)
% plot((Hm/arm)*1e-3,dotdelta_deg*deg2rad*arm*1e3,'--',(Hm/arm)*1e-3,dotdelta_deg_RL*deg2rad*arm*1e3,...
%     (rateSizingPoint_RateHinge(1)/arm)*1e-3,rateSizingPoint_RateHinge(2)*deg2rad*arm*1e3,'k*',...
%     (hingeSizingPoint_RateHinge(1)/arm)*1e-3,hingeSizingPoint_RateHinge(2)*deg2rad*arm*1e3,'ko','MarkerSize',10),grid
% title(strcat('Velocity-force curve,',textCriteria))
% legend('No rate limit','Rate limit applied','Valve sizing point',...
%     'Cylinder sizing point','Location','southwest')
% xlabel('Force [KN]')
% ylabel('Velocity [mm/s]')
% 
% subplot(3,2,5)
% plot(Hm*1e-3,dotdelta_deg,'--',Hm*1e-3,dotdelta_deg_RL,...
%     rateSizingPoint_RateHinge(1)*1e-3,rateSizingPoint_RateHinge(2),'k*',...
%     hingeSizingPoint_RateHinge(1)*1e-3,hingeSizingPoint_RateHinge(2),'ko','MarkerSize',10),grid
% title(strcat('Rate-hinge curve,',textCriteria))
% legend('No rate limit','Rate limit applied','Valve sizing point',...
%     'Cylinder sizing point','Location','southwest')
% xlabel('Hinge moment [KN*m]')
% ylabel('Surface rate [deg/s]')
% 
% subplot(3,2,[2,6])
% plot(PL_psi,developedPower_W*1e-3,'--',PL_psi,developedPower_W_RL*1e-3,...
%     PL_psi,consumedPower_W*1e-3,'--',PL_psi,consumedPower_W_RL*1e-3,...
%     PowerSizingPoint(1),PowerSizingPoint(2)*1e-3,'kd','MarkerSize',10),grid
% title(strcat('Developed and consumed Power,',textCriteria))
% legend('Developed no rate limit','Developed rate limit applied',...
%     'Consumed no rate limit','Consumed rate limit applied','Sizing point',...
%     'Location','northeast')
% xlabel('Pressure [psi]')
% ylabel('Power [KW]')