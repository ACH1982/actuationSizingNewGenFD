% Program for estimating the main characteristics of the actuation system
clear all
%--------------------------------------------------------------------------
% Input data and parameters------------------------------------------------
load('actuationInputDataListDualTandem.mat')
% Conversion---------------------------------------------------------------
load('actuationUnitsConversion.mat')
%--------------------------------------------------------------------------
% Parameters for computation ----------------------------------------------
hingeRatioConfB = Hm1 / Hm0;
loadFlowMargin = 1.0; % QNL = loadFlowMargin*QNL
%--------------------------------------------------------------------------
% Cylinder and valve sizing
%--------------------------------------------------------------------------
% Arm / area / stroke -----------------------------------------------------
PS_Pa = PS_psi * psi2Pa;
% Configuration A (low speed configuration)
armArea_ConfA = (Hm2/PS_Pa) / sizingCriteria_ConfA; % m^3, hinge moment req
area_ConfA = armArea_ConfA / arm; % m^2
totalVol_ConfA = volumeRatio * armArea_ConfA * deltaRange_deg * deg2rad;
% Configuration B (high speed configuration)
armArea_ConfB_stiffReq = (Hm0/PS_Pa); % m^3, stiffness req
armArea_ConfB_hingeReq = (Hm1/PS_Pa) / sizingCriteria_ConfB; % m^3, hinge moment req
armArea_ConfB = max(armArea_ConfB_stiffReq,armArea_ConfB_hingeReq);
area_ConfB = armArea_ConfB / arm; % m^2
totalVol_ConfB = volumeRatio * armArea_ConfB * deltaRange_deg * deg2rad;
% Common to both configurations
stroke_m = deltaRange_deg * deg2rad * arm; % m
stroke_mm = stroke_m * 1e3;
%--------------------------------------------------------------------------
% Valve size --------------------------------------------------------------
dotdelta1_rad = dotdelta1_deg * deg2rad; 
dotdelta2_rad = dotdelta2_deg * deg2rad;
QNL_ConfA_m3s = loadFlowMargin * armArea_ConfA * ...
    dotdelta2_rad / sqrt(1 - sizingCriteria_ConfA);
QNL_ConfB_m3s = loadFlowMargin * armArea_ConfB * ...
    dotdelta1_rad / sqrt(1 - sizingCriteria_ConfB);
QNL_ConfA_lpm = QNL_ConfA_m3s * m3s2lpm;
QNL_ConfB_lpm = QNL_ConfB_m3s * m3s2lpm;
%--------------------------------------------------------------------------
% Pressure-flow curves
%--------------------------------------------------------------------------
PL_psi = 0:10:PS_psi;
% Configuration A
rateLimit_ConfA_rad = rateLimit_ConfA_deg * pi/180;
QL_rateLimit_ConfA = loadFlowMargin*rateLimit_ConfA_rad*armArea_ConfA;
QL_ConfA_m3s = QNL_ConfA_m3s * sqrt(1 - PL_psi./PS_psi);
QL_ConfA_m3s_RL = min(QL_rateLimit_ConfA,QNL_ConfA_m3s * sqrt(1 - PL_psi/PS_psi));

QL_ConfA_lpm = QL_ConfA_m3s * m3s2lpm;
QL_ConfA_lpm_RL = QL_ConfA_m3s_RL * m3s2lpm;
% Configuration B
rateLimit_ConfB_rad = rateLimit_ConfB_deg * pi/180;
QL_rateLimit_ConfB = loadFlowMargin*rateLimit_ConfB_rad*armArea_ConfB;
QL_ConfB_m3s = QNL_ConfB_m3s * sqrt(1 - PL_psi./PS_psi);
QL_ConfB_m3s_RL = min(QL_rateLimit_ConfB,QNL_ConfB_m3s * sqrt(1 - PL_psi/PS_psi));

QL_ConfB_lpm = QL_ConfB_m3s * m3s2lpm;
QL_ConfB_lpm_RL = QL_ConfB_m3s_RL * m3s2lpm;
%--------------------------------------------------------------------------
% Load-velocity curves
%--------------------------------------------------------------------------
% Configuration A
load_ConfA_N = PL_psi*psi2Pa * area_ConfA;
velocity_ConfA_ms = QL_ConfA_m3s / area_ConfA;
velocity_ConfA_ms_RL = QL_ConfA_m3s_RL / area_ConfA;

load_ConfA_KN = load_ConfA_N * 1e-3;
velocity_ConfA_mms = velocity_ConfA_ms * 1e3;
velocity_ConfA_mms_RL = velocity_ConfA_ms_RL * 1e3;
% Configuration B
load_ConfB_N = PL_psi*psi2Pa * area_ConfB;
velocity_ConfB_ms = QL_ConfB_m3s / area_ConfB;
velocity_ConfB_ms_RL = QL_ConfB_m3s_RL / area_ConfB;

load_ConfB_KN = load_ConfB_N * 1e-3;
velocity_ConfB_mms = velocity_ConfB_ms * 1e3;
velocity_ConfB_mms_RL = velocity_ConfB_ms_RL * 1e3;
%--------------------------------------------------------------------------
% Hinge-rate curves
%--------------------------------------------------------------------------
% Configuration A
Hinge_ConfA_Nm = load_ConfA_N * arm;
dotdelta_ConfA_rad = velocity_ConfA_ms / arm;
dotdelta_ConfA_rad_RL = velocity_ConfA_ms_RL / arm;

Hinge_ConfA_KNm = Hinge_ConfA_Nm * 1e-3;
dotdelta_ConfA_deg = dotdelta_ConfA_rad * rad2deg;
dotdelta_ConfA_deg_RL = dotdelta_ConfA_rad_RL * rad2deg;
% Configuration b
Hinge_ConfB_Nm = load_ConfB_N * arm;
dotdelta_ConfB_rad = velocity_ConfB_ms / arm;
dotdelta_ConfB_rad_RL = velocity_ConfB_ms_RL / arm;

Hinge_ConfB_KNm = Hinge_ConfB_Nm * 1e-3;
dotdelta_ConfB_deg = dotdelta_ConfB_rad * rad2deg;
dotdelta_ConfB_deg_RL = dotdelta_ConfB_rad_RL * rad2deg;

%--------------------------------------------------------------------------
% Save
%--------------------------------------------------------------------------
save('actuationSizingDataDualTandem.mat',...
    'area_ConfA','area_ConfB',...
    'QNL_ConfA_m3s','QNL_ConfB_m3s',...
    'QL_rateLimit_ConfA','QL_rateLimit_ConfB',...
    'totalVol_ConfA','totalVol_ConfB')
%--------------------------------------------------------------------------
% Plot
%--------------------------------------------------------------------------
subplot(2,2,[1,2])
plot(PL_psi,QL_ConfA_lpm,'r--',...
    PL_psi,QL_ConfA_lpm_RL,'b',...
    PL_psi,QL_ConfB_lpm_RL,'g'),grid
xlabel('Pressure (psi)')
ylabel('Flow (lpm)')
legend('Low Speed','Low speed (RL = 80 deg/s)','High Speed')

subplot(2,2,3)
plot(load_ConfA_KN,velocity_ConfA_mms,'r--',...
    load_ConfA_KN,velocity_ConfA_mms_RL,'b',...
    load_ConfB_KN,velocity_ConfB_mms_RL,'g'),grid
xlabel('Load (KN)')
ylabel('Velocity (mms)')
legend('Low Speed','Low speed (RL = 80 deg/s)','High Speed')

subplot(2,2,4)
plot(Hinge_ConfA_KNm,dotdelta_ConfA_deg,'r--',...
    Hinge_ConfA_KNm,dotdelta_ConfA_deg_RL,'b',...
    Hinge_ConfB_KNm,dotdelta_ConfB_deg_RL,'g'),grid
xlabel('Hinge moment (KN*m)')
ylabel('Surface rate (deg/s)')
legend('Low Speed','Low speed (RL = 80 deg/s)','High Speed')

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


