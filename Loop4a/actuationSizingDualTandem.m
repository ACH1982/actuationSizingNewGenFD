% Program for estimating the main characteristics of the actuation system
% Configuration A means: all area is used for acting
% Configuration B means: only low airspeed area is used for acting
clear all
%--------------------------------------------------------------------------
% Input data and parameters------------------------------------------------
load('actuationInputDataListDualTandem.mat')
% Conversion---------------------------------------------------------------
load('actuationUnitsConversion.mat')
%--------------------------------------------------------------------------
% Parameters for computation ----------------------------------------------
hingeRatioConfB = Hm1 / Hm0;
%--------------------------------------------------------------------------
% Cylinder and valve sizing
%--------------------------------------------------------------------------
% Arm / area / stroke -----------------------------------------------------
PS_Pa = PS_psi * psi2Pa;
% Configuration A (high airspeed configuration)
A_armArea_hingeReq = (Hm1/PS_Pa) / A_sizingCriteria; % m^3, hinge moment req
A_armArea_stiffReq = (Hm0/PS_Pa); % m^3, stiffness req
A_armArea = max(A_armArea_stiffReq,A_armArea_hingeReq);
A_area = A_armArea / arm; % m^2
A_totalVol = volumeRatio * A_armArea * deltaRange_deg * deg2rad;
% Configuration B (low speed configuration)
B_armArea = (Hm2/PS_Pa) / B_sizingCriteria; % m^3, hinge moment req
B_area = B_armArea / arm; % m^2
B_totalVol = volumeRatio * B_armArea * deltaRange_deg * deg2rad;
% Common to both configurations
stroke_m = deltaRange_deg * deg2rad * arm; % m
    stroke_mm = stroke_m * 1e3;
%--------------------------------------------------------------------------
% Valve size --------------------------------------------------------------
    dotdelta1_rad = dotdelta1_deg * deg2rad; 
    dotdelta2_rad = dotdelta2_deg * deg2rad;
% Configuration A (high airspeed configuration)
A_QNL_m3s = loadFlowMargin * A_armArea * ...
    dotdelta1_rad / sqrt(1 - A_sizingCriteria);
    A_QNL_lpm = A_QNL_m3s * m3s2lpm;
% Configuration B (low speed configuration)
B_QNL_m3s = loadFlowMargin * B_armArea * ...
    dotdelta2_rad / sqrt(1 - B_sizingCriteria);
    B_QNL_lpm = B_QNL_m3s * m3s2lpm;
%--------------------------------------------------------------------------
% Pressure-flow curves
%--------------------------------------------------------------------------
PL_psi = 0:10:PS_psi;
% Configuration A (high airspeed configuration)
    A_rateLimit_rad = A_rateLimit_deg * deg2rad;
A_QL_rateLimit = loadFlowMargin * A_rateLimit_rad * A_armArea;

A_QL_m3s = A_QNL_m3s * sqrt(1 - PL_psi./PS_psi);
A_QL_m3s_RL = min(A_QL_rateLimit,A_QNL_m3s * sqrt(1 - PL_psi/PS_psi));
    A_QL_lpm = A_QL_m3s * m3s2lpm;
    A_QL_lpm_RL = A_QL_m3s_RL * m3s2lpm;
% Configuration B (low speed configuration)
B_rateLimit_rad = B_rateLimit_deg * deg2rad;
B_QL_rateLimit = loadFlowMargin * B_rateLimit_rad * B_armArea;
B_QL_m3s = B_QNL_m3s * sqrt(1 - PL_psi./PS_psi);
B_QL_m3s_RL = min(B_QL_rateLimit,B_QNL_m3s * sqrt(1 - PL_psi/PS_psi));
    B_QL_lpm = B_QL_m3s * m3s2lpm;
    B_QL_lpm_RL = B_QL_m3s_RL * m3s2lpm;
% 
% plot(PL_psi,A_QL_lpm,'*',PL_psi,B_QL_lpm,'*',...
%     PL_psi,A_QL_lpm_RL,PL_psi,B_QL_lpm_RL),grid
% legend('high airspeed chambers','low airspeed chambers',...
%     'high airspeed chambers RL','low speed chambers RL')

%--------------------------------------------------------------------------
% Load-velocity curves
%--------------------------------------------------------------------------
% Configuration A (high airspeed configuration)
A_load_N = PL_psi*psi2Pa * A_area;
A_velocity_ms = A_QL_m3s / A_area;
A_velocity_ms_RL = A_QL_m3s_RL / A_area;

A_load_KN = A_load_N * 1e-3;
    A_velocity_mms = A_velocity_ms * 1e3;
    A_velocity_mms_RL = A_velocity_ms_RL * 1e3;
% Configuration B (low speed configuration)
B_load_N = PL_psi * psi2Pa * B_area;
B_velocity_ms = B_QL_m3s / B_area;
B_velocity_ms_RL = B_QL_m3s_RL / B_area;
    B_load_KN = B_load_N * 1e-3;
    B_velocity_mms = B_velocity_ms * 1e3;
    B_velocity_mms_RL = B_velocity_ms_RL * 1e3;
% plot(A_load_KN,A_velocity_mms,'*',B_load_KN,B_velocity_mms,'*',...
%     A_load_KN,A_velocity_mms_RL,B_load_KN,B_velocity_mms_RL),grid
% legend('high airspeed chambers','low airspeed chambers',...
%      'high airspeed chambers RL','low speed chambers RL')

%--------------------------------------------------------------------------
% Hinge-rate curves
%--------------------------------------------------------------------------
% Configuration A (high speed configuration)
A_Hinge_Nm = A_load_N * arm;
A_dotdelta_rad = A_velocity_ms / arm;
A_dotdelta_rad_RL = A_velocity_ms_RL / arm;

    A_Hinge_KNm = A_Hinge_Nm * 1e-3;
    A_dotdelta_deg = A_dotdelta_rad * rad2deg;
    A_dotdelta_deg_RL = A_dotdelta_rad_RL * rad2deg;
% Configuration B (low speed configuration)
B_Hinge_Nm = B_load_N * arm;
B_dotdelta_rad = B_velocity_ms / arm;
    B_rateLimit_rad = B_rateLimit_deg * deg2rad;
B_dotdelta_rad_RL = min(B_velocity_ms_RL / arm, B_rateLimit_rad);
    B_Hinge_KNm = B_Hinge_Nm * 1e-3;
    B_dotdelta_deg = B_dotdelta_rad * rad2deg;
    B_dotdelta_deg_RL = B_dotdelta_rad_RL * rad2deg;

%--------------------------------------------------------------------------
% Main PLOT----------------------------------------------------------------
% Requirement points
loadReq = [70 20];
rateReq = [24.5 80];
% Sizing points
A_sizingLoad = [87.5 0];
A_sizingRate = [70 20*1.1];
B_sizingLoad = [24.5*3/2 0];
B_sizingRate = [24.5 80*1.1];
% Hm-dotdelta plot
limB = 200;
rangeB = limB:1:length(B_Hinge_KNm);
p1 = plot(A_Hinge_KNm,A_dotdelta_deg,'r',B_Hinge_KNm(rangeB),B_dotdelta_deg(rangeB),'b--',...
    B_Hinge_KNm,B_dotdelta_deg_RL,'b',...
    A_sizingLoad(1),A_sizingLoad(2),'ro',...
    A_sizingRate(1),A_sizingRate(2),'r*',...
    B_sizingLoad(1),B_sizingLoad(2),'bo',...
    B_sizingRate(1),B_sizingRate(2),'b*',...
    loadReq(1),loadReq(2),'g>',...
    rateReq(1),rateReq(2),'g<','MarkerSize',26);
grid on
ax1 = gca; 
ax1.FontSize = 22;
p1(1).LineWidth = 4;
p1(2).LineWidth = 4;
p1(3).LineWidth = 4;
p1(4).LineWidth = 4;
p1(5).LineWidth = 4;
p1(6).LineWidth = 4;
p1(7).LineWidth = 4;
p1(8).LineWidth = 4;
p1(9).LineWidth = 4;
legend('High airspeed (max rate of 48 deg/s)',...
    'Low airspeed (no rate limit)',...
    'Low airspeed RL (rate limit at 80 deg/s)',...
    'Cylinder sizing point (high airspeed)','Valve sizing point (high airspeed)',...
    'Cylinder sizing point (low airspeed)','Valve sizing point (low airspeed)',...
    'High airspeed REQ','Low airspeed REQ',...
    'Location','northeast','FontSize', 20);
xlabel('Hinge moment (KN*m)','FontSize', 22);
ylabel('Surface rate (deg/s)','FontSize', 22);

%--------------------------------------------------------------------------
% Save
%--------------------------------------------------------------------------
save('actuationSizingDataDualTandem.mat',...
    'A_area','B_area',...
    'A_QNL_m3s','B_QNL_m3s',...
    'A_QL_rateLimit','B_QL_rateLimit',...
    'A_totalVol','B_totalVol')
% %--------------------------------------------------------------------------
% % Plot
% %--------------------------------------------------------------------------
% subplot(2,2,[1,2])
% plot(PL_psi,QL_ConfA_lpm,'r--',...
%     PL_psi,QL_ConfA_lpm_RL,'b',...
%     PL_psi,QL_ConfB_lpm_RL,'g'),grid
% xlabel('Pressure (psi)')
% ylabel('Flow (lpm)')
% legend('Low Speed','Low speed (RL = 80 deg/s)','High Speed')
% 
% subplot(2,2,3)
% plot(load_ConfA_KN,velocity_ConfA_mms,'r--',...
%     load_ConfA_KN,velocity_ConfA_mms_RL,'b',...
%     load_ConfB_KN,velocity_ConfB_mms_RL,'g'),grid
% xlabel('Load (KN)')
% ylabel('Velocity (mms)')
% legend('Low Speed','Low speed (RL = 80 deg/s)','High Speed')
% 
% subplot(2,2,4)
% plot(Hinge_ConfA_KNm,dotdelta_ConfA_deg,'r--',...
%     Hinge_ConfA_KNm,dotdelta_ConfA_deg_RL,'b',...
%     Hinge_ConfB_KNm,dotdelta_ConfB_deg_RL,'g'),grid
% xlabel('Hinge moment (KN*m)')
% ylabel('Surface rate (deg/s)')
% legend('Low Speed','Low speed (RL = 80 deg/s)','High Speed')
% 
% % % -------------------------------------------------------------------------
% % % Plot results-------------------------------------------------------------
% % hingeSizingPoint_FlowPressure = [sizingCriteria*PS_psi,QNL_lpm*sqrt(1-sizingCriteria)];
% % hingeSizingPoint_RateHinge = [Hm1,(QNL_m3s*sqrt(1-sizingCriteria)/(armArea*loadFlowMargin))*rad2deg];
% % PowerSizingPoint = [sizingCriteria*PS_psi,QNL_m3s*sqrt(1-sizingCriteria)*sizingCriteria*PS_psi*psi2Pa];
% % if sizingCase == 0
% %     rateSizingPoint_FlowPressure = [Hm1/armArea*Pa2psi,dotdelta1_rad*armArea*m3s2lpm*loadFlowMargin];
% %     rateSizingPoint_RateHinge = [Hm1,(QNL_m3s*sqrt(1-sizingCriteria)/ (armArea*loadFlowMargin))*rad2deg];
% % else
% %     rateSizingPoint_FlowPressure = [Hm2/armArea*Pa2psi,dotdelta2_rad*armArea*m3s2lpm*loadFlowMargin];
% %     rateSizingPoint_RateHinge = [Hm2,dotdelta2_deg];
% % end
% % figure %-------------------------------------------------------------------
% % subplot(3,2,1)
% % plot(PL_psi,QL_lpm,'--',PL_psi,QL_lpm_RL,...
% %     rateSizingPoint_FlowPressure(1),rateSizingPoint_FlowPressure(2),'k*',...
% %     hingeSizingPoint_FlowPressure(1),hingeSizingPoint_FlowPressure(2),'ko',...
% %     'MarkerSize',10),grid
% % title(strcat('Flow-pressure curve,',textCriteria))
% % legend('No rate limit','Rate limit applied','Valve sizing point',...
% %     'Cylinder sizing point','Location','southwest')
% % xlabel('Pressure [psi]')
% % ylabel('Load-flow [lpm]')
% % 
% % subplot(3,2,3)
% % plot((Hm/arm)*1e-3,dotdelta_deg*deg2rad*arm*1e3,'--',(Hm/arm)*1e-3,dotdelta_deg_RL*deg2rad*arm*1e3,...
% %     (rateSizingPoint_RateHinge(1)/arm)*1e-3,rateSizingPoint_RateHinge(2)*deg2rad*arm*1e3,'k*',...
% %     (hingeSizingPoint_RateHinge(1)/arm)*1e-3,hingeSizingPoint_RateHinge(2)*deg2rad*arm*1e3,'ko','MarkerSize',10),grid
% % title(strcat('Velocity-force curve,',textCriteria))
% % legend('No rate limit','Rate limit applied','Valve sizing point',...
% %     'Cylinder sizing point','Location','southwest')
% % xlabel('Force [KN]')
% % ylabel('Velocity [mm/s]')
% % 
% % subplot(3,2,5)
% % plot(Hm*1e-3,dotdelta_deg,'--',Hm*1e-3,dotdelta_deg_RL,...
% %     rateSizingPoint_RateHinge(1)*1e-3,rateSizingPoint_RateHinge(2),'k*',...
% %     hingeSizingPoint_RateHinge(1)*1e-3,hingeSizingPoint_RateHinge(2),'ko','MarkerSize',10),grid
% % title(strcat('Rate-hinge curve,',textCriteria))
% % legend('No rate limit','Rate limit applied','Valve sizing point',...
% %     'Cylinder sizing point','Location','southwest')
% % xlabel('Hinge moment [KN*m]')
% % ylabel('Surface rate [deg/s]')
% % 
% % subplot(3,2,[2,6])
% % plot(PL_psi,developedPower_W*1e-3,'--',PL_psi,developedPower_W_RL*1e-3,...
% %     PL_psi,consumedPower_W*1e-3,'--',PL_psi,consumedPower_W_RL*1e-3,...
% %     PowerSizingPoint(1),PowerSizingPoint(2)*1e-3,'kd','MarkerSize',10),grid
% % title(strcat('Developed and consumed Power,',textCriteria))
% % legend('Developed no rate limit','Developed rate limit applied',...
% %     'Consumed no rate limit','Consumed rate limit applied','Sizing point',...
% %     'Location','northeast')
% % xlabel('Pressure [psi]')
% % ylabel('Power [KW]')
% 
% 
