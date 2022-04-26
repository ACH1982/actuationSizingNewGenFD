% Bulk estimation considering: 
% rod made of steel: E = 200GPa
% rod inner diameter: for sensors location = 40 mm
% bulk modulus (hydraulic fluid) = 1.7e9 Pa
% Supply pressure = 5000 psi
clear all
% Load data
load('actuationUnitsConversion.mat');
load('actuationInputDataList.mat');
load('actuationSizingData.mat');
%--------------------------------------------------------------------------
% Estimation --------------------------------------------------------------
% Container bulk ----------------------------------------------------------
rodInnerDiam = 40e-3; % m
youngM = 2e11; % Pa
stallAreaScaling = (572e-6)/(125e3); % mm^2/KN for 
wallThickness_3 = 3e-3; % m
wallThickness_5 = 5e-3; % m
wallThickness_10 = 10e-3; % m
PS_Pa = PS_psi*psi2Pa;
stall = area*PS_Pa;
rodOuterDiam = sqrt(rodInnerDiam^2 + 4*stallAreaScaling*stall/pi);
rodBoreInnerDiam = sqrt(rodOuterDiam^2 + 4*area/pi);

rodBoreOuterDiam_3 = rodBoreInnerDiam + wallThickness_3;
rodBoreOuterDiam_5 = rodBoreInnerDiam + wallThickness_5;
rodBoreOuterDiam_10 = rodBoreInnerDiam + wallThickness_10;

bulkContainer_3 = youngM*wallThickness_3/rodBoreInnerDiam;
bulkContainer_5 = youngM*wallThickness_5/rodBoreInnerDiam;
bulkContainer_10 = youngM*wallThickness_10/rodBoreInnerDiam;

% Hydraulic fluid bulk ----------------------------------------------------
bulkFluid = 1.7e9; % Pa
% Hydraulic fluid air -----------------------------------------------------
bulkAir = 1.4*PS_Pa;
airContent = [0.1/100 0.5/100 1/100 1.5/100 2/100 2.5/100 3/100]; % percentage
airContentPercentage = airContent*100;
bulkAirEff = bulkAir./airContent;
bulkFluidEff = bulkFluid*bulkAirEff./(bulkFluid+bulkAirEff);
% Effective Bulk
sizingBulkEff_Pa = 1e5*psi2Pa*ones(length(airContent),1);
bulkEff_3 = (bulkContainer_3*bulkFluidEff)./(bulkContainer_3+bulkFluidEff);
bulkEff_5 = (bulkContainer_5*bulkFluidEff)./(bulkContainer_5+bulkFluidEff);
bulkEff_10 = (bulkContainer_10*bulkFluidEff)./(bulkContainer_10+bulkFluidEff);
stiffness_3 = (4*bulkEff_3*area^2)./totalVol;
stiffness_5 = (4*bulkEff_5*area^2)./totalVol;
stiffness_10 = (4*bulkEff_10*area^2)./totalVol;
stiffness = (4*sizingBulkEff_Pa*area^2)./totalVol;
yyaxis left
plot(airContentPercentage,bulkEff_3,'r*--',...
    airContentPercentage,bulkEff_5,'b*--',...
    airContentPercentage,bulkEff_10,'g*--',...
    airContentPercentage,sizingBulkEff_Pa,'k'),grid
xlabel('Air content (percentage)')
ylabel('Effective bulk (Pa)')
yyaxis right
plot(airContentPercentage,stiffness_3,'ro--',...
    airContentPercentage,stiffness_5,'bo--',...
    airContentPercentage,stiffness_10,'go--',...
    airContentPercentage,stiffness,'k--')
ylabel('Stiffness (N/m)')
legend('Computed effective Bulk wall = 3 mm',...
    'Computed effective Bulk wall = 5 mm',...
    'Computed effective Bulk wall = 10 mm',...
    'Effective bulk for sizing',...
    'Stiffness for wall = 3 mm',...
    'Stiffness for wall = 5 mm',...
    'Stiffness for wall = 10 mm',...
    'Stiffness (using effective bulk for sizing)')




