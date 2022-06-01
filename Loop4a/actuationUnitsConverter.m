% Units conversion for actuation sizing and dynamics
% Conversion parameters----------------------------------------------------
psi2Pa = 6894.757;
Pa2psi = 1/6894.757;
m3s2lpm = 6e4;
lpm2m3s = 1/6e4;
deg2rad = pi/180;
rad2deg = 180/pi;
% Save data----------------------------------------------------------------
save('actuationUnitsConversion.mat',...
    'psi2Pa','Pa2psi',...
    'm3s2lpm','lpm2m3s',...
    'deg2rad','rad2deg');