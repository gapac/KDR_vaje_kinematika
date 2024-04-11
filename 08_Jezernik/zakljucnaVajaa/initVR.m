global cont
warning off all

% Vzrocni cas simulacije v Simulinku.
dT = 0.005;

% Zacetna lega robota za vse gibe.
q0 = [0 0 0]'*pi/180;

% Ojacenje za izracun inverzne kinematike.
fbGain = 20;

% nalozi trajektorijo
load('xTrajec.mat');

% Odpiranje predloge.
open HapticMasterVR;


