%% specifyng the variables

%beam dimensions and mesh size
global meshSize_finer meshSize_coarse L_by_width Beam_width depth  
meshSize_finer=8;
meshSize_coarse=3;
L_by_width=15;
Beam_width=0.05;
depth=0.05;

%material properties
global density youngsModulus StifferYoungModulus gravity gravity_direction
density=1200;
youngsModulus=1*10^6;
% youngsModulus=8/(3*depth);
StifferYoungModulus=youngsModulus*100; %based on what variation is needed in the Kfile
gravity=0.000;
gravity_direction=1;

%simulation properties
global MassDamping StiffnesDamping initialPose time_for_full_load
MassDamping=1;
StiffnesDamping=0.1*10^0.5;
initialPose=false;

%continium robot param
global nSection force_ramp impulse_force timed_force
nSection=5;

impulse_force=0;
force_ramp=0;% 1 for true and 0 for false
timed_force=0;
time_for_full_load=40;
%% creating obj
%file name will be beam.obj
createObj();

%% creating the constraints file
%file name will be constraints.bou
% constraintsFileFixedTop();
constraintsFileFixedTopWithFixedZ();
% constraintsFileRollingTopWithFixedZ();

%% creating the loading file
noLoad();
% verticalLoad(-1000);
% bendingLoadX(80);
% bendingLoadZ(80);
%% moment for continium robot
M=zeros(nSection,1);
M(2)=0;
M(1)=0;
momentAtEachSection(M);
%% creating config files
configFiles();

%% creating the gravity file for implicitNewmarkSparse.cpp
gravityFile();

%% beam dimensions  and configurations for vega 
vegaConfigurationFile();

%% creating the stiffness file for massSpringSystem.cpp
% sameStiffness();
% TwoRigidBarAlongWidthStiffness();
nRigidBarAlongWidthStiffness();

%% writing config file for matalb to read while analyzing vega data
matlabConfigforReadingData();
