%% initializing ROS and variables
clear
global tip_poses_t tip_vel_t xt yt n timeStep nColsNodes deltaL original_pose f1 f2 f3 f5 slopes_t force_t TimeStepCount
timeStep=0.5;
force_t=[];
tip_poses_t=[];
tip_vel_t=[];
xt=[];
yt=[];
slopes_t=[];
TimeStepCount=0;
rosinit
tip_pose_sub=rossubscriber("/steady_system/state","std_msgs/Float32MultiArray",@tip_pose_calllback,"DataFormat","struct");
% slope_pose_sub=rossubscriber("/actual_system/slope","std_msgs/Float32MultiArray",@slope_calllback,"DataFormat","struct");
state_sub=rossubscriber("/steady_system/complete_state","std_msgs/Float32MultiArray",@state_callback,"DataFormat","struct");
target_sub=rossubscriber("/actual_system/target","std_msgs/Float32MultiArray",@target_callback,"DataFormat","struct");
force_sub=rossubscriber("/steady_system/force","std_msgs/Float32MultiArray",@force_calllback,"DataFormat","struct");
vel_sub=rossubscriber("/steady_system/velocity","std_msgs/Float32MultiArray",@velocity_calllback,"DataFormat","struct");
radius_sub=rossubscriber("/actual_system/radius","std_msgs/Float32MultiArray",@radius_calllback,"DataFormat","struct");
f1=figure;
f2=figure;
f3=figure;
f5=figure;

%% read configuration to know beam dimensions
f=csvread('~/catkin_ws/src/Soft_robo_sim/MATLAB/ros_data/config.csv');
n=f(1);
nColsNodes=f(2);
meshSize=nColsNodes-1;
BeamWidth=f(3);
deltaL=BeamWidth/meshSize;
original_pose=zeros(n,2);
nRows=n/nColsNodes;
nColumns=zeros(nRows,1);
nColumns(1:end)=meshSize;
count=1;
for i=0:(nRows-1)
    for j=0:nColumns(i+1,1)
        original_pose(count,:)=[-1*nColumns(i+1,1)*0.5*deltaL+j*deltaL,-deltaL*i];
        count=count+1;
    end
end