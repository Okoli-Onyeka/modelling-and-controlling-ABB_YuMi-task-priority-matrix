clc; 
clear;
%%% initialization

a1 = -0.03;
a2 = 0.03;
a3 = 0.0405;
a4 = 0.0405;
a5 = 0.027;
a6 = -0.027;
a7 = 0;

d1=0.166;     % distance from base frame to second joint frame 
d3=0.2515;     % distance from second joint frame to fourth joint frame
d5=0.265;    % distance from fourth joint frame to sixth joint frame 
d7=0.036;   % distance from sixth joint frame to EE frame; EE in the tip of KUKA without auxiliary addition

Zd =[0,0,0];% parallel to x-axis, for desired pointing task 

%initial position
q1=[0;0;0;0;0;0;0];
dq1=[0;0;0;0;0;0;0];%initial velocity
ddq1=[0;0;0;0;0;0;0];%initial acceleration
    
global robot
robot=KUKA_LWR(a1,a2,a3,a4,a5,a6,a7,d1,d3,d5,d7,q1,dq1,Zd);
[~,Ori_z]=robot.computeEnd_effectorOrientation;


robot.plotarm()
% Jpos = robot.compute_control_points_position;
% Jori = robot.computeJacobianOri(Zd, Ori_z);
% rank([Jpos;Jori])