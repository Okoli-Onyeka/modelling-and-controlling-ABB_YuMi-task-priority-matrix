%      Modified by Onyeka Franklin Okoli on 15/11/2025
%      Original code from Maram Khatib task-priority-relaxed-constraints
%      Modifications:
%                    - assigns ABB YuMi's link length to variables a1 to a7
%                    - assigns ABB YuMi's link offset to variables d1, d3, d5, and d7
%                    - This code now runs robot, an instance of ABB_YUMI (previously KUKA_LWR).  
%                    - It uses trajectory planning with Task_ABB_TRAJECTORY() and moving_obstacles() 
%                    - Removed the body collision avoidance task
%                    - Changed proportional gain for moving obstacles
%                    - calls Task_ABB()
% 
%      Copyright 2020 Maram Khatib
%  
%      Licensed under the Apache License, Version 2.0 (the "License");
%      you may not use this file except in compliance with the License.
%      You may obtain a copy of the License at
%      http://www.apache.org/licenses/LICENSE-2.0
%  
%      Unless required by applicable law or agreed to in writing, software
%      distributed under the License is distributed on an "AS IS" BASIS,
%      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%      See the License for the specific language governing permissions and
%      limitations under the License.

clc; 
clear;
%%% initialization

% included to toggle moving obstacles and trajectory planning on or off
% set to true or false
movingObs = false;
trajectoryABB = false;

%changed to  ABB YuMi's link length to variables a1 to a7
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
d7=0.036;   % distance from sixth joint frame to EE frame; EE in the tip of YuMi without auxiliary addition


Zd =[1,0,0];% parallel to x-axis, for desired pointing task 


q1=[-2.8913;-1.4011;1.9339;2.4051;2.9015;1.4097;0];
dq1=[0;0;0;0;0;0;0];
ddq1=[0;0;0;0;0;0;0];

global robot

% Original Code robot=KUKA_LWR(d1,d3,d5,d7,q1,dq1,Zd)
% Changed by Onyeka Okoli on 15/11/2025
% robot is now instantiated as an object of class ABB_YUMI
robot=ABB_YUMI(a1,a2,a3,a4,a5,a6,a7,d1,d3,d5,d7,q1,dq1,Zd);

numberOfObstacles=2;
obs = zeros(numberOfObstacles,3);
obs(1,:)=[-0.0484;0.4121;-0.0484]; % cross the path
obs(2,:)=[-0.1225;0.7;-0.1225]; % second moving obstacle


if movingObs
    p_obs0 = obs;
else
    p_obs = obs;
end

if trajectoryABB
    T=0.001; %sampling time
    Ttot = 2.5;
else
    T=0.001; %sampling time
    Ttot = 6*pi;
end 

n=7; %number of joints
I=eye(n);

p0_ee=robot.computeEnd_effectorPosition;%initial Cartesian position

[~,Ori_z]=robot.computeEnd_effectorOrientation;
orientation1_ee=Zd*Ori_z;%cos alpha

%current configuration
q=q1;
dq_prref=dq1;
ddq_prref=ddq1;

%controller gains

Kp0=70;%positional task
Kd0=20;%positional task

Krp1=5;%pointing task
Krd1=2;%pointing task

if trajectoryABB
    [p,dp,ddp] = Task_ABB_Trajectory(0);%initial Cartesian point
else
    [p,dp,ddp] = Task_ABB(0);%initial Cartesian point
end

desiredAcc0=zeros(3,1);
desiredVelocity0=zeros(3,1);
desiredPosition0=p;
desiredAcc1=0;
desiredVelocity1=0;

alpha_d=5;% desired alpha for pointing task 5
desiredorientation1=cos(deg2rad(alpha_d));

%save quantities
Size=round(Ttot/T);
QV=zeros(Size,7);
dQV=zeros(Size,7);
ddQV=zeros(Size,7);
distanceControlPointsV=zeros(Size,8);
    

T0d=zeros(Size,3);
e0=zeros(Size,3);
Ze1=zeros(Size,3);
Pe1=zeros(Size,3);

T1d=zeros(Size,1);
e1=zeros(Size,1);

extTime=zeros(Size,1);

distanceEEMin=ones(Size,1);
distanceEEMin=distanceEEMin*.11;

%%% control loop
i=1;
for t=0:T:Ttot
   % Set Moving Obstacles
    if movingObs
        for k = 1:numberOfObstacles
            p_obs(k,:) = movingObstacle(t, p_obs0(k,:));
        end
        P_obs(:,:,i) = p_obs;
    end
   
    %TASK 0: ellipse with EE
    robot.set_q_and_qdot(q,dq_prref);
    p0_ee=robot.computeEnd_effectorPosition;
    dp0_ee=robot.computeEnd_effectorPosVelocity;
    A0=robot.computeJacobianPos;
    dJacobianPos= (A0 - robot.PrevJacobianPos)/T;
    
    if trajectoryABB
        [p,dp,ddp] = Task_ABB_Trajectory(t);
    else
        [p,dp,ddp] = Task_ABB(t);
    end
    
    desiredPosition0=p;
	desiredVelocity0=dp;
    desiredAcc0=ddp;
    e0(i,:)=desiredPosition0-p0_ee;
    ev0=desiredVelocity0-dp0_ee;
   
	b0= Kp0*(e0(i,:)')+Kd0*ev0;
   	b0= b0 + desiredAcc0 - dJacobianPos*dq_prref;

    %TASK 1: EE pointing 
    [~,Ori_z]=robot.computeEnd_effectorOrientation;
    Ze1(i,:)=Ori_z;
    orientation1_ee=Zd*Ori_z;
    alpha=rad2deg(acos(orientation1_ee));
    [~,A1]=robot.computeJacobianOri(Zd,Ori_z);
    dJacobianOri= (A1 - robot.PrevJacobianOri)/T;
    
    desiredAcc1=0;
    desiredVelocity1=0;
    e1(i,1)=desiredorientation1-orientation1_ee;%pointing error
    ev1=desiredVelocity1-robot.computeEnd_effectorOriVelocity(Zd);
	b1= Krp1*e1(i,1)+Krd1*ev1;
   	b1= b1 + desiredAcc1- (dJacobianOri*dq_prref);

	desiredPosition0=p;
    desiredorientation1=cos(deg2rad(alpha_d));
    
    %save current quantities
    QV(i,:)=q;
    dQV(i,:)=dq_prref;
    ddQV(i,:)=ddq_prref;
    
    T0d(i,:)=p0_ee;
    T1d(i,:)=orientation1_ee;
    
   %TASK 2: EE obstacle avoidance
    [dp2,dmin,velocity2,nn] = repulsiveEE(p0_ee,p_obs);
    b2=dp2;
    A2=A0;
    distanceEEMin(i,1)=dmin;
 
   %Compute cone top
    Pe1(i,:) = robot.computejoint6';
    
%%% Arranging tasks priority
    A=A0;
    idx=3;%task dimention
    b=b0+b2;% positional task + EE cllision avoidance

 if  e1(i,1)> 0 %1e-4 %%pointing error
     
     A=[A;A1];
     idx=[idx,1];
     b=[b;b1]; 
     
 end

%%% damping parameters for Jacobian inverse 
eps=1e-1;
lambda=0.5e-1;

[Q,R]=qr(A',0);
X=tasksPriorityMatrix(R,idx,lambda,eps);% task priority matrix

Kdamp=5;% joint damping gain
Jtotal = A;
JtotalInv= Q*pinv(R');

ddq_prref=JtotalInv*X*b +  ((I-(JtotalInv*Jtotal))* -Kdamp*dq_prref);% commanded acceleration  

q= q + dq_prref*T+ddq_prref*0.5*T*T; %%current position
dq_prref=dq_prref+ddq_prref*T; %%current velocity

robot.set_update(Zd);

i=i+1;

    if (mod(t,1)==0)
        t
    end
    
end

disp('finish');



