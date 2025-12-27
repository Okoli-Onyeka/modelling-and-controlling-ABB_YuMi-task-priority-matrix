%      Modified by Onyeka Okoli on 15/11/2025
%      Original code from Maram Khatib task-priority-relaxed-constraints
%      Modifications:
%                    - Previously KUKA_LWR, the class has been changed to
%                      ABB_YUMI and its functions have been modified to
%                      define the kinematics of ABB YuMi Dual Arm Robot
%                    
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


classdef ABB_YUMI<handle
    
    % Modified properties now include the links length and a moveable
    % base, Tbase.
    properties
        a1;
        a2;
        a3;
        a4;
        a5;
        a6;
        a7;
        d1; % distance from base to second joint
        d3; % distance from second joint to fourth joint
        d5; % distance from fourth joint to sixth joint
        d7; % distance from sixth joint to end-effector
        q; % 7x1 vector with the current joint configuration
        qdot; % 7x1 vector with the current joint velocity 

        Tbase; %the base reference frame
        
        PrevJacobianPos; % 3x7 matrix
        PrevJacobianOri; % 1x7 matrix
        PrevJacobianControlPoints; % 3x7x8 matrix related to the 8 control points
    
    end
    
    methods
        % New function ABB_YUMI()
        function obj=ABB_YUMI(a1,a2,a3,a4,a5,a6,a7,d1,d3,d5,d7,q,qdot,zd)
            
            obj.Tbase =  obj.transl(-0.3, 0.35, -0.2);
            obj.a1 = a1;
            obj.a2 = a2;
            obj.a3 = a3;
            obj.a4 = a4;
            obj.a5 = a5;
            obj.a6 = a6;
            obj.a7 = a7;
            obj.d1=d1;
            obj.d3=d3;
            obj.d5=d5;
            obj.d7=d7;
            obj.q=q;
            obj.qdot=qdot;
            obj.PrevJacobianPos = obj.computeJacobianPos;
            [~,ze]=obj.computeEnd_effectorOrientation;
            [~,obj.PrevJacobianOri] = obj.computeJacobianOri(zd,ze);
            
        end
        
        function obj=set_q_and_qdot(obj,q,qdot)
            obj.q=q;
            obj.qdot=qdot;
        end
        
        function obj=set_update(obj,zd)
            obj.PrevJacobianPos = obj.computeJacobianPos;
            [~,ze]=obj.computeEnd_effectorOrientation;
            [~,obj.PrevJacobianOri] = obj.computeJacobianOri(zd,ze);
        end
        
        % Modified, the jacobian J has been changed to YuMi's position
        % jacobian
        function J=computeJacobianPos(obj)
       
            %for jacobian 3*7
            q1=obj.q(1);
            q2=obj.q(2);
            q3=obj.q(3);
            q4=obj.q(4);
            q5=obj.q(5);
            q6=obj.q(6);
           
        %position jacobian 3*7  
        J =[(9*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)))/250 - (503*sin(q1)*sin(q2))/2000 - (27*cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)))/1000 - (81*cos(q1)*sin(q3))/2000 + (27*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)))/1000 + (27*cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/1000 - (3*sin(q1)*(cos(q2) - 1))/100 - (9*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/250 - (81*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/2000 + (53*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/200 + (27*sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))/1000 + (81*sin(q1)*sin(q2)*sin(q4))/2000 - (81*cos(q2)*cos(q3)*sin(q1))/2000 + (53*cos(q4)*sin(q1)*sin(q2))/200, -cos(q1)*((3*sin(q2))/100 - (503*cos(q2))/2000 + (53*cos(q2)*cos(q4))/200 + (81*cos(q3)*sin(q2))/2000 + (81*cos(q2)*sin(q4))/2000 - (27*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/1000 + (9*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/250 + (27*cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)))/1000 + (9*cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/250 + (27*sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/1000 + (27*sin(q2)*sin(q3)*sin(q5))/1000 + (81*cos(q3)*cos(q4)*sin(q2))/2000 - (53*cos(q3)*sin(q2)*sin(q4))/200), (53*cos(q3)*sin(q1)*sin(q4))/200 - (27*sin(q1)*sin(q3)*sin(q5))/1000 - (81*cos(q1)*cos(q2)*sin(q3))/2000 - (81*cos(q3)*cos(q4)*sin(q1))/2000 - (81*cos(q3)*sin(q1))/2000 - (81*cos(q1)*cos(q2)*cos(q4)*sin(q3))/2000 + (27*cos(q1)*cos(q2)*cos(q3)*sin(q5))/1000 - (27*cos(q3)*cos(q4)*cos(q5)*sin(q1))/1000 + (53*cos(q1)*cos(q2)*sin(q3)*sin(q4))/200 + (9*cos(q3)*cos(q6)*sin(q1)*sin(q4))/250 + (27*cos(q3)*sin(q1)*sin(q4)*sin(q6))/1000 + (27*cos(q6)*sin(q1)*sin(q3)*sin(q5))/1000 - (9*sin(q1)*sin(q3)*sin(q5)*sin(q6))/250 - (27*cos(q1)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/1000 - (27*cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q5))/1000 + (27*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1))/1000 + (9*cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q4))/250 + (9*cos(q1)*cos(q2)*cos(q3)*sin(q5)*sin(q6))/250 - (9*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6))/250 + (27*cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q6))/1000 + (27*cos(q1)*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3))/1000 - (9*cos(q1)*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6))/250, sin(q2)*sin(q3)*((9*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)))/250 - (27*cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)))/1000 + (27*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)))/1000 + (27*cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/1000 - (9*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/250 - (81*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/2000 + (53*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/200 + (27*sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))/1000 + (81*sin(q1)*sin(q2)*sin(q4))/2000 + (53*cos(q4)*sin(q1)*sin(q2))/200) - (cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))*((53*cos(q2)*cos(q4))/200 + (81*cos(q2)*sin(q4))/2000 - (27*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/1000 + (9*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/250 + (27*cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)))/1000 + (9*cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/250 + (27*sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/1000 + (27*sin(q2)*sin(q3)*sin(q5))/1000 + (81*cos(q3)*cos(q4)*sin(q2))/2000 - (53*cos(q3)*sin(q2)*sin(q4))/200), (9*(4*sin(q6) - 3*cos(q6) + 3)*(cos(q3)*cos(q5)*sin(q1) + cos(q1)*cos(q2)*cos(q5)*sin(q3) + cos(q1)*sin(q2)*sin(q4)*sin(q5) + cos(q4)*sin(q1)*sin(q3)*sin(q5) - cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)))/1000, (sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3))*((9*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)))/250 + (27*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)))/1000 + (27*cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/1000 - (9*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/250) + (sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))*((9*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/250 - (27*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/1000 + (9*cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/250 + (27*sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/1000), 0;
            (503*cos(q1)*sin(q2))/2000 - (81*sin(q1)*sin(q3))/2000 - (27*cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)))/1000 + (9*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)))/250 + (27*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)))/1000 + (27*cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/1000 + (3*cos(q1)*(cos(q2) - 1))/100 - (9*sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/250 - (81*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/2000 + (53*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/200 + (27*sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))/1000 + (81*cos(q1)*cos(q2)*cos(q3))/2000 - (53*cos(q1)*cos(q4)*sin(q2))/200 - (81*cos(q1)*sin(q2)*sin(q4))/2000, -sin(q1)*((3*sin(q2))/100 - (503*cos(q2))/2000 + (53*cos(q2)*cos(q4))/200 + (81*cos(q3)*sin(q2))/2000 + (81*cos(q2)*sin(q4))/2000 - (27*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/1000 + (9*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/250 + (27*cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)))/1000 + (9*cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/250 + (27*sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/1000 + (27*sin(q2)*sin(q3)*sin(q5))/1000 + (81*cos(q3)*cos(q4)*sin(q2))/2000 - (53*cos(q3)*sin(q2)*sin(q4))/200), (81*cos(q1)*cos(q3))/2000 + (81*cos(q1)*cos(q3)*cos(q4))/2000 - (53*cos(q1)*cos(q3)*sin(q4))/200 - (81*cos(q2)*sin(q1)*sin(q3))/2000 + (27*cos(q1)*sin(q3)*sin(q5))/1000 + (27*cos(q1)*cos(q3)*cos(q4)*cos(q5))/1000 - (9*cos(q1)*cos(q3)*cos(q6)*sin(q4))/250 - (81*cos(q2)*cos(q4)*sin(q1)*sin(q3))/2000 + (27*cos(q2)*cos(q3)*sin(q1)*sin(q5))/1000 - (27*cos(q1)*cos(q3)*sin(q4)*sin(q6))/1000 - (27*cos(q1)*cos(q6)*sin(q3)*sin(q5))/1000 + (53*cos(q2)*sin(q1)*sin(q3)*sin(q4))/200 + (9*cos(q1)*sin(q3)*sin(q5)*sin(q6))/250 - (27*cos(q1)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/1000 + (9*cos(q1)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/250 - (27*cos(q2)*cos(q4)*cos(q5)*sin(q1)*sin(q3))/1000 - (27*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q5))/1000 + (9*cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q4))/250 + (9*cos(q2)*cos(q3)*sin(q1)*sin(q5)*sin(q6))/250 + (27*cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q6))/1000 + (27*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q3))/1000 - (9*cos(q2)*cos(q4)*cos(q5)*sin(q1)*sin(q3)*sin(q6))/250, - (cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))*((53*cos(q2)*cos(q4))/200 + (81*cos(q2)*sin(q4))/2000 - (27*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/1000 + (9*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/250 + (27*cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)))/1000 + (9*cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/250 + (27*sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/1000 + (27*sin(q2)*sin(q3)*sin(q5))/1000 + (81*cos(q3)*cos(q4)*sin(q2))/2000 - (53*cos(q3)*sin(q2)*sin(q4))/200) - sin(q2)*sin(q3)*((27*cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)))/1000 - (9*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)))/250 - (27*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)))/1000 - (27*cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/1000 + (9*sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/250 + (81*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/2000 - (53*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/200 - (27*sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))/1000 + (53*cos(q1)*cos(q4)*sin(q2))/200 + (81*cos(q1)*sin(q2)*sin(q4))/2000), -(9*(4*sin(q6) - 3*cos(q6) + 3)*(cos(q1)*cos(q3)*cos(q5) - cos(q2)*cos(q5)*sin(q1)*sin(q3) + cos(q1)*cos(q4)*sin(q3)*sin(q5) - sin(q1)*sin(q2)*sin(q4)*sin(q5) + cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)))/1000, (sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))*((9*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/250 - (27*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/1000 + (9*cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/250 + (27*sin(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4)))/1000) + (sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3))*((9*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)))/250 + (27*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)))/1000 + (27*cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/1000 - (9*sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/250), 0;
            0, (53*cos(q4)*sin(q2))/200 - (503*sin(q2))/2000 - (81*cos(q2)*cos(q3))/2000 - (3*cos(q2))/100 + (81*sin(q2)*sin(q4))/2000 - (81*cos(q2)*cos(q3)*cos(q4))/2000 + (53*cos(q2)*cos(q3)*sin(q4))/200 + (9*cos(q4)*cos(q6)*sin(q2))/250 - (27*cos(q2)*sin(q3)*sin(q5))/1000 + (27*cos(q5)*sin(q2)*sin(q4))/1000 + (27*cos(q4)*sin(q2)*sin(q6))/1000 - (27*cos(q2)*cos(q3)*cos(q4)*cos(q5))/1000 + (9*cos(q2)*cos(q3)*cos(q6)*sin(q4))/250 + (27*cos(q2)*cos(q3)*sin(q4)*sin(q6))/1000 + (27*cos(q2)*cos(q6)*sin(q3)*sin(q5))/1000 - (27*cos(q5)*cos(q6)*sin(q2)*sin(q4))/1000 - (9*cos(q2)*sin(q3)*sin(q5)*sin(q6))/250 + (9*cos(q5)*sin(q2)*sin(q4)*sin(q6))/250 + (27*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6))/1000 - (9*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6))/250, -(sin(q2)*(54*cos(q3)*sin(q5) - 81*cos(q4)*sin(q3) - 81*sin(q3) + 530*sin(q3)*sin(q4) + 54*sin(q3)*sin(q4)*sin(q6) - 54*cos(q4)*cos(q5)*sin(q3) - 54*cos(q3)*cos(q6)*sin(q5) + 72*cos(q6)*sin(q3)*sin(q4) + 72*cos(q3)*sin(q5)*sin(q6) + 54*cos(q4)*cos(q5)*cos(q6)*sin(q3) - 72*cos(q4)*cos(q5)*sin(q3)*sin(q6)))/2000, (53*cos(q2)*sin(q4))/200 - (81*cos(q2)*cos(q4))/2000 - (27*cos(q2)*cos(q4)*cos(q5))/1000 + (53*cos(q3)*cos(q4)*sin(q2))/200 + (9*cos(q2)*cos(q6)*sin(q4))/250 + (81*cos(q3)*sin(q2)*sin(q4))/2000 + (27*cos(q2)*sin(q4)*sin(q6))/1000 + (27*cos(q2)*cos(q4)*cos(q5)*cos(q6))/1000 + (9*cos(q3)*cos(q4)*cos(q6)*sin(q2))/250 - (9*cos(q2)*cos(q4)*cos(q5)*sin(q6))/250 + (27*cos(q3)*cos(q5)*sin(q2)*sin(q4))/1000 + (27*cos(q3)*cos(q4)*sin(q2)*sin(q6))/1000 - (27*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4))/1000 + (9*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6))/250, (9*(cos(q2)*sin(q4)*sin(q5) - cos(q5)*sin(q2)*sin(q3) + cos(q3)*cos(q4)*sin(q2)*sin(q5))*(4*sin(q6) - 3*cos(q6) + 3))/1000, (9*cos(q2)*cos(q4)*sin(q6))/250 - (27*cos(q2)*cos(q4)*cos(q6))/1000 - (9*cos(q2)*cos(q5)*cos(q6)*sin(q4))/250 + (27*cos(q3)*cos(q6)*sin(q2)*sin(q4))/1000 - (27*cos(q2)*cos(q5)*sin(q4)*sin(q6))/1000 - (9*cos(q3)*sin(q2)*sin(q4)*sin(q6))/250 - (9*cos(q6)*sin(q2)*sin(q3)*sin(q5))/250 - (27*sin(q2)*sin(q3)*sin(q5)*sin(q6))/1000 - (9*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2))/250 - (27*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6))/1000, 0];
             
        
        end
        
       % Modified, the jacobian J has been changed to YuMi's orientation
       % jacobian
       function [J,Jori]=computeJacobianOri(obj,Zd,Ze)
           
            q1=obj.q(1);
            q2=obj.q(2);
            q3=obj.q(3);
            q4=obj.q(4);
            q5=obj.q(5);
            q6=obj.q(6);
            
            %jacobian ori 3*7    
            J =[0, -sin(q1), cos(q1)*sin(q2), - cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3), sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2), sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)), cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)));
                0, cos(q1), sin(q1)*sin(q2), cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3), - sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2), - sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)), sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2));
                1, 0, cos(q2), sin(q2)*sin(q3), cos(q3)*sin(q2)*sin(q4) - cos(q2)*cos(q4), sin(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3), - sin(q6)*(cos(q5)*(cos(q2)*sin(q4) + cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4))];   
                
            v =[ 0, -Ze(3), Ze(2); Ze(3), 0, -Ze(1); -Ze(2), Ze(1), 0];
	
            Jori=Zd*v'*J;
       end
        
        function Jdot=computeJacobianDot(obj)
            %%%%Compute the time derivative of the Jacobian%%%%%%%%%%%%%%%
            l2=obj.d3;
            l4=obj.d5;
            l7=obj.d7;
            qdot1=obj.qdot(1);
            qdot2=obj.qdot(2);
            qdot3=obj.qdot(3);
            qdot4=obj.qdot(4);
            qdot5=obj.qdot(5);
            qdot6=obj.qdot(6);
            s1=sin(obj.q(1));
            s2=sin(obj.q(2));
            s3=sin(obj.q(3));
            s4=sin(obj.q(4));
            s5=sin(obj.q(5));
            s6=sin(obj.q(6));
            c1=cos(obj.q(1));
            c2=cos(obj.q(2));
            c3=cos(obj.q(3));
            c4=cos(obj.q(4));
            c5=cos(obj.q(5));
            c6=cos(obj.q(6));
            Jdot=[qdot1*(l4*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)+l7*(c6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)-s6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3)))+l2*c1*s2)-qdot4*(l7*(c6*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+c5*s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2))+l4*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4))-qdot3*(l7*(s6*(s5*(c1*s3+c2*c3*s1)-c4*c5*(c1*c3-c2*s1*s3))+c6*s4*(c1*c3-c2*s1*s3))+l4*s4*(c1*c3-c2*s1*s3))+qdot2*(l7*(s1*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+c6*s1*(c2*c4+c3*s2*s4))+l4*s1*(c2*c4+c3*s2*s4)+l2*c2*s1)+l7*qdot6*(s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+c6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3)))-l7*qdot5*s6*(s5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)-c5*(c1*c3-c2*s1*s3)),qdot4*(l4*c1*(c2*s4-c3*c4*s2)+l7*c1*(c6*(c2*s4-c3*c4*s2)-c5*s6*(c2*c4+c3*s2*s4)))+qdot2*(l4*c1*(c4*s2-c2*c3*s4)+l2*c1*s2+l7*c1*(s6*(c5*(s2*s4+c2*c3*c4)-c2*s3*s5)+c6*(c4*s2-c2*c3*s4)))+qdot1*(l4*s1*(c2*c4+c3*s2*s4)+l2*c2*s1+l7*s1*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4)))-qdot3*(l7*c1*(s6*(c3*s2*s5+c4*c5*s2*s3)-c6*s2*s3*s4)-l4*c1*s2*s3*s4)-l7*qdot6*c1*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))+l7*qdot5*c1*s6*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3),l7*qdot6*(c3*s1*s4*s6-c6*s1*s3*s5+c1*c2*c3*c6*s5+c3*c4*c5*c6*s1+c1*c2*s3*s4*s6+c1*c2*c4*c5*c6*s3)-qdot3*(l4*c1*c2*c3*s4-l4*s1*s3*s4-l7*c6*s1*s3*s4+l7*c3*s1*s5*s6+l7*c1*c2*c3*c6*s4+l7*c1*c2*s3*s5*s6+l7*c4*c5*s1*s3*s6-l7*c1*c2*c3*c4*c5*s6)-qdot4*(c3*s1+c1*c2*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot1*(l4*c1*c3*s4+l7*c1*c3*c6*s4-l4*c2*s1*s3*s4+l7*c1*s3*s5*s6-l7*c1*c3*c4*c5*s6-l7*c2*c6*s1*s3*s4+l7*c2*c3*s1*s5*s6+l7*c2*c4*c5*s1*s3*s6)+qdot2*c1*s2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)-l7*qdot5*s6*(c5*s1*s3-c1*c2*c3*c5+c3*c4*s1*s5+c1*c2*c4*s3*s5),qdot4*(l4*c1*c4*s2+l4*s1*s3*s4-l4*c1*c2*c3*s4+l7*c1*c4*c6*s2+l7*c6*s1*s3*s4-l7*c1*c2*c3*c6*s4+l7*c1*c5*s2*s4*s6-l7*c4*c5*s1*s3*s6+l7*c1*c2*c3*c4*c5*s6)-qdot1*(l4*c1*c4*s3+l4*s1*s2*s4+l4*c2*c3*c4*s1+l7*c1*c4*c6*s3+l7*c6*s1*s2*s4+l7*c2*c3*c4*c6*s1-l7*c4*c5*s1*s2*s6+l7*c1*c5*s3*s4*s6+l7*c2*c3*c5*s1*s4*s6)-qdot3*(c3*s1+c1*c2*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-l7*qdot6*(c1*s2*s4*s6-c4*s1*s3*s6+c1*c2*c3*c4*s6+c1*c4*c5*c6*s2+c5*c6*s1*s3*s4-c1*c2*c3*c5*c6*s4)-qdot2*c1*(l4*c3*c4*s2-l4*c2*s4-l7*c2*c6*s4+l7*c3*c4*c6*s2+l7*c2*c4*c5*s6+l7*c3*c5*s2*s4*s6)+l7*qdot5*s5*s6*(s1*s3*s4+c1*c4*s2-c1*c2*c3*s4),l7*qdot6*c6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)-l7*qdot5*s6*(c3*s1*s5+c1*c2*s3*s5-c1*c5*s2*s4+c4*c5*s1*s3-c1*c2*c3*c4*c5)-l7*qdot1*s6*(c2*c5*s1*s3-c1*c3*c5+c1*c4*s3*s5+s1*s2*s4*s5+c2*c3*c4*s1*s5)-l7*qdot3*s6*(c5*s1*s3-c1*c2*c3*c5+c3*c4*s1*s5+c1*c2*c4*s3*s5)+l7*qdot4*s5*s6*(s1*s3*s4+c1*c4*s2-c1*c2*c3*s4)-l7*qdot2*c1*s6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5),qdot6*(l7*(s5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)-c5*(c1*c3-c2*s1*s3))*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))-l7*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3)*(s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+c6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3))))-qdot1*(l7*(s5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)-c5*(c3*s1+c1*c2*s3))*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4))+l7*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3)*(c6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)-s6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3))))+qdot3*(l7*c3*s1*s4*s6-l7*c6*s1*s3*s5+l7*c1*c2*c3*c6*s5+l7*c3*c4*c5*c6*s1+l7*c1*c2*s3*s4*s6+l7*c1*c2*c4*c5*c6*s3)-qdot4*(l7*c1*s2*s4*s6-l7*c4*s1*s3*s6+l7*c1*c2*c3*c4*s6+l7*c1*c4*c5*c6*s2+l7*c5*c6*s1*s3*s4-l7*c1*c2*c3*c5*c6*s4)+l7*qdot5*c6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)+l7*qdot2*c1*(c2*c4*s6-c2*c5*c6*s4+c3*s2*s4*s6-c6*s2*s3*s5+c3*c4*c5*c6*s2),0;
                  l7*qdot6*(s6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)+c6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3)))-qdot2*(l7*(c1*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+c1*c6*(c2*c4+c3*s2*s4))+l4*c1*(c2*c4+c3*s2*s4)+l2*c1*c2)-qdot1*(l4*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)+l7*(c6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)-s6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3)))-l2*s1*s2)-qdot3*(l7*(s6*(s5*(s1*s3-c1*c2*c3)-c4*c5*(c3*s1+c1*c2*s3))+c6*s4*(c3*s1+c1*c2*s3))+l4*s4*(c3*s1+c1*c2*s3))-qdot4*(l7*(c6*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+c5*s6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2))+l4*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4))-l7*qdot5*s6*(s5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)-c5*(c3*s1+c1*c2*s3)),qdot4*(l4*s1*(c2*s4-c3*c4*s2)+l7*s1*(c6*(c2*s4-c3*c4*s2)-c5*s6*(c2*c4+c3*s2*s4)))-qdot1*(l4*c1*(c2*c4+c3*s2*s4)+l2*c1*c2+l7*c1*(s6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)+c6*(c2*c4+c3*s2*s4)))+qdot2*(l4*s1*(c4*s2-c2*c3*s4)+l2*s1*s2+l7*s1*(s6*(c5*(s2*s4+c2*c3*c4)-c2*s3*s5)+c6*(c4*s2-c2*c3*s4)))-qdot3*(l7*s1*(s6*(c3*s2*s5+c4*c5*s2*s3)-c6*s2*s3*s4)-l4*s1*s2*s3*s4)-l7*qdot6*s1*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4))+l7*qdot5*s1*s6*(s5*(c2*s4-c3*c4*s2)-c5*s2*s3),qdot4*(c1*c3-c2*s1*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot3*(l4*c1*s3*s4+l4*c2*c3*s1*s4+l7*c1*c6*s3*s4-l7*c1*c3*s5*s6+l7*c2*c3*c6*s1*s4-l7*c1*c4*c5*s3*s6+l7*c2*s1*s3*s5*s6-l7*c2*c3*c4*c5*s1*s6)-qdot1*(l4*c3*s1*s4+l4*c1*c2*s3*s4+l7*c3*c6*s1*s4+l7*s1*s3*s5*s6+l7*c1*c2*c6*s3*s4-l7*c1*c2*c3*s5*s6-l7*c3*c4*c5*s1*s6-l7*c1*c2*c4*c5*s3*s6)+l7*qdot6*(c1*c6*s3*s5-c1*c3*s4*s6-c1*c3*c4*c5*c6+c2*c3*c6*s1*s5+c2*s1*s3*s4*s6+c2*c4*c5*c6*s1*s3)+qdot2*s1*s2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)+l7*qdot5*s6*(c1*c5*s3+c2*c3*c5*s1+c1*c3*c4*s5-c2*c4*s1*s3*s5),qdot1*(l4*c1*s2*s4-l4*c4*s1*s3+l4*c1*c2*c3*c4+l7*c1*c6*s2*s4-l7*c4*c6*s1*s3-l7*c1*c4*c5*s2*s6-l7*c5*s1*s3*s4*s6+l7*c1*c2*c3*c4*c6+l7*c1*c2*c3*c5*s4*s6)+qdot4*(l4*c4*s1*s2-l4*c1*s3*s4-l4*c2*c3*s1*s4+l7*c4*c6*s1*s2-l7*c1*c6*s3*s4-l7*c2*c3*c6*s1*s4+l7*c1*c4*c5*s3*s6+l7*c5*s1*s2*s4*s6+l7*c2*c3*c4*c5*s1*s6)+qdot3*(c1*c3-c2*s1*s3)*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-qdot2*s1*(l4*c3*c4*s2-l4*c2*s4-l7*c2*c6*s4+l7*c3*c4*c6*s2+l7*c2*c4*c5*s6+l7*c3*c5*s2*s4*s6)-l7*qdot6*(c1*c4*s3*s6+s1*s2*s4*s6+c2*c3*c4*s1*s6+c4*c5*c6*s1*s2-c1*c5*c6*s3*s4-c2*c3*c5*c6*s1*s4)-l7*qdot5*s5*s6*(c1*s3*s4-c4*s1*s2+c2*c3*s1*s4),l7*qdot1*s6*(c3*c5*s1+c1*c2*c5*s3+c1*s2*s4*s5-c4*s1*s3*s5+c1*c2*c3*c4*s5)+l7*qdot5*s6*(c1*c3*s5+c1*c4*c5*s3-c2*s1*s3*s5+c5*s1*s2*s4+c2*c3*c4*c5*s1)+l7*qdot6*c6*(c2*c5*s1*s3-c1*c3*c5+c1*c4*s3*s5+s1*s2*s4*s5+c2*c3*c4*s1*s5)+l7*qdot3*s6*(c1*c5*s3+c2*c3*c5*s1+c1*c3*c4*s5-c2*c4*s1*s3*s5)-l7*qdot4*s5*s6*(c1*s3*s4-c4*s1*s2+c2*c3*s1*s4)-l7*qdot2*s1*s6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5),l7*(qdot1*c1*c4*s2*s6-qdot5*c1*c3*c5*c6+qdot2*c2*c4*s1*s6+qdot1*c3*c6*s1*s5-qdot3*c1*c3*s4*s6+qdot3*c1*c6*s3*s5-qdot4*c1*c4*s3*s6+qdot6*c4*c6*s1*s2-qdot6*c1*c6*s3*s4+qdot6*c1*c3*s5*s6+qdot1*s1*s3*s4*s6-qdot4*s1*s2*s4*s6-qdot1*c1*c2*c3*s4*s6+qdot1*c1*c2*c6*s3*s5-qdot1*c1*c5*c6*s2*s4+qdot1*c4*c5*c6*s1*s3-qdot2*c2*c5*c6*s1*s4+qdot3*c2*c3*c6*s1*s5-qdot4*c2*c3*c4*s1*s6-qdot4*c4*c5*c6*s1*s2+qdot5*c2*c5*c6*s1*s3-qdot6*c2*c3*c6*s1*s4+qdot4*c1*c5*c6*s3*s4+qdot5*c1*c4*c6*s3*s5+qdot6*c1*c4*c5*s3*s6+qdot2*c3*s1*s2*s4*s6-qdot2*c6*s1*s2*s3*s5+qdot3*c2*s1*s3*s4*s6+qdot5*c6*s1*s2*s4*s5-qdot6*c2*s1*s3*s5*s6+qdot6*c5*s1*s2*s4*s6-qdot3*c1*c3*c4*c5*c6-qdot1*c1*c2*c3*c4*c5*c6+qdot2*c3*c4*c5*c6*s1*s2+qdot3*c2*c4*c5*c6*s1*s3+qdot4*c2*c3*c5*c6*s1*s4+qdot5*c2*c3*c4*c6*s1*s5+qdot6*c2*c3*c4*c5*s1*s6),0;
                  0,qdot4*(l4*s2*s4+l4*c2*c3*c4+l7*c6*s2*s4+l7*c2*c3*c4*c6-l7*c4*c5*s2*s6+l7*c2*c3*c5*s4*s6)-qdot2*(l2*c2+l4*c2*c4+l7*c2*c4*c6+l4*c3*s2*s4+l7*c3*c6*s2*s4+l7*c2*c5*s4*s6+l7*s2*s3*s5*s6-l7*c3*c4*c5*s2*s6)-l7*qdot6*(c2*c3*s4*s6-c4*s2*s6-c2*c6*s3*s5+c5*c6*s2*s4+c2*c3*c4*c5*c6)-qdot3*c2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)+l7*qdot5*s6*(s2*s4*s5+c2*c5*s3+c2*c3*c4*s5),l7*qdot6*s2*(s3*s4*s6+c3*c6*s5+c4*c5*c6*s3)-qdot2*c2*(l4*s3*s4+l7*c6*s3*s4-l7*c3*s5*s6-l7*c4*c5*s3*s6)-qdot3*s2*(l4*c3*s4+l7*c3*c6*s4+l7*s3*s5*s6-l7*c3*c4*c5*s6)-qdot4*s2*s3*(l4*c4+l7*c4*c6+l7*c5*s4*s6)+l7*qdot5*s2*s6*(c3*c5-c4*s3*s5),qdot2*(l4*s2*s4+l4*c2*c3*c4+l7*c6*s2*s4+l7*c2*c3*c4*c6-l7*c4*c5*s2*s6+l7*c2*c3*c5*s4*s6)-qdot4*(l4*c2*c4+l7*c2*c4*c6+l4*c3*s2*s4+l7*c3*c6*s2*s4+l7*c2*c5*s4*s6-l7*c3*c4*c5*s2*s6)+l7*qdot6*(c2*s4*s6+c2*c4*c5*c6-c3*c4*s2*s6+c3*c5*c6*s2*s4)-qdot3*s2*s3*(l4*c4+l7*c4*c6+l7*c5*s4*s6)-l7*qdot5*s5*s6*(c2*c4+c3*s2*s4),l7*qdot2*s6*(s2*s4*s5+c2*c5*s3+c2*c3*c4*s5)-l7*qdot5*s6*(s2*s3*s5+c2*c5*s4-c3*c4*c5*s2)+l7*qdot6*c6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5)-l7*qdot4*s5*s6*(c2*c4+c3*s2*s4)+l7*qdot3*s2*s6*(c3*c5-c4*s3*s5),l7*qdot4*(c2*s4*s6+c2*c4*c5*c6-c3*c4*s2*s6+c3*c5*c6*s2*s4)-l7*qdot6*(c2*c4*c6+c3*c6*s2*s4+c2*c5*s4*s6+s2*s3*s5*s6-c3*c4*c5*s2*s6)-l7*qdot2*(c2*c3*s4*s6-c4*s2*s6-c2*c6*s3*s5+c5*c6*s2*s4+c2*c3*c4*c5*c6)+l7*qdot5*c6*(c5*s2*s3-c2*s4*s5+c3*c4*s2*s5)+l7*qdot3*s2*(s3*s4*s6+c3*c6*s5+c4*c5*c6*s3),0];
        end
        
        % Modified: the DH parameters in the transmat function has been
        % changed to YuMi's DH Parameters
        function p=computeEnd_effectorPosition(obj)
            p0=[0;0;0;1];
            A=obj.Tbase * obj.transMat(obj.q(1),0,-1,obj.a1,obj.d1);
            A=A*obj.transMat(obj.q(2),0,1,obj.a2,0);
            A=A*obj.transMat(obj.q(3),0,-1,obj.a3,obj.d3);
            A=A*obj.transMat(obj.q(4),0,-1,obj.a4,0);
            A=A*obj.transMat(obj.q(5),0,-1,obj.a5,obj.d5);
            A=A*obj.transMat(obj.q(6),0,1,obj.a6,0);
            A=A*obj.transMat(obj.q(7),1,0,0,obj.d7);
            p=A*p0;
            p=p(1:3);
        end

        % New function to compute the position of joint 6 (the origin
        % of link 7 for the pointing task
        function p=computejoint6(obj)
            p0=[0;0;0;1];
            A=obj.Tbase * obj.transMat(obj.q(1),0,-1,obj.a1,obj.d1);
            A=A*obj.transMat(obj.q(2),0,1,obj.a2,0);
            A=A*obj.transMat(obj.q(3),0,-1,obj.a3,obj.d3);
            A=A*obj.transMat(obj.q(4),0,-1,obj.a4,0);
            A=A*obj.transMat(obj.q(5),0,-1,obj.a5,obj.d5);
            A=A*obj.transMat(obj.q(6),0,1,obj.a6,0);
            p=A*p0;
            p=p(1:3);
        end


        % Modified: the DH parameters in the transmat function has been
        % changed to YuMi's DH Parameters
        function [Ori,Ori_z]=computeEnd_effectorOrientation(obj)
            A=obj.Tbase * obj.transMat(obj.q(1),0,-1,obj.a1,obj.d1);
            A=A*obj.transMat(obj.q(2),0,1,obj.a2,0);
            A=A*obj.transMat(obj.q(3),0,-1,obj.a3,obj.d3);
            A=A*obj.transMat(obj.q(4),0,-1,obj.a4,0);
            A=A*obj.transMat(obj.q(5),0,-1,obj.a5,obj.d5);
            A=A*obj.transMat(obj.q(6),0,1,obj.a6,0);
            A=A*obj.transMat(obj.q(7),1,0,0,obj.d7);
            Ori=A(1:3,1:3);
            Ori_z=A(1:3,3);
        end
        
        function pdot=computeEnd_effectorPosVelocity(obj)
            J=obj.computeJacobianPos();
            pdot=J*obj.qdot;
        end
        
        function pdot=computeEnd_effectorOriVelocity(obj,zd)
            [~,ze]=obj.computeEnd_effectorOrientation;
            [~,Jori]=obj.computeJacobianOri(zd,ze);
            pdot=Jori*obj.qdot;
        end
        
        function A=transMat(~,q,ca,sa,a,d)
            sq=sin(q);
            cq=cos(q);
            A=sparse([cq, -sq*ca, sq*sa, a*cq; sq, cq*ca, -cq*sa, a*sq; 0, sa, ca, d; 0 0 0 1]);
        end

        % New function sets the origin of the base, to move the arm around
        % the workspace
        function Tbase = transl(~, x, y, z)
            Tbase = [
                1 0 0 x;
                0 1 0 y;
                0 0 1 z;
                0 0 0 1];
        end
        
        % Modified: the DH parameters in the transmat function has been
        % changed to YuMi's DH Parameters. Uses Tbase to set the origin
        function plotarm(obj)
            p0=[0;0;0;1];
            A1=obj.Tbase * obj.transMat(obj.q(1),0,-1,obj.a1,obj.d1);
            A2=A1*obj.transMat(obj.q(2),0,1,obj.a2,0);
            A3=A2*obj.transMat(obj.q(3),0,-1,obj.a3,obj.d3);
            A4=A3*obj.transMat(obj.q(4),0,-1,obj.a4,0);
            A5=A4*obj.transMat(obj.q(5),0,-1,obj.a5,obj.d5);
            A6=A5*obj.transMat(obj.q(6),0,1,obj.a6,0);
            A7=A6*obj.transMat(obj.q(7),1,0,0,obj.d7);
            p1=A1*p0;
            p2=A2*p0;
            p3=A3*p0;
            p4=A4*p0;
            p5=A5*p0;
            p6=A6*p0;
            p7=A7*p0;
            plot3([p0(1),p1(1)],[p0(2),p1(2)],[p0(3),p1(3)],'ok');
            plot3([p0(1),p1(1)],[p0(2),p1(2)],[p0(3),p1(3)],'Color','r');
            hold on
            axis([-1 1 -1 1 -1 1])
            plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'ok');
            plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'r');
            hold on
            plot3([p2(1),p3(1)],[p2(2),p3(2)],[p2(3),p3(3)],'ok');
            plot3([p2(1),p3(1)],[p2(2),p3(2)],[p2(3),p3(3)],'r');
             hold on
            plot3([p3(1),p4(1)],[p3(2),p4(2)],[p3(3),p4(3)],'ok');
             plot3([p3(1),p4(1)],[p3(2),p4(2)],[p3(3),p4(3)],'r');
             hold on
            plot3([p4(1),p5(1)],[p4(2),p5(2)],[p4(3),p5(3)],'ok');
             plot3([p4(1),p5(1)],[p4(2),p5(2)],[p4(3),p5(3)],'r');
             hold on
            plot3([p5(1),p6(1)],[p5(2),p6(2)],[p5(3),p6(3)],'ok');
            plot3([p5(1),p6(1)],[p5(2),p6(2)],[p5(3),p6(3)],'r');
             hold on
            plot3([p6(1),p7(1)],[p6(2),p7(2)],[p6(3),p7(3)],'ok');
            plot3([p6(1),p7(1)],[p6(2),p7(2)],[p6(3),p7(3)],'r');
             hold on
            %axis square;
            %axis([-3 3 -3 3]);
        end
    end
end