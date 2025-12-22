%      Modified by Onyeka Franklin Okoli on 15/11/2025
%      Original code from Maram Khatib task-priority-relaxed-constraints
%      Modifications:
%                    - defines a circular task in 3D space
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


function [p,dp,ddp] = Task_ABB(lamda)

    cx=0; r=0.2; cy=0.6;

    ux = 0.70710678;
    uz = 0.70710678;
    
    px = cx + r*ux*cos(lamda);
    py = cy + r*sin(lamda);
    pz = r*uz*cos(lamda);
    
    dpx = -r*ux*sin(lamda);
    dpy =  r*cos(lamda);
    dpz = -r*uz*sin(lamda);
    
    ddpx = -r*ux*cos(lamda);
    ddpy = -r*sin(lamda);
    ddpz = -r*uz*cos(lamda);
    
    p   = [px;py;pz];
    dp  = [dpx;dpy;dpz];
    ddp = [ddpx;ddpy;ddpz];

end