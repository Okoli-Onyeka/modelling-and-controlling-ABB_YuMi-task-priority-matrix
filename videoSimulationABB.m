%      Modified by Onyeka Franklin Okoli on 15/11/2025
%      Original code from Maram Khatib task-priority-relaxed-constraints
%      Modifications:
%                    - Displays moving obstacles
%                    - now calls YuMi functions
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


set(0, 'defaultTextInterpreter', 'none'); 
hfig=figure;
set(gcf,'Position',[500 0 500 500])

q=QV;
samples=size(q,1);

T=0.001;
tt=0:T:samples*T-T;

ee0=T0d; %end effector position
error_ee=e0; %end effector position error
error_eeori=e1;

subplot(3,1,1:2)

p=zeros(63,3);
k=1;
for ts=0:0.1:2*pi+0.1
    [p(k,:),~,~] = Task_ABB(ts);
    k=k+1;
end
plot3(p(:,1),p(:,2),p(:,3),'g','LineWidth',1);
hold on

YuMi=get_ABBPoints(q(1,:));

YuMix=YuMi(1,:);
YuMiy=YuMi(2,:);
YuMiz=YuMi(3,:);
hPlotRobot1=plot3(YuMix,YuMiy,YuMiz,'o-k','LineWidth',1);
set(hPlotRobot1,'XDataSource','YuMix')
set(hPlotRobot1,'YDataSource','YuMiy')
set(hPlotRobot1,'ZDataSource','YuMiz')
hold on

ee_X=ee0(1,1);
ee_Y=ee0(1,2);
ee_Z=ee0(1,3);
hPlotRH=plot3(ee_X,ee_Y,ee_Z,'r','LineWidth',2);
set(hPlotRH,'XDataSource','ee_X')
set(hPlotRH,'YDataSource','ee_Y')
set(hPlotRH,'ZDataSource','ee_Z')
hold on

%draw the cone for pointing task
Ze=Zd;
Ze=Ze/norm(Ze);
Pe=Pe1(1,:);
H = 0.036; %// height
R1 = H*tan(deg2rad(alpha_d)); %// radius
N = 100; %// number of points to define the circu reference
r1=Pe;
r2=Pe+Ze*H;
[x,y,z]=cylinder2P([0,R1],N,r1,r2);
drawCone=mesh(x, y, z);
set(drawCone,'XDataSource','x');
set(drawCone,'YDataSource','y');
set(drawCone,'ZDataSource','z');
hold on 

%moving obstacles
if movingObs
    hObs = gobjects(numberOfObstacles,1);
    for k=1:numberOfObstacles
        hObs(k) = plot3(P_obs(k,1,1),p_obs(k,2,1),p_obs(k,3,1),'og','LineWidth',5);
    end
else
    for k=1:1:size(p_obs,1)
        plot3(p_obs(k,1),p_obs(k,2),p_obs(k,3),'og','LineWidth',5);
    end
    hold on 
end



grid on
xlabel('X','FontSize',10);
ylabel('Y','FontSize',10);
zlabel('Z','FontSize',10);

az=128;
el=7;
view([az,el]);
axis manual
zlim([-.5,.5])
ylim([0,1])
xlim([-0.6,0.4])
axis square 

subplot(3,1,3)
TT=tt(1);
E_ee=error_ee(1);
E_eeori=error_eeori(1);
hplotEP=plot(2*TT,E_ee,'r','LineWidth',1.2);
hold on
set(hplotEP,'XDataSource','TT');
set(hplotEP,'YDataSource','E_ee')
hplotER=plot(2*TT,E_eeori,'b','LineWidth',1.2);
set(hplotER,'XDataSource','TT');
set(hplotER,'YDataSource','E_eeori')

grid on
xlabel('time [s]')
s=sprintf('task errors');
ylabel(s)
legend('pos','ori')
set(gca,'LineWidth',1.2,'FontSize',10)
ylim([-.05,0.05]);
xlim([0,2*Ttot]);

% vidObj = VideoWriter('abb_acceleration_moving_obstacle.mp4','MPEG-4'); %to save video
% vidObj.FrameRate = 30;
% open(vidObj);%to save video

f=1;
for i=1:samples

    if (mod(i,40)==1)  %15 for video, other 40
        
        if movingObs
            % Update moving obstacles
            for k = 1:numberOfObstacles
                set(hObs(k), 'XData', P_obs(k,1,i), 'YData', P_obs(k,2,i), 'ZData', P_obs(k,3,i));
            end
        end

        YuMi=get_ABBPoints(q(i,:));
        YuMix=YuMi(1,:);
        YuMiy=YuMi(2,:);
        YuMiz=YuMi(3,:);
        refreshdata(hPlotRobot1)
        
        ee_X=ee0(1:i,1);
        ee_Y=ee0(1:i,2);
        ee_Z=ee0(1:i,3);
        refreshdata(hPlotRH)
        
        %draw the cone for pointing task
        Ze=Zd;
        Ze=Ze/norm(Ze);
        Pe=Pe1(i,:);
        H = 0.036; 
        R2 = H*tan(deg2rad(alpha_d)); 
        N = 100; 
        r1=Pe;
        r2=Pe+Ze*H;
        [x,y,z]=cylinder2P([0,R2],N,r1,r2);
        refreshdata(drawCone)

        robot.set_q_and_qdot(q(i,:),[]);
        

        subplot(3,1,1:2)
       
        TT=2*tt(1:i);
        
        E_ee=error_ee(1:i);
        E_eeori=error_eeori(1:i);
        refreshdata(hplotEP)        
        refreshdata(hplotER)        

        M(f)=getframe(hfig);
        % writeVideo(vidObj,M(f));%to save video
       
        f=f+1;
    end

end
% close(vidObj);%to save video

disp('VIDEO PERFORMED') 

