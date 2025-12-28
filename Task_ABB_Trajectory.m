% New function defines bang bang trajectory planning for a circle in 3D space

function [p,dp,ddp] = Task_ABB_Trajectory(t)

cx=0; r=0.2; cy=0.6;

thetai = 0;
thetaf = 2*pi;

L = 2 * (r*(thetaf - thetai));   % circumference of the circle

a_max = 1;
v_max = sqrt(L*a_max);

Tf = (L * a_max + v_max^2) /(a_max*v_max);
Ts = 0.5*Tf;


if t <= Ts
    sigma=0.5*a_max*t*t;
    dsigma=a_max*t;
    ddsigma=a_max;
    s = sigma/L;

elseif t <= Tf
    sigma =-0.5*a_max*(t-Tf)^2 + v_max*Tf - v_max^2/a_max;
    dsigma = a_max*(Tf - t);
    ddsigma = -a_max;
    s=sigma/L;

else
    sigma= L; %stop regardless of time
    dsigma=0;
    ddsigma=0;
    s = sigma/L;
end


theta = thetai + s*(thetaf - thetai);
dtheta = (dsigma/L) * (thetaf - thetai);
ddtheta= (ddsigma/L) * (thetaf - thetai);


ux = 0.70710678;
uz = 0.70710678;

px = cx + r*ux*cos(theta);
py = cy + r*sin(theta);
pz = r*uz*cos(theta);

dpx = -r*ux*sin(theta)*dtheta;
dpy =  r*cos(theta)*dtheta;
dpz = -r*uz*sin(theta)*dtheta;

ddpx = -r*ux*cos(theta)*(dtheta^2) - r*ux*sin(theta)*ddtheta;
ddpy = -r*sin(theta)*(dtheta^2)    + r*cos(theta)*ddtheta;
ddpz = -r*uz*cos(theta)*(dtheta^2) - r*uz*sin(theta)*ddtheta;

p = [px;py;pz];
dp = [dpx;dpy;dpz];
ddp = [ddpx;ddpy;ddpz];

end
