clc
clearvars

%% initialization
%self
x = 0;
y = 0;
psi = deg2rad(0);
v = 30;
vx = v * sin(psi);
vy = v * cos(psi);
r = 1;

%target
xt = 5;
yt = 6;
vt = 20;
vt_psi = deg2rad(-10);
vtx = vt * sin(vt_psi);
vty = vt * cos(vt_psi);

% relative velocities
vrx = vtx - vx;
vry = vty - vy;
vr = sqrt(vrx^2 + vry^2);

%% ARPA
d_at_cpa = abs((xt*vry - yt*vrx)/vr);
t_2_cpa = -(xt*vrx + yt*vry) / vr^2;

%self coords at cpa
x_at_cpa = x + vx * t_2_cpa;
y_at_cpa = y + vy * t_2_cpa;

% target coords at cpa
t_x_at_cpa = xt + t_2_cpa* vtx;
t_y_at_cpa = yt + t_2_cpa* vty;

if d_at_cpa < r
    t_2_r = [ (- r^2 + xt^2 + yt^2)/((r^2*vtx^2 - 2*r^2*vtx*vx + r^2*vty^2 - 2*r^2*vty*vy + r^2*vx^2 + r^2*vy^2 - vtx^2*yt^2 + 2*vtx*vty*xt*yt + 2*vtx*vx*yt^2 - 2*vtx*vy*xt*yt - vty^2*xt^2 - 2*vty*vx*xt*yt + 2*vty*vy*xt^2 - vx^2*yt^2 + 2*vx*vy*xt*yt - vy^2*xt^2)^(1/2) - vtx*xt + vx*xt - yt*(vty - vy));
        -(- r^2 + xt^2 + yt^2)/((r^2*vtx^2 - 2*r^2*vtx*vx + r^2*vty^2 - 2*r^2*vty*vy + r^2*vx^2 + r^2*vy^2 - vtx^2*yt^2 + 2*vtx*vty*xt*yt + 2*vtx*vx*yt^2 - 2*vtx*vy*xt*yt - vty^2*xt^2 - 2*vty*vx*xt*yt + 2*vty*vy*xt^2 - vx^2*yt^2 + 2*vx*vy*xt*yt - vy^2*xt^2)^(1/2) + vtx*xt - vx*xt + yt*(vty - vy))];

    avtd = vtx + xt/min(t_2_r);
    bvtd = vty + yt/min(t_2_r);

    % target coords at r
    t_x_at_r = xt + min(t_2_r)*vtx;
    t_y_at_r = yt + min(t_2_r)*vty;

    % self coords at r
    x_at_r = x + min(t_2_r)*vx;
    y_at_r = y + min(t_2_r)*vy;
end 

figure(1);
%origin
axis on
plot(0,0,'o')
hold on
plot(xt,yt,'x')
hold on

% plot at cpa
plot(x_at_cpa,y_at_cpa,'o')
hold on
plot(t_x_at_cpa,t_y_at_cpa,'x')
hold on
plot([x, x_at_cpa], [y, y_at_cpa])
hold on
plot([xt, t_x_at_cpa], [yt, t_y_at_cpa])
hold on
plot([x_at_cpa, t_x_at_cpa], [y_at_cpa, t_y_at_cpa],'--')
hold on

if d_at_cpa < r
    %plot at r
    plot(x_at_r,y_at_r,'o')
    hold on
    plot(t_x_at_r,t_y_at_r,'x')
    hold on
    plot([x, x_at_r], [y, y_at_r],'.')
    hold on
    plot([xt, t_x_at_r], [yt, t_y_at_r],'.')
    hold on
    plot([x_at_r, t_x_at_r], [y_at_r, t_y_at_r],'--')
    hold on
    circle(t_x_at_r,t_y_at_r,r);
    hold on
    circle(t_x_at_r,t_y_at_r,r);
end

function circle(x,y,r)
ang=0:0.01:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp);
end

