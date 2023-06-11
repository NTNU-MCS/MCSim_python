clc
clearvars
close all

%% constants
self.S = [0 -1; 1 0];
self.k1 = 1;
self.lambda = 0.5;
self.dq = 200;
self.dt = 0.1;
self.gamma_2 = 70;
self.gamma_1 = .2;
self.t = 1;
self.t_t = 0;
self.t_tot = 600;
self.hist = zeros(20,1);
self.arpa = [];
self.max_rd = deg2rad(10);

%% initialize
rd = 0;
p = [0; 0];
ut = 6;
u = ut;
psi = deg2rad(-90);
z = [sin(psi); cos(psi)];
v = norm(u*z);
tq = [sin(psi); cos(psi)];
po = [-1505 -700; 10 100];
self.n_po= size(po,2);
uo = [2 1];
psio = [deg2rad(130) deg2rad(180)];
zo = [sin(psio); cos(psio)];
[ux, uy] = get_u_xy(z,u);
[uo_x, uo_y] = get_uo_xy(zo,uo);

[d_at_cpa, t_2_cpa, p_at_cpa, po_at_cpa, t_2_r ,p_at_r, po_at_r] = get_arpa(self, ux, uy, uo_x, uo_y, p, po);
self.arpa = [d_at_cpa; t_2_cpa; p_at_cpa; po_at_cpa; t_2_r; p_at_r; po_at_r];
figure("Name",'arpa')
plot_arpa(self, p, po)
hold on

%% main loop
while self.t_t <= self.t_tot
    self.hist(1:2,self.t) = p;
    self.hist(3:4,self.t) = z;
    self.hist(5,self.t) = rd;
    self.hist(6: 5 + self.n_po*2, self.t) = reshape(po,[self.n_po*2,1]);

    %% find the closest target
    dist = vecnorm((p - po),2,1);
    [~, closest] = min(dist);

    %% find rd
    rdc = get_nominal_control(self, z, tq);
    ei = p - po(:,closest);
    ui = uo(closest);
    zi = zo(:,closest);
    B1 = self.dq - norm(ei);
    LfB1 = - (ei'*(u*z - ui*zi))/norm(ei);
    B2 = LfB1 + (1/self.gamma_1)*B1;
    LfB2 = ((ei'*(u*z - ui*zi))^2)/norm(ei)^3 - (norm(u*z - ui*zi)^2)/norm(ei) + (1/self.gamma_1)*LfB1;
    LgB2 = (-u*ei'*self.S*z)/norm(ei);
    B2_dot = LfB2 + LgB2*rdc;

    if B2_dot <= -(1/self.gamma_2)*B2
        rd = rdc;
    else
        a = LfB2 + LgB2*rdc + (1/self.gamma_2)*B2;
        b = LgB2;
        rd = rdc;% - (a*b')/b*b'; 
    end

    if rd > self.max_rd
            rd = self.max_rd;
    elseif rd < -self.max_rd
            rd = -self.max_rd;
    end
    
    p = p + u*z*self.dt;
    po = po + zo.*uo*self.dt;
    z = z + self.S*z*rd*self.dt;
    z = z / norm(z);
    self.t_t = self.t_t + self.dt;
    self.t =self.t+1;
end
get_plots(self, size(self.hist, 2))
animate(self, p, po)

function [d_at_cpa, t_2_cpa, p_at_cpa, po_at_cpa, t_2_r ,p_at_r, po_at_r] = get_arpa(self, ux, uy, uo_x, uo_y, p, po)

d_at_cpa = zeros(1, self.n_po);
t_2_cpa = zeros(1, self.n_po);
x_at_cpa = zeros(1, self.n_po);
y_at_cpa = zeros(1, self.n_po);
t_x_at_cpa = zeros(1, self.n_po);
t_y_at_cpa = zeros(1, self.n_po);
t_2_r = inf(1, self.n_po);
t_x_at_r = inf(1, self.n_po);
t_y_at_r = inf(1, self.n_po);
x_at_r = inf(1, self.n_po);
y_at_r = inf(1, self.n_po);

for i = 1:self.n_po
    urx = uo_x(i) - ux;
    ury = uo_y(i) - uy;
    ur = sqrt(urx^2 + ury^2);
    d_at_cpa(i) = abs((po(1,i)*ury - po(2,i)*urx)/ur);
    t_2_cpa(i) = -(po(1,i)*urx + po(2,i)*ury) / ur^2;
    %self coords at cpa
    x_at_cpa(i) = p(1) + ux * t_2_cpa(i);
    y_at_cpa(i) = p(2) + uy * t_2_cpa(i);
    % target coords at cpa
    t_x_at_cpa(i) = po(1,i) + t_2_cpa(i)* uo_x(i);
    t_y_at_cpa(i) = po(2,i) + t_2_cpa(i)* uo_y(i);

    % time  to dq
    if d_at_cpa(i) < self.dq
        R = self.dq;
        vrx = urx;
        vry = ury;
        vr = ur;
        x = po(1,i) ;
        y = po(2,i) ;
        L = sqrt(x^2 + y^2); 
        A = 2*(x*vrx+y*vry);
        B = L^2 - R^2;
%         syms t2r
%         eq = R == sqrt(L^2 + vr^2*t2r^2+2*(x*vrx + y*vry)*t2r);
%         t_2_r(i) = min(solve(eq, t2r))
         s1 = (-A + sqrt(A^2 - 4*vr^2*B))/2*vr^2;
         s2 = (-A - sqrt(A^2 - 4*vr^2*B))/2*vr^2;
         t_2_r(i) =  min([s1, s2]); 
        % target coords at r
        t_x_at_r(i) = po(1,i) + t_2_r(i)*uo_x(i);
        t_y_at_r(i) = po(2,i) + t_2_r(i)*uo_y(i);

        % self coords at r
        x_at_r(i) = p(1) + min(t_2_r)*ux;
        y_at_r(i) = p(2) + min(t_2_r)*uy;
    end
    p_at_cpa=[x_at_cpa; y_at_cpa];
    po_at_cpa=[t_x_at_cpa; t_y_at_cpa];
    p_at_r=[x_at_r; y_at_r];
    po_at_r=[t_x_at_r; t_y_at_r];
end
end

function plot_arpa(self,p, po)
% for j = 0 : self.n_po * (size(self.arpa, 2)/self.n_po) %go through histories
for i = 1: self.n_po %go through columns
    plot([p(1), self.arpa(3,i)], [p(2), self.arpa(4,i)])
    hold on
    plot([po(1, i), self.arpa(5,i)], [po(2, i), self.arpa(6,i)])
    hold on
    plot([self.arpa(3,i),self.arpa(5,i)], [self.arpa(4,i),self.arpa(6,i)], '--')
    hold on
    if self.arpa(8,i) ~= inf
        plot([p(1), self.arpa(8,i)], [p(2), self.arpa(9,i)])
        hold on
        plot([po(1, i), self.arpa(10,i)], [po(2, i), self.arpa(11,i)])
        hold on
        circle(self.arpa(10,i), self.arpa(11,i), self.dq);
        hold on
    end
end
end

function [ux, uy] = get_u_xy(z,u)
ux = u * z(1);
uy = u * z(2);
end

function [uo_x, uo_y] = get_uo_xy(zo,uo)
y = zeros([2, size(zo,2)]);
for i = 1:size(zo,2)
    y(:,i) = zo(:,i).*uo(i);
end
uo_x = y(1,:);
uo_y = y(2,:);
end

function y = get_nominal_control(self, z, tq)
z_tilde = [tq self.S*tq]'*z;
y = (-self.k1 * z_tilde(2)) / sqrt(1 - self.lambda^2 * z_tilde(1)^2);
end

function circle(x,y,r)
ang=0:0.01:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp);
end

function get_plots(self, tot)
% figure('Name','position');

plot(self.hist(1,1:tot),self.hist(2,1:tot));
hold on
for i = 1:self.n_po
    plot(self.hist(5+(i*2-1),1:tot),self.hist(5+(i*2),1:tot));
    hold on
    circle(self.hist(5+(i*2-1),tot),self.hist(5+(i*2),tot), self.dq);
    hold on
end
hold off

figure('Name','orientation');
angle_t_s = vecnorm(self.hist(3:4,1:tot), 2, 1);
plot(angle_t_s);

figure('Name','input');
plot(self.hist(5,1:tot));
end

function y = animate(self, p, po)
writerObj = VideoWriter('advectionOut.avi');
writerObj.FrameRate = 10;
open(writerObj);
fid = figure;
set(gcf,'Position',[0 0 440 400])
for t = 1: 10 :size(self.hist,2)
    figure(fid); 
%     plot_arpa(self, p, po)
%     hold on
    plot(self.hist(1,1:t),self.hist(2,1:t));
    hold on
    for i = 1:self.n_po
        plot(self.hist(5+(i*2-1),1:t),self.hist(5+(i*2),1:t));
        hold on
        circle(self.hist(5+(i*2-1),t),self.hist(5+(i*2),t), self.dq);
        hold on
    end
    frame = getframe(gcf) ;
    writeVideo(writerObj, frame);
    hold off
end
% close the writer object
close(writerObj);
end