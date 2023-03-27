clc
clearvars
close all

%% constants
self.S = [0 1; -1 0];
self.k1 = 1;
self.lambda = 0.5;
self.P = diag([1,10]);
self.dq = 200;
self.dt = 0.1;
self.gamma_2 = 1.5;
self.sigma = 27.0;
self.t = 1;
self.t_t = 0;
self.t_tot = 200;
self.hist = zeros(20,1);
self.arpa = [];

%% initialize
rd = 0;
p = [0; 0];
u = 10;
psi = deg2rad(55);
z = [sin(psi); cos(psi)];
tq = [sin(psi); cos(psi)];
[ux, uy] = get_u_xy(z,u);

po = [1150 1500; 600 -300];
self.n_po= size(po,2);
uo = [10 10];
psio = [deg2rad(-90), deg2rad(-40)];
zo = [sin(psio); cos(psio)];
[uo_x, uo_y] = get_uo_xy(zo,uo);


[fa, fb] = get_f(z, zo, u, uo);
g = get_g(self, z);

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

    rd = real(get_nominal_control(self, z, tq));

    ub = [rd, linspace(-1,1,31)];
    %% find rd
    min_ub = Inf;
    pe = p-po;
    B1 = get_B1(self, pe);
    B1_dot = get_B1_dot(pe, u, z);
    alpha_1 = u*atan(B1/self.sigma);
    B2 = B1_dot + alpha_1;
    for c = 1:size(ub,2)
        rdc = ub(c);
        B2_dot = get_B2_dot(self, pe, z, B1, u, rdc);
        if (all(B2_dot <= -self.gamma_2 * B2))
            if (min_ub) == Inf || norm(rd - rdc) < norm(rd - min_ub)
                min_ub = rdc;
            end
        end
    end
    rd= min_ub;
    [fa, fb] = get_f(z, zo, u, uo);
    g = get_g(self, z);

    p = p + fa*self.dt;
    po = po + fb*self.dt;
    z = z + g(5:6)*rd*self.dt;

    self.t_t = self.t_t + self.dt;
    self.t =self.t+1;
end
% [d_at_cpa, t_2_cpa, p_at_cpa, po_at_cpa, t_2_r ,p_at_r, po_at_r] = get_arpa(self, ux, uy, uo_x, uo_y, p, po);
% self.arpa = [d_at_cpa; t_2_cpa; p_at_cpa; po_at_cpa; t_2_r; p_at_r; po_at_r];
% plot_arpa(self, p, po)
hold on
get_plots(self, size(self.hist, 2))
animate(self)

%% functions
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
        tt_2_r = [
            (- self.dq^2 + po(1,i)^2 + po(2,i)^2)/((self.dq^2*uo_x(i)^2 - ...
            2*self.dq^2*uo_x(i)*ux + self.dq^2*uo_y(i)^2 - ...
            2*self.dq^2*uo_y(i)*uy + self.dq^2*ux^2 + self.dq^2*uy^2 - ...
            uo_x(i)^2*po(2,i)^2 + 2*uo_x(i)*uo_y(i)*po(1,i)*po(2,i) + ...
            2*uo_x(i)*ux*po(2,i)^2 - 2*uo_x(i)*uy*po(1,i)*po(2,i) - ...
            uo_y(i)^2*po(1,i)^2 - 2*uo_y(i)*ux*po(1,i)*po(2,i) + ...
            2*uo_y(i)*uy*po(1,i)^2 - ux^2*po(2,i)^2 + 2*ux*uy*po(1,i)*po(2,i) - ...
            uy^2*po(1,i)^2)^(1/2) - uo_x(i)*po(1,i) + ux*po(1,i) - ...
            po(2,i)*(uo_y(i) - uy));
            -(- self.dq^2 + po(1,i)^2 + po(2,i)^2)/((self.dq^2*uo_x(i)^2 - ...
            2*self.dq^2*uo_x(i)*ux + self.dq^2*uo_y(i)^2 - ...
            2*self.dq^2*uo_y(i)*uy + self.dq^2*ux^2 + self.dq^2*uy^2 - ...
            uo_x(i)^2*po(2,i)^2 + 2*uo_x(i)*uo_y(i)*po(1,i)*po(2,i) + ...
            2*uo_x(i)*ux*po(2,i)^2 - 2*uo_x(i)*uy*po(1,i)*po(2,i) - ...
            uo_y(i)^2*po(1,i)^2 - 2*uo_y(i)*ux*po(1,i)*po(2,i) + ...
            2*uo_y(i)*uy*po(1,i)^2 - ux^2*po(2,i)^2 + 2*ux*uy*po(1,i)*po(2,i) - ...
            uy^2*po(1,i)^2)^(1/2) + uo_x(i)*po(1,i) - ux*po(1,i) + ...
            po(2,i)*(uo_y(i) - uy))];
        t_2_r(i) =  min(tt_2_r);
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

function y = get_B2_dot(self, pe, z, B1, u, rdc)
y = zeros([1, size(pe,2)]);
for i = 1:size(pe,2)
    e = pe(:,i)'/norm(pe(:,i));
    y(i) = -(u^2/norm(pe(:,i)))*(e*self.S'*z)^2 - (u*self.sigma/(self.sigma^2 + B1(i)^2))*e*u*z - e*[z, u*self.S*z]*[0;rdc];
end
end

function y = get_B1_dot(pe, u, z)
y = zeros([1, size(pe,2)]);
for i = 1:size(pe,2)
    y(i) = -(pe(:,i)'/norm(pe(:,i)))*u*z ;
end
end

function y = get_B1(self, pe)
y = zeros([1, size(pe,2)]);
for i = 1:size(pe,2)
    y(i) = self.dq-norm(pe(:,i));
end
end

function y = get_nominal_control(self, z, tq)
z_tilde = [tq self.S*tq]'*z;
y = (-self.k1 * z_tilde(2)) / sqrt(1 - self.lambda^2 * z_tilde(1)^2);
end

function y = get_g(self, z)
y = [zeros(4,1);self.S*z];
end

function [p_dot, po_dot] = get_f(z, zo, u, uo)
p_dot = z*u;
po_dot = zo.*uo;
end

function circle(x,y,r)
ang=0:0.01:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp);
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
% end

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
angle_t_s = atan(self.hist(3,1:tot)./ self.hist(4,1:tot));
angle_t_s = rad2deg(angle_t_s);
plot(angle_t_s);

% figure('Name','B');
% subplot(3,1,1);
% plot(self.hist(8,:));
%
% subplot(3,1,2);
% plot(self.hist(9,:));
%
% subplot(3,1,3);
% plot(self.hist(10,:));
% hold on
% plot(self.hist(11,:));
% hold off
% legend({'B2_dot','-self.gamma_2 * B2'},'Location','southwest')


figure('Name','input');
plot(self.hist(5,1:tot));
end

function y = animate(self)
writerObj = VideoWriter('advectionOut.avi');
writerObj.FrameRate = 10;
open(writerObj);
fid = figure;
set(gcf,'Position',[0 0 440 400])
for t = 1: 10 :size(self.hist,2)
    figure(fid);

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