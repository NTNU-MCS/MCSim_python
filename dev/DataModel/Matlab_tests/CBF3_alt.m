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
        rd = rdc - (a*b')/b*b'; 
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
animate(self)

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