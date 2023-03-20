clc
clearvars
close all

%% constants
self.max_v_d = 50;
self.S = [0 -1; 1 0];
self.mu_s = 5;
self.mu_w = 0.05;
self.lambda_w = 1;
self.k1 = 0.1;
self.k2 = pi/180;
self.lambda = 0.95;
self.sigma = 100;
self.t1 = 15;
self.t2 = 45;
self.P = diag([1,10]);
self.ro = 200; %obstacle radius
self.L = 100;
self.delta = 10*self.L;
self.dt = 0.1;
self.s_0 = 0;
self.t_r = 1000000;
%% Initialization
po = [700; 000]; %obtsacle position
s = self.s_0;
p_r = get_pr(self,s);
p = p_r;
p_d = p_r;% + [100; -1000];
t_s = get_ts(s); % direction of the path is parallel to boat at start
z = t_s;
z_d = t_s;
v_d = 5;
v_r = 5;
w = 0;
%control input
u_d = [0; 0];
u = [0; 0];
% error init
e1 = (p_d - p_r)'*t_s;
e2 = (p_d - p_r)' * self.S *t_s;

%deltas
t_s_dot = get_ts_dot(s);
s_dot = 0;
w_dot = 0;
p_d_dot = 0;
v_d_dot = 0;
z_d_dot = 0;

self.t = 1;
self.t_t = 0;
self.t_tot = 600;
self.hist = zeros(20,1);

%% main loop
while self.t_t <= self.t_tot
    self.hist(1,self.t) = p_r(1);
    self.hist(2,self.t) = p_r(2);
    self.hist(3,self.t) = p_d(1);
    self.hist(4,self.t) = p_d(2);
    self.hist(5,self.t) = s;
    self.hist(6,self.t) = t_s(1);
    self.hist(7,self.t) = t_s(2);
    self.hist(8,self.t) = z_d(1);
    self.hist(9,self.t) = z_d(2);
    self.hist(10,self.t) = u_d(1);
    self.hist(11,self.t) = u_d(2);

    %get next point in path
    p_r = get_pr(self,s);
    t_s = get_ts(s);
    e1 = (p_d - p_r)'*t_s;
    e2 = (p_d - p_r)' * self.S *t_s;
    z_delta = get_z_delta(self, e2); 
    z_tilde = get_z_tilde(self, z_d, t_s, z_delta);
    w_delta = get_w_delta(self,v_d,e2);
    w_r = get_w_r(self, t_s, t_s_dot, v_r);
    k_d = get_ud(self, v_d, v_r, z_tilde, w_delta, w_r);

    p_e = get_p_e(p_d, po);
    c_ud_2 = linspace(-30,30,9);
    c_ud_1 = 0;
    c_ud_1 = linspace(-0.1,0.2,3);
    u_d = get_U_B2(self, v_d, v_r ,p_e, z_d, c_ud_1, c_ud_2, k_d);
    u_d = k_d;
    v_d_dot = u_d(1) * self.dt;
    z_d_dot = u_d(2) * self.S * z_d * self.dt;
    v_d = v_d + v_d_dot ;
    z_d = z_d + z_d_dot ;
    p_d_dot = v_d*z_d* self.dt;
    p_d = p_d + p_d_dot;
    z_d = get_k_z(self, t_s, z_delta);%+ z_d_dot ;
    s_dot =self.dt*get_s_dot(self, z_delta, v_r, e1, 0);
    s = s + arc2rad(s_dot, self.t_r);
    self.t_t = self.t_t + self.dt;
    self.t =self.t+1; 
end

get_plots(self, po)

%%Functions
% function y = get_k_B(ud, k)
% y = min()
% end
function y = get_U_B2_bar(self, v_d, v_r ,p_e, z_d, u_d, w, t_s)
min_u = [Inf; Inf];
B_1 = get_B_1(self, p_e);
B_2 = get_B_2_bar(p_e, v_d, v_r, z_d, w, t_s, B_1);

for c = 1:size(u_d,2)
    for d = 1:size(u_d,2)
        u_d_c1 = u_d(1,c);
        u_d_c2 = u_d(2,d);
        u_d_c = [u_d_c1; u_d_c2];
        B_2_dot = B_2_bar_dot(self, s ,t_s ,v_d, v_r ,p_e, z_d, u_d_c,B_1, w) ;
        C = -B_2 / self.t2;
        if (B_2_dot <= C &&  norm(u_d_c) < norm(min_u))
            min_u = u_d_c;
        end
    end
end
y= min_u;
end

function y = get_U_B2(self, v_d, v_r ,p_e, z_d, u_d_1, u_d_2, u_d)
min_u = [Inf; Inf];
B_2 = get_B_2(self, v_d, z_d, p_e, v_r);
for c = 1:size(u_d_1,2)
    for d = 1:size(u_d_2,2)
        u_d_c1 = u_d_1(c);
        u_d_c2 = u_d_2(d);
        u_d_c = [u_d_c1; u_d_c2];
        B_2_dot = get_B_2_dot(self, v_d, v_r ,p_e, z_d, u_d_c) ;
        C = -B_2 / self.t2;
        if (B_2_dot <= C)
            if (min_u(1) == Inf) || norm(u_d - u_d_c) < norm(u_d - min_u) 
                min_u = u_d_c;
            end
        end
    end
end
y= min_u;
end

function y = arc2rad(arclen, r)
y = arclen / r;

end

function y = get_z_tilde(self, z_d, t_s, z_delta)
kz = get_k_z(self, t_s, z_delta);
z_tilde = get_R_transform(self,kz)'*z_d;
y = z_tilde;
end

function y = get_w_delta(self,v_d,e2)
y = -(v_d*e2*self.delta)/(self.delta^2 + e2^2)^(3/2);
end

function y = get_w_r(self, t_s, t_s_dot, v_r)
%  y = - t_s' * self.S' * t_s_dot * v_r;
   y = - t_s' * t_s_dot * v_r;
end

function y = get_ud(self, v_d, v_r, z_tilde, w_delta, w_r)
ud_1 = -self.k1*(v_d - v_r);
ud_2 = -self.k2*(z_tilde(2))/sqrt(1 - (self.lambda^2)*(z_tilde(1)^2))+ w_delta + w_r;
y = [ud_1; ud_2];
end

function y = B_2_bar_dot(self, s ,t_s ,v_d, v_r ,p_e, z_d, u,B_1, w)
w_dot =  get_w_dot(self, w, p, p_d, t_s);
s_dot = get_f_s(self, z_delta, v_r, e1, w);
ts_dot = get_ts_dot(s);
y = (-(v_d + w)^2/norm(p_e))*((p_e'/norm(p_e))*self.S'*((v_d*z_d + w*t_s)/norm(v_d*z_d + w*t_s)))^2 - ...
    ((v_r*self.delta)/(self.delta^2 + B_1^2))*(p_e'/norm(p_e))*(v_d*z_d + t_s*w) - ...
    (p_e'/norm(p_e))*(t_s*w_dot + ts_dot*s_dot * w) -...
    (p_e'/norm(p_e))*[z_d, v_d*self.S*z_d]*u;
end

function y = get_B_2_bar(self, p_e, v_d, v_r, z_d, w, t_s, B_1)
alpha_1 = get_alpha_1_alt(self, B_1, v_r);
y = -(p_e'/norm(p_e))*(v_d*z_d + w*t_s) + alpha_1;
end

function y = get_w_dot(self, w, p, p_d, t_s)
y = -self.lambda_w*(w + self.mu_w*(p - p_d)'*t_s);
end

function y = get_B_2_dot(self, v_d, v_r ,p_e, z_d, u_d)
B_1 = get_B_1(self, p_e);
y = (-v_d^2/norm(p_e))*((p_e'/norm(p_e))*self.S'*z_d)^2 - ...
    ((v_r*self.sigma)/(self.sigma^2 + B_1^2))*(p_e'/norm(p_e))*v_d*z_d ...
    -(p_e'/norm(p_e))*[z_d, v_d*self.S*z_d]*u_d;
end

function y = get_B_2(self, v_d, z_d, p_e, v_r)
B_1 = get_B_1(self, p_e);
B_1_dot = get_B_1_dot(p_e, v_d, z_d);
alpha_1 = get_alpha_1_alt(self, B_1, v_r);
y = B_1_dot + alpha_1;
end

%better saturation function
function y = get_alpha_1_alt(self, phi, v_r)
y = v_r * atan(phi/self.sigma);
end

function y = get_B_1_dot(p_e, v_d, z_d)
y = - (p_e'/norm(p_e)) * v_d * z_d;
end

function y = get_p_e(p_d, p_o)
y = p_d - p_o;
end

function y = get_B_1(self, p_e)
y = self.ro - norm(p_e);
end

function y = get_V_dot(self, e1, e2, v_r)
y = -(v_r*e2^2/sqrt(self.delta^2 + e2^2)) - self.mu_s*e1^2;
end

function y = get_V(e)
y = (e'*e)/2;
end

function y = get_s_dot(self, z_delta, v_r, e1, w)
y = z_delta(1)*v_r + self.mu_s*e1 + w;
end

function y = get_k_z(self, t_s, z_delta)
y = get_R_transform(self,t_s)*z_delta;
end

function y = get_z_delta(self, e2)
y = (1 / sqrt(self.delta^2 + e2^2)) * [self.delta; -e2];
end

function y = get_p_d_dot(vd,zd,w,t_s)
y = vd*zd + w*t_s;
end

function y = get_ts(s)
y = [cos(s);sin(s)];
end

function y = get_ts_dot(s)
y = [-sin(s); cos(s)];
end

function y = get_pr(self, s)
y = self.t_r*[(sin(s) - sin(self.s_0)); (-cos(s) + cos(self.s_0))];
end

function y = get_e(self, t_s, p_d, p_r)
y = get_R_transform(self, t_s)' * (p_d - p_r);
end

function y = get_R_transform(self, z)
y = [z, self.S*z];
end

function circle(x,y,r)
ang=0:0.01:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp);
end

function get_plots(self, po)
figure('Name','trajectory');
circle(po(1), po(2), self.ro);
hold on
plot(self.hist(1,:),self.hist(2,:));
hold on
plot(self.hist(3,:),self.hist(4,:));
hold off

figure('Name','orientation');
angle_t_s = atan(self.hist(6,:)./ self.hist(7,:));
angle_t_s = rad2deg(angle_t_s);
plot(angle_t_s);
hold on
angle_z_d = atan(self.hist(8,:)./ self.hist(9,:));
angle_z_d = rad2deg(angle_z_d);
plot(angle_z_d);
hold off
legend({'t_s','z_d'},'Location','southwest')

figure('Name','inputs');
plot(self.hist(10,:));
hold on
plot(self.hist(11,:));
hold off
legend({'u_d_1','u_d_2'},'Location','southwest')
end