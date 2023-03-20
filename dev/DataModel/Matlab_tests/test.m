clc
clearvars
syms tq p po u z g1 dq tq zo uo g1 S
B1 = dq-abs(tq*(p-po));
B1_dot = -tq'*((p - po)/ norm(p-po));
f = get_f(z, zo, u, uo);
g = [0;0;S*z];
B1_dot_f = B1_dot * f;
B2 = B1_dot_f + u * atan(B1*g1);
B2_dot = [
    diff(B2(1), p)*p + diff(B2(1), po)*po + diff(B2(1), z)*z;
    diff(B2(2), p)*p + diff(B2(2), po)*po + diff(B2(2), z)*z;
    diff(B2(3), p)*p + diff(B2(3), po)*po + diff(B2(3), z)*z];
B2_dot_f = B2_dot.* f;
B2_dot_g = B2_dot.* g

function y = get_f(z, zo, u, uo)
a = z*u;
b = zo*uo;
y = [a;b;0];
end
