# -*- coding: utf-8 -*-
"""
Created on Fri May 13 10:43:42 2022

@author: mathima
"""

from casadi import *
import numpy as np
import matplotlib.pyplot as plt

plt.close('all')

x = SX.sym('x',2)
y = SX.sym('y')


T=20
N=40

x = SX.sym('x1',2)
u = SX.sym('u',1)


xdot = vertcat((1-x[1]**2)*x[0] - x[1] + u, x[0])

L = x[0]**2 + x[1]**2 + 0.1*u**2

M = 4
DT = T/N/M
f = Function('f', [x, u], [xdot,L])

X0 = MX.sym('X0', 2)
U = MX.sym('U')
X = X0
Q = 0
for j in range(0,M):
    [k1, k1_q] = f(X, U);
    [k2, k2_q] = f(X + DT/2 * k1, U)
    [k3, k3_q] = f(X + DT/2 * k2, U)
    [k4, k4_q] = f(X + DT * k3, U)
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)

F = Function('F', [X0, U], [X, Q], ['x0','p'], ['xf', 'qf'])

w = []
w0 = np.zeros(N)
lbw = np.zeros(N)-1
ubw = np.zeros(N)+1
J=0
g=[]
lbg = np.zeros(N)-1
ubg = np.zeros(N)+10


Xk = [0,1]

for k in range(0,N):
#    print('U'+str(k))
    Uk = MX.sym('U' +str(k))
    w = vertcat(w,Uk)
    
    Fk = F(x0=Xk,p=Uk)
    Xk = Fk['xf']
    J = J + Fk['qf']
    g = vertcat(g,Xk[0])


prob = {'f':J,'x':w,'g':g}

S = nlpsol('S','ipopt',prob)

sol = S(x0=w0,lbx=lbw,ubx=ubw,ubg=ubg,lbg=lbg)

uopt = np.ndarray.flatten(sol['x'].full())

xout = np.zeros((2,N+1))
x=[0,1]
xout[:,0] = x
for i in range(N):

    Fk = F(x0=x,p=uopt[i])
    x = np.ndarray.flatten(Fk['xf'].full())
    xout[:,i+1] = x


plt.plot(np.linspace(0,10,N+1),xout.T)
plt.step(np.linspace(0,10,N+1),np.append(uopt,0),where='post')
# =============================================================================
# 
# dt = 0.1
# N=8
# DT=dt/N
# 
# X0 = MX.sym('X0',1)
# U = MX.sym('U',1)
# X = X0
# 
# for j in range(0,N):
#     tmp1 = f(X,U)
#     tmp2 = f(X+DT/2*tmp1,U)    
#     tmp3 = f(X+DT/2*tmp2,U)
#     tmp4 = f(X+DT*tmp3,U)
#     X=X+DT/6*(tmp1+2*tmp2+2*tmp3+tmp4)
# 
# F = Function('F',[X0,U],[X])
# 
# u=1
# 
# 
# tmax=10
# tvec = np.linspace(0,tmax,int(tmax/dt)+1)
# xout = tvec*0
# csout = tvec*0
# x=-1
# cs=x
# 
# for i1,t in enumerate(tvec):
#     csout[i1] =cs
#     xout[i1] = x
#     u=-x**3+1    
#     for i in range(0,100):
# 
#         x += (-x+u)*dt/100
#     u=-cs**3+1
#     cs = F(cs,u)
# 
#  
# plt.plot(tvec,xout)
# plt.plot(tvec,csout,'--')
# 
# =============================================================================
