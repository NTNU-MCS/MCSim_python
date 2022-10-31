# -*- coding: utf-8 -*-
"""
Created on Fri May 13 10:43:42 2022

@author: mathima
"""

from casadi import *
import numpy as np
import matplotlib.pyplot as plt
import pickle

import os
from pathlib import Path
import sys
import time

#plt.close('all')

par_path = str(Path(os.path.dirname(__file__)).parents[1])  
library_path = par_path + '\\lib'
sys.path.append(library_path)
model_path = par_path+'\\models\\RVG_maneuvering'
sys.path.append(model_path)

#vessel data
parV = pickle.load( open(model_path+"\\data\\parV_RVG6DOF.pkl", "rb" ) )

#actuator data
parA = pickle.load( open(model_path+"\\data\\parA_RVG.pkl", "rb" ) )

# =============================================================================
# Create differential equations
# =============================================================================


p = SX.sym('p',2)
psi = SX.sym('psi',1)
phi = SX.sym('phi',1)
nu = SX.sym('nu',4) #u,v,r,p
alpha = SX.sym('alpha',1)
Ft=1.8*160**2#Ft = SX.sym('Ft',1)
u = SX.sym('u',1)
pdot = vertcat(nu[0]*cos(psi)-nu[1]*sin(psi),nu[0]*sin(psi)+nu[1]*cos(psi))

psidot = nu[2]
phidot = nu[3]

M6 = parV['Ma']+parV['Mrb']

M=np.zeros((4,4))        
M[0,0] = M6[0,0]
M[1,1] = M6[1,1]
M[2,2] = M6[5,5]
M[3,3] = M6[3,3]
M[1,3] = M6[1,3]  
M[3,1] = M6[3,1] 
M[1,2] = M6[1,5]
M[2,1] = M6[5,1]


M=np.round(M)

D6 = parV['Dl']

D=np.zeros((4,4))        
D[0,0] = D6[0,0]+parV['Du'][0,0]*5
D[1,1] = D6[1,1]+parV['dvv']*2
D[2,2] = D6[5,5]+parV['Dr'][5,5]*np.pi*2/180
D[3,3] = D6[3,3]
D[1,3] = D6[1,3]*4
D[3,1] = D6[3,1]*4
D[1,2] = D6[1,5]
D[2,1] = D6[5,1]

K = np.zeros(4)
K6 = parV['K']
K[3]=K6[3,3]

Fcorr = vertcat(M[1,1]*nu[1]*nu[2],
                -M[0,0]*nu[0]*nu[2],
                (M[0,0]-M[1,1])*nu[0]*nu[1],
                0)

rt = parA['rt']



Fact = vertcat(Ft*cos(alpha),
               Ft*sin(alpha),
               Ft*sin(alpha)*rt[0],
               -Ft*sin(alpha)*rt[2])

nudot = np.linalg.inv(M)@(Fcorr-D@nu-K@phi+Fact)

fx = vertcat(pdot,psidot,phidot,nudot,0)

gx = vertcat((0,0,0,0,0,0,0,0,1))

x=vertcat(p,psi,phi,nu,alpha)


T=60
N=T#10#2*T

M = 1
DT = T/N/M

L=5*(x[2]+np.pi/2)**2+x[3]**2+u**2

L=x[1]

f = Function('f', [x, u], [fx+gx*u,L])
 
X0 = MX.sym('X0', 9)
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
    

F = Function('F', [X0, U], [X, Q], ['x0','u'], ['xf', 'qf'])
 

w = []
lbw = []
ubw = []


J=0
g=[]
lbg = []#np.zeros(3*N)-inf
ubg = []#np.zeros(3*N)
S = MX.sym('S',1)


x0 = MX.sym('x0',9)

Xk = x0
xmax=0

u0=MX.sym('u0')
p0 = vertcat(x0,u0)

for k in range(0,N):
#    print('U'+str(k))
    Uk = MX.sym('U' +str(k))
    w = vertcat(w,Uk)
    lbw = vertcat(lbw,-np.pi/20)
    ubw = vertcat(ubw,np.pi/20)
    
    Fk = F(x0=Xk,u=Uk)
    Xk_end = Fk['xf']
    J = J + Fk['qf']+(Uk-u0)**2*0
    
    
    Xk = MX.sym('X' +str(k),9)    
    w = vertcat(w,Xk)
    
    lbw = vertcat(lbw,np.zeros(9)-inf)
    ubw = vertcat(ubw,np.zeros(9)+inf)
    
    g = vertcat(g,Xk-Xk_end)
    lbg = vertcat(lbg,np.zeros(9))
    ubg = vertcat(ubg,np.zeros(9))
    
    g = vertcat(g,Xk_end[3]-S)
    lbg = vertcat(lbg,-inf)
    ubg = vertcat(ubg,30*np.pi/180)

    g = vertcat(g,-Xk_end[3]-S)
    lbg = vertcat(lbg,-inf)
    ubg = vertcat(ubg,30*np.pi/180)    


    g = vertcat(g,Xk[8])
    lbg = vertcat(lbg,-np.pi/4)
    ubg = vertcat(ubg,np.pi/4)    

    
    u0 = Uk
    
    xmax = fmax(xmax,Xk_end[1])

J = J+S*0+xmax*20


w = vertcat(w,S)
lbw = vertcat(lbw,0)
ubw = vertcat(ubw,inf)


prob = {'f':J,'x':w,'g':g,'p':p0}

S = nlpsol('S','ipopt',prob)
    
    
    
x0=np.zeros(9)
x0[2]=np.pi/2
x0[4]=4.88
x0[5]=0
x0[3]=0
u0 = 0
    
    
sol = S(x0=0,lbx=lbw,ubx=ubw,lbg=-ubg,ubg=ubg,p=vertcat(x0,u0))
solx = np.ndarray.flatten(sol['x'].full())

opts = {'ipopt.print_level':0, 'print_time':0,'ipopt.max_iter':10,'ipopt.tol':10**(-8)}

S = nlpsol('S','ipopt',prob,opts)
    

uout = np.zeros(N)
xout = np.zeros((9,N+1))
xout[:,0]= x0

solx = np.ndarray.flatten(sol['x'].full())

for i in range(N):
    uout[i] = solx[i*10]
    xout[:,i+1] = solx[i*10+1:i*10+10]

SimT=T
SimN = N
  
plt.figure(1)
plt.plot(xout[0,:],xout[1,:])
plt.title('Trajectory')
plt.axis('equal')

plt.figure(2)
plt.plot(np.linspace(0,SimT,SimN+1),xout[2,:]*180/np.pi)
plt.title('Heading')

plt.figure(3)
plt.plot(np.linspace(0,SimT,SimN+1),xout[3,:]*180/np.pi)
plt.title('Roll angle')

plt.figure(4)
plt.plot(np.linspace(0,SimT,SimN+1),xout[4,:])
plt.title('Surge velocity')

plt.figure(5)
plt.plot(np.linspace(0,SimT,SimN+1),xout[5,:])
plt.title('Sway velocity')

plt.figure(6)
plt.plot(np.linspace(0,SimT,SimN+1),xout[6,:]*180/np.pi)
plt.title('Yaw velocity')

plt.figure(7)
plt.plot(np.linspace(0,SimT,SimN+1),xout[7,:]*180/np.pi)
plt.title('Roll velocity')

plt.figure(8)
plt.plot(np.linspace(0,SimT,SimN+1),xout[8,:]*180/np.pi)
plt.title('Azimuth angle')

plt.figure(9)

plt.step(np.linspace(0,SimT,SimN+1),np.append(uout,uout[-1])*180/np.pi,'-',where='post')
#plt.plot(np.linspace(0,T,N+1),np.append(uopt,0))
plt.title('Control input')

SimT=2*T
SimN = int(SimT*N/T)

uout = np.zeros(SimN)
xout = np.zeros((9,SimN+1))
xout[:,0]= x0

for j in range(1):

    x=x0
    
    xout[:,0] = x0
    
    p = vertcat(x0,0)
    
    tmp = solx
    tmpstore = solx
    w0=tmp
    
    solvetime= np.zeros(SimN)
    
    
    
    tic1 = time.time()
    for i in range(SimN):
        
          tic = time.time()
          
    
          
          sol = S(x0=w0,lbx=lbw,ubx=ubw,lbg=lbg,ubg=ubg,p=p)
          toc = time.time()
          solvetime[i]=toc-tic
          print(np.round(solvetime[i],2))
          print(str(i)+'of'+str(SimN))
          tmp = np.ndarray.flatten(sol['x'].full())
    
          u = tmp[0]
          uout[i]=u
          Fk = F(x0=x,u=u)
          x = np.ndarray.flatten(Fk['xf'].full())
          xout[:,i+1] = x
          p = vertcat(x,u)
          
          w0 = np.append(tmp[10:-1],tmp[-11:])
          
    toc1 = time.time()     
    print('Total' + str(round(toc1-tic1,2)) + 'sec elapsed')
    
    plt.figure(10)
    plt.plot(np.linspace(0,SimT,SimN+1),np.append(solvetime,solvetime[SimN-1]))
          
    
plt.figure(1)
plt.plot(xout[0,:],xout[1,:],'--')
plt.title('Trajectory')
plt.axis('equal')

plt.figure(2)
plt.plot(np.linspace(0,SimT,SimN+1),xout[2,:]*180/np.pi,'--')
plt.title('Heading')

plt.figure(3)
plt.plot(np.linspace(0,SimT,SimN+1),xout[3,:]*180/np.pi,'--')
plt.title('Roll angle')

plt.figure(4)
plt.plot(np.linspace(0,SimT,SimN+1),xout[4,:],'--')
plt.title('Surge velocity')

plt.figure(5)
plt.plot(np.linspace(0,SimT,SimN+1),xout[5,:],'--')
plt.title('Sway velocity')

plt.figure(6)
plt.plot(np.linspace(0,SimT,SimN+1),xout[6,:]*180/np.pi,'--')
plt.title('Yaw velocity')

plt.figure(7)
plt.plot(np.linspace(0,SimT,SimN+1),xout[7,:]*180/np.pi,'--')
plt.title('Roll velocity')

plt.figure(8)
plt.plot(np.linspace(0,SimT,SimN+1),xout[8,:]*180/np.pi,'--')
plt.title('Azimuth angle')

plt.figure(9)

plt.step(np.linspace(0,SimT,SimN+1),np.append(uout,uout[-1])*180/np.pi,'--',where='post')
#plt.plot(np.linspace(0,T,N+1),np.append(uopt,0))
plt.title('Control input')



