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

par_path = str(Path(os.path.dirname(__file__)).parents[1])  
library_path = par_path + '\\lib'
sys.path.append(library_path)
model_path = par_path+'\\models\\RVG_maneuvering'
sys.path.append(model_path)

#vessel data
parV = pickle.load( open(model_path+"\\data\\parV_RVG6DOF.pkl", "rb" ) )

#actuator data
parA = pickle.load( open(model_path+"\\data\\parA_RVG.pkl", "rb" ) )

#plt.close('all')

# casadi
p = SX.sym('p',2)

psi = SX.sym('psi',1)
phi = SX.sym('phi',1)

nu = SX.sym('nu',4) #u,v,r,p
alpha = SX.sym('alpha',1)
#Ft = SX.sym('Ft',1)
#Ft=1.8*160**2
Ft = SX.sym('Ft',1)

u1 = SX.sym('u1',1)
u2 = SX.sym('u2',1)

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

fx = vertcat(pdot,psidot,phidot,nudot,0,0)

gx1 = vertcat((0,0,0,0,0,0,0,0,1,0))

gx2 = vertcat((0,0,0,0,0,0,0,0,0,1))

x=vertcat(p,psi,phi,nu,alpha,Ft)


T=20
N=T#10#2*T

M = 1
DT = T/N/M

L=x[2]**2+x[3]**2+u**2+(x[4]-5)**2

f = Function('f', [x, u1, u2], [fx+gx1*u1+gx2*u2,L])
 
X0 = MX.sym('X0', 9)
U1 = MX.sym('U1',1)
U2 = MX.sym('U2',1)
X = X0
Q = 0


for j in range(0,M):
    [k1, k1_q] = f(X, U1,U2);
    [k2, k2_q] = f(X + DT/2 * k1, U1,U2)
    [k3, k3_q] = f(X + DT/2 * k2, U1,U2)
    [k4, k4_q] = f(X + DT * k3, U1,U2)
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)

F = Function('F', [X0, U1,U2], [X, Q], ['x0','u1','u2'], ['xf', 'qf'])
 

w1 = []
w2 = []
w0 = np.zeros(2*N+1)

lbw = np.zeros(2*N+1)
lbw[0:N] = -np.pi/10
ubw[0:N] = np.pi/10
lbw[N:2*N] = -1000
ubw[N:2*N] = 1000


lbw[2*N]=0
ubw[2*N]=inf

J=0
g=[]
lbg1 = np.zeros(N)-inf
ubg1 = np.zeros(3*N)
S = MX.sym('S',1)



x0 = MX.sym('x0',9)



Xk = x0
xmax=0

for k in range(0,N):
#    print('U'+str(k))
    U1k = MX.sym('U1' +str(k))
    U2k = MX.sym('U2' +str(k))
    w1 = vertcat(w1,U1k)
    w2 = vertcat(w2,U2k)
        
    Fk = F(x0=Xk,u1=U1k,u2=U2k)
    Xk = Fk['xf']
    
    xmax = fmax(xmax,Xk[0])
    
    J = J + Fk['qf']
    g = vertcat(g,Xk[3]-S,-Xk[3]-S,Xk[8],Xk[9])
    ubg[3*k]=14*np.pi/180
    ubg[3*k+1]=ubg[3*k]
    ubg[3*k+2] = np.pi/4
    lbg[3*k+2] = -np.pi/4

J = J+S*10000#xmax*0

w = vertcat(w,S)


#g = vertcat(g,Xk[2],Xk[6])


prob = {'f':J,'x':w,'g':g,'p':x0}
opts = {'ipopt.print_level':0, 'print_time':0,'ipopt.tol':10**(-9)} #,'ipopt.max_iter':100,'ipopt.tol':10**(-9)}


for i in [1]:

    S = nlpsol('S','ipopt',prob,opts)
    
    
    
    x0=np.zeros(9)
    x0[2]=np.pi/2
    x0[4]=4.88
    x0[5]=0
    x0[3]=0
    
    
    
    sol = S(x0=w0,lbx=lbw,ubx=ubw,lbg=-ubg,ubg=ubg,p=x0)
    
    uopt = np.ndarray.flatten(sol['x'].full())[0:N]
    
#    plt.figure(1)
#    plt.plot(range(N),uopt)




uopt = np.ndarray.flatten(sol['x'].full())[0:N]

S = nlpsol('S','ipopt',prob,opts)


SimT = 70
SimN = int(SimT*N/T)

uout=np.zeros(SimN)

xout = np.zeros((9,SimN+1))
x=x0

xout[:,0] = x0

tmp = np.ndarray.flatten(sol['x'].full())
tmpstore = np.ndarray.flatten(sol['x'].full())
Svec = np.zeros(SimN)

solvetime= np.zeros(SimN)

cvec = ['r','b','g']

tol=[10**(-8),10**(-4),10**(-8)]
iterat=[40,40,40]

for iCase in [0]:

    for j in range(1):    
        print('Sim='+str(j+10*iCase))
        

        x=x0

        tic1 = time.time()
        tmp = tmpstore
        opts = {'ipopt.print_level':0, 'print_time':0,'ipopt.tol':tol[iCase]} #,'ipopt.max_iter':100,'ipopt.tol':10**(-9)}
        opts = {'ipopt.tol':tol[iCase]}
        opts = {'ipopt.print_level':0, 'print_time':0,'ipopt.max_iter':iterat[iCase]}
        S = nlpsol('S','ipopt',prob,opts)
            
        
        for i in range(SimN):
            
              tic = time.time()
              
              sol = S(x0=np.append(tmp[1:N],tmp[N-1:]),lbx=lbw,ubx=ubw,lbg=lbg,ubg=ubg,p=x)
              toc = time.time()
              solvetime[i]=toc-tic
         #     print(str(round(toc-tic,3)) + 'sec elapsed. S='+str(tmp[N]))
              print('Step ' + str(i) + ' of ' + str(SimN))
        
              tmp = np.ndarray.flatten(sol['x'].full())
              Svec[i] = tmp[N]
              u = tmp[0]
              uout[i]=u
              Fk = F(x0=x,u=u+0*np.random.rand(1)/100)
              x = np.ndarray.flatten(Fk['xf'].full())
              xout[:,i+1] = x
        toc1 = time.time()     
        print('Total' + str(round(toc1-tic1,2)) + 'sec elapsed')

        plt.figure(10)
        plt.plot(np.linspace(0,SimT,SimN+1),np.append(solvetime,solvetime[SimN-1]))#,cvec[iCase])
         
    
    plt.figure(1)
    plt.plot(xout[1,:],xout[0,:])
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
    
    plt.step(np.linspace(0,SimT,SimN+1),np.append(uout,uout[-1])*180/np.pi,'-o',where='post')
    #plt.plot(np.linspace(0,T,N+1),np.append(uopt,0))
    plt.title('Control input')


