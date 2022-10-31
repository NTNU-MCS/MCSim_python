# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley Checked that results make sense
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Initialize example simulation of a pseudo-6DOF RVG maneuvering model moving in
uniform and steady currents, using a linear+quadratic damping formulation. The
model captures the 3DOF horizontal plane motion together with roll, while
heave and pitch motion is effectively suppressed by large damping and
excluding certain load effects.

Vessel parameters are loaded from pickle files, generated in
GenerateRVGManeuveringModelData.py

Example simulation uses a PD-controller to maintain constant heading 
before executing a hard turn. Propeller revs are ramped up and down.  

Control inputs are the commanded thruster states: azimuth angle and propeller
revs of an equivalent thruster located at centreline. 
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: used together with Module_RVG_3DOF
# ---------------------------------------------------------------------------

# =============================================================================
# Set path
# =============================================================================

import os
from pathlib import Path
import sys

par_path = str(Path(os.path.dirname(__file__)).parents[1])  
library_path = par_path + '\\lib'
sys.path.append(library_path)
model_path = par_path+'\\models\\RVG_maneuvering'
sys.path.append(model_path)

# =============================================================================
# Load general modules
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import pickle
import kinematics as km
import visualization as viz

# =============================================================================
# Load model module
# =============================================================================
import Module_RVGManModel as model

plt.close('all')

# =============================================================================
# load model data
# =============================================================================
#vessel data
parV = pickle.load( open(model_path+"\\data\\parV_RVG6DOF.pkl", "rb" ) )

#actuator data
parA = pickle.load( open(model_path+"\\data\\parA_RVG.pkl", "rb" ) )


#parV['Ma'] = parV['Ma']
#parV['Ma'][1,5]=0
#parV['Ma'][5,1]=0
#parV['dvv']=10*parV['dvv']
#parV['dvv']=parV['Du'][0,0]
#parV['Dl'][1,1] = parV['Dl'][0,0]
#parV['Ma']=parV['Ma']*0
#parV['Ma'][1,1]
#parV['Ma'] = 0.5*(parV['Ma']+parV['Ma'].T)


# =============================================================================
# simulation parameters
# =============================================================================
tmax=600
dt=0.25 #time step
tvec = np.linspace(0,tmax,int(tmax/dt)+1)

Uc=np.array(0.5) #current speed
Uc=0.5
betac = -np.pi/4#current direction
parS = {'dt':dt, 'Uc': Uc, 'betac': betac} #dict containing simulation param

# =============================================================================
# Control parameters
# =============================================================================
Kp = 1 #heading proportional gain
Kd = 5 #heading derivative gain

azi_sat = np.pi/6

Uref=parV['reference_velocity']
thrustforce = parV['Dl'][0,0]*Uref+parV['Du'][0,0]*Uref**2

parA['Tazi']=1

# =============================================================================
# Initial conditions and allocate memory
# =============================================================================
legstr = ['Without roll damping','With roll damping']

revs_d=170

thrust_state = np.array([0,100]) #azimuth angle and revs
nu=np.array([Uref,0,0,0,0,0])
eta=np.array([0,0,0,0,0,0])
x=np.concatenate((eta,nu,thrust_state))
x_out=np.zeros([len(x),len(tvec)])
ForceAct = np.zeros([6,len(tvec)])

u0=5.1
revs_d = 200
u0 = [3.8,5.1,6.4]
u0 = [5.1,5.1,5.1]
w_azi_vec = 2*np.pi*np.array([1/1,1/10,1/10])
w_azi_vec = 2*np.pi*np.array([1/1,1/10,1/10])
dx2_max_vec = np.array([1,1,1/20])
dx2_max_vec = np.array([1,1/10,1/10])
x_r_vec=np.pi*np.array([25,25,30])/180

w0=2*np.pi/10 #peak frequency
damp=0.2 #damping (controls narrow-bandedness)
Kw=2*damp*w0*10**6*0.05
Aw = np.array([[0, 1],[ -w0**2, -2*damp*w0**2]])
Bw = np.array([Kw, 0])
zw = np.array([0,0])





for i2 in [0,1]:#[0,1,2]:

    parA['Tazi']=1
    w_azi = w_azi_vec[i2]
    dx2max = dx2_max_vec[i2]
    
    thrust_state = np.array([0,revs_d]) #azimuth angle and revs

    nu=np.array([u0[i2],0,0,0,0,0])
    eta=np.array([0,0,0,0,0,0])
    x=np.concatenate((eta,nu,thrust_state))
    x_out=np.zeros([len(x),len(tvec)])
    ForceAct = np.zeros([6,len(tvec)])    
    psiref=0
    azi_d=0

    x_r=0
    x2 =0
    azi_d=0
    
    w=[0,0]
    np.random.seed(1)
# =============================================================================
# run simulation
# =============================================================================
    for i1, t in enumerate(tvec):
        
        #store results
        x_out[:,i1] = x
        
        #control system (same azimuth angle for both thrusters)
        azi_d = Kp*(km.rad2pipi(x[5]-psiref))+Kd*x[11]
        
        if i2==1:
           azi_d = Kp*(km.rad2pipi(x[5]-psiref))+Kd*x[11]-Kd*x[9]*0.5         
        
        azi_d = np.max([np.min([azi_d,azi_sat]),-azi_sat]) 
         
        if t>200:
            azi_d = np.pi/8
            if i2==1:
                azi_d = np.pi/8-Kd*x[9]*0.5

        
        u = np.array([azi_d,revs_d])
        
        #time integration
        dw = Aw@w+Bw*(np.random.random(1)-0.5)
        w = w+dw*dt
        Fw = np.zeros(6) #no disturbance force
        Fw[3] = w[0]*0.5
        Fw[1] = w[0]*0.1
        Fw[5] = w[0]*10
        
        x, ForceAct[:,i1] = model.int_RVGMan6_lq(x, u, Fw, parV, parA, parS)
        
          
    #sort output
    eta=x_out[0:6,:]
    nu=x_out[6:12,:]
    thruststates = x_out[13:14,:]
    
    # =============================================================================
    # plot
    # =============================================================================
    plt.figure(1)
    plt.plot(tvec,x_out[6,:])
    plt.xlabel('Time [s]')
    plt.ylabel('Surge velocity [m/s]')
    plt.title('Surge')
    plt.legend(legstr)
    
    plt.figure(2)
    plt.plot(tvec,x_out[7,:])
    plt.xlabel('Time [s]') 
    plt.ylabel('Sway velocity [m/s]')
    plt.title('Sway')
    plt.legend(legstr)

    
    plt.figure(3)
    plt.plot(tvec,x_out[11,:]*180/np.pi)
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw velocity [deg/s]')
    plt.title('Yaw')
    plt.legend(legstr)
    
    plt.figure(4)
    plt.plot(tvec,x_out[3,:]*180/np.pi)
    plt.xlabel('Time [s]')
    plt.ylabel('Roll angle [deg]')
    plt.title('Roll')
    plt.legend(legstr)

    plt.figure(5)
    plt.plot(tvec,x_out[12,:]*180/np.pi)
    plt.xlabel('Time [s]')
    plt.ylabel('Azimuth angle [deg]')
    plt.title('Azimuth angle')
    plt.legend(legstr)

    plt.figure(6)
    plt.plot(x_out[0,:],x_out[1,:])
    plt.axis('equal')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Trajectory')
    plt.legend(legstr)
    
    plt.figure(7)
    plt.plot(tvec,x_out[5,:]*180/np.pi)
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw angle [deg]')
    plt.title('Yaw')
    plt.legend(legstr)


# fignum in plt.get_fignums():
#    plt.figure(fignum).tight_layout()
#    plt.figure(fignum).savefig(plt.figure(fignum).axes[0].get_title()+'rolldampingturn.pdf')
    



