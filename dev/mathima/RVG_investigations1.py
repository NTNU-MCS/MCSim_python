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
tmax=1800
dt=1 #time step
tvec = np.linspace(0,tmax,int(tmax/dt)+1)

Uc=np.array(0.5) #current speed
Uc=0
betac = -np.pi/4#current direction
parS = {'dt':dt, 'Uc': Uc, 'betac': betac} #dict containing simulation param

# =============================================================================
# Control parameters
# =============================================================================
Kp = 2 #heading proportional gain
Kd = 20 #heading derivative gain

azi_sat = np.pi/6

Uref=parV['reference_velocity']
thrustforce = parV['Dl'][0,0]*Uref+parV['Du'][0,0]*Uref**2


# =============================================================================
# Initial conditions and allocate memory
# =============================================================================

revs_d=170

thrust_state = np.array([0,100]) #azimuth angle and revs
nu=np.array([Uref,0,0,0,0,0])
eta=np.array([0,0,0,0,0,0])
x=np.concatenate((eta,nu,thrust_state))
x_out=np.zeros([len(x),len(tvec)])
ForceAct = np.zeros([6,len(tvec)])

u0 = [3.8,5.1,6.4]

for i2 in [0,1,2]:

    revs_d=150+i2*50
    
    
    thrust_state = np.array([0,revs_d]) #azimuth angle and revs

    nu=np.array([u0[i2],0,0,0,0,0])
    eta=np.array([0,0,0,0,0,0])
    x=np.concatenate((eta,nu,thrust_state))
    x_out=np.zeros([len(x),len(tvec)])
    ForceAct = np.zeros([6,len(tvec)])    
    psiref=0
# =============================================================================
# run simulation
# =============================================================================
    for i1, t in enumerate(tvec):
        
        #store results
        x_out[:,i1] = x
        
        #control system (same azimuth angle for both thrusters)
        azi_d = Kp*(km.rad2pipi(x[5]-psiref))+Kd*x[11]
        azi_d = np.max([np.min([azi_d,azi_sat]),-azi_sat]) 
        
    #    if t>=100:
    #        psiref = np.pi/2
    #    
    #    if t>=300:
    #        azi_d = azi_sat #execute evasive manuever
    #    
        if x[0] >200:
            psiref = np.pi
            
        azi_d = (t-300)/(tmax-300)*np.pi/6
        
        if t<=300:
            azi_d=0
        
        u = np.array([azi_d,revs_d])
        
        #time integration
        Fw = np.zeros(6) #no disturbance force

        x, ForceAct[:,i1] = model.int_RVGMan6_lq(x, u, Fw, parV, parA, parS)
    
      
    #sort output
    eta=x_out[0:6,:]
    nu=x_out[6:12,:]
    thruststates = x_out[13:14,:]
    
    # =============================================================================
    # plot
    # =============================================================================
    plt.figure(1)
    plt.plot(x_out[12,:]*180/np.pi,x_out[6,:])
    plt.xlabel('Azimuth angle [deg]')
    plt.ylabel('Surge velocity [m/s]')
    plt.title('Surge velocity versus azimuth angle')
    plt.legend(('Revs=150RPM','Revs=200RPM','Revs=250RPM'))
    
    plt.figure(2)
    plt.plot(x_out[12,:]*180/np.pi,x_out[7,:])
    plt.xlabel('Azimuth angle [deg]')    
    plt.ylabel('Sway velocity [m/s]')
    plt.title('Sway velocity versus azimuth angle')
    plt.legend(('Revs=150RPM','Revs=200RPM','Revs=250RPM'))

    
    plt.figure(3)
    plt.plot(x_out[12,:]*180/np.pi,x_out[11,:]*180/np.pi)
    plt.xlabel('Azimuth angle [deg]')
    plt.ylabel('Yaw velocity [deg/s]')
    plt.title('Yaw velocity versus azimuth angle')
    plt.legend(('Revs=150RPM','Revs=200RPM','Revs=250RPM'))
    
    plt.figure(4)
    plt.plot(x_out[12,:]*180/np.pi,x_out[3,:]*180/np.pi)
    plt.xlabel('Azimuth angle [deg]')
    plt.ylabel('Roll angle [deg/s]')
    plt.title('Roll angle versus azimuth angle')
    plt.legend(('Revs=150RPM','Revs=200RPM','Revs=250RPM'))
    
#    plt.title('velocity versus azimuth angle')  
#    plt.legend(('surge','sway','yaw'))

    # figxy = [5,2] #figure size
    
    # if i2==0:
    #    plt.figure(1,figsize=figxy, clear='True')
    # else:
    #    plt.figure(1)
    # plt.plot(eta[0,:],eta[1,:])
    # plt.title('Trajectory')
    # plt.xlabel('x [m]')
    # plt.ylabel('y [m]')
    # plt.axis('equal')
    # plt.tight_layout()
    
    # titles = ['x position','y position','z position','roll','pitch','yaw',
    #           'surge velocity','sway velocity','heave velocity','roll velocity',
    #           'pitch velocity','yaw velocity','azimuth angle','thruster revs']
    # units = ['m','m','m','deg','deg','deg','m/s','m/s','m/s','deg/s','deg/s','deg/s','deg','RPM']
    # scale = [1,1,1,180/np.pi,180/np.pi,180/np.pi,1,1,1,180/np.pi,180/np.pi,180/np.pi,180/np.pi,1]
    
    # if i2==0:
    #     parP={'figsize':figxy,'titles':titles,'units':units}
    # else:   
    #     parP={'titles':titles,'units':units}    
    # parP['fig0'] = 2
    # parP['scale'] = scale
    # viz.plot_timeseries(tvec,x_out,parP)


for fignum in plt.get_fignums():
    plt.figure(fignum).tight_layout()
    plt.figure(fignum).savefig(plt.figure(fignum).axes[0].get_title()+'.pdf')
    
