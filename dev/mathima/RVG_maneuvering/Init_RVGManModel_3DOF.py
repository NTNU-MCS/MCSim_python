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
Initialize example simulation of 3DOF RVG maneuvering model moving in
uniform and steady currents, using a linear+quadratic damping formulation

Vessel parameters are loaded from pickle files, generated in
GenerateRVGManeuveringModelData.py

Example simulation uses a PD-controller to maintain constant heading 
before executing a hard turn.   

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
parV = pickle.load( open("data\\parV_RVG3DOF.pkl", "rb" ) )
#actuator data
parA = pickle.load( open("data\\parA_RVG.pkl", "rb" ) )

# =============================================================================
# simulation parameters
# =============================================================================
tmax=600
dt=0.1 #time step
tvec = np.linspace(0,tmax,int(tmax/dt)+1)

Uc=np.array(0.5) #current speed

betac = -np.pi/4#current direction
parS = {'dt':dt, 'Uc': Uc, 'betac': betac} #dict containing simulation param

# =============================================================================
# Control parameters
# =============================================================================
Kp = 2 #heading proportional gain
Kd = 20 #heading derivative gain

Uref=parV['reference_velocity']
thrustforce = parV['Dl'][0,0]*Uref+parV['Du'][0,0]*Uref**2
psiref=np.pi/2

# =============================================================================
# Initial conditions and allocate memory
# =============================================================================

revs_d=170

thrust_state = np.array([0,100]) #azimuth angle and revs
nu=np.array([Uref,0,0])
eta=np.array([0,0,0])
x=np.concatenate((eta,nu,thrust_state))
x_out=np.zeros([len(x),len(tvec)])

# =============================================================================
# run simulation
# =============================================================================
for i1, t in enumerate(tvec):
    
    #store results
    x_out[:,i1] = x
    
    #control system (same azimuth angle for both thrusters)
    azi_d = Kp*(km.rad2pipi(x[2]-psiref))+Kd*x[5]
    azi_d = np.max([np.min([azi_d,np.pi/6]),-np.pi/6]) 
    if t>=400:
        azi_d = np.pi/6 #execute evasive manuever
    
    if t>=200:
        revs_d=220 #ramp up propeller revs
        
    #desired thruster state, i.e., control input:
    u = np.array([azi_d,revs_d])
    
    #time integration
    Fw = np.zeros(3) #no disturbance force
    x = model.int_RVGMan3_lq(x, u, Fw, parV, parA, parS)
  
#sort output
eta=x_out[0:3,:]
nu=x_out[3:6,:]
thruststates = x_out[6:8,:]

# =============================================================================
# plot
# =============================================================================
figxy = [10,5] #figure size

plt.figure(1,figsize=figxy, clear='True')
plt.plot(eta[0,:],eta[1,:])
plt.title('Trajectory')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')

titles = ['x position','y position','heading','surge velocity','sway velocity',
           'yaw velocity','azimuth angle','thruster revs']
units = ['m','m','deg','m/s','m/s','deg/s','deg','RPM']
scale = [1,1,180/np.pi,1,1,180/np.pi,180/np.pi,1]


parP={'figsize':figxy,'titles':titles,'units':units,'scale':scale,'fig0':2}

viz.plot_timeseries(tvec,x_out,parP)


