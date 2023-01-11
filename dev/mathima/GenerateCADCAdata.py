# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-11-01
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley Checked that results make sense
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
# """
# Generates data for CADCA as part of Endure project
# """



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
import pandas as pd

# =============================================================================
# Load model module
# =============================================================================
import Module_RVGManModel as model

# =============================================================================
# Simulation cases
# =============================================================================


cases = pd.read_csv('symlist_Endure.csv')

colname = {'Time[s]', 'Wave[m]', 'Vs[m/s]', 'Xg[m]', 'Yg[m]',   'Zg[m]',  
            'Fi[deg]', 'Theta[deg]', 'Psi[deg]',  'Del[deg]',  'R[deg/s]',  
            'Drift[deg]', 'Thrust[kN]', 'Yrud[kN]', 'ACTHEA[deg]'}

data = np.zeros((3,15))

df = pd.DataFrame(data)
df.columns = colname

df.to_csv('test')

#plt.close('all')

# =============================================================================
# load model data
# =============================================================================
#vessel data
parV = pickle.load( open(model_path+"\\data\\parV_RVG6DOF.pkl", "rb" ) )

#actuator data
parA = pickle.load( open(model_path+"\\data\\parA_RVG.pkl", "rb" ) )


raise SystemExit


# =============================================================================
# simulation parameters
# =============================================================================
tmax=800
dt=0.2 #time step
tvec = np.linspace(0,tmax,int(tmax/dt)+1)

Uc=np.array(0) #current speed
betac = -np.pi/4#current direction
parS = {'dt':dt, 'Uc': Uc, 'betac': betac} #dict containing simulation param




# =============================================================================
# Control parameters
# =============================================================================
Kp = 2 #heading proportional gain
Kd = 20 #heading derivative gain

azi_sat = np.pi/8

Uref=parV['reference_velocity']
thrustforce = parV['Dl'][0,0]*Uref+parV['Du'][0,0]*Uref**2
psiref=np.pi/2

# =============================================================================
# Initial conditions and allocate memory
# =============================================================================

revs_d=170

thrust_state = np.array([0,170]) #azimuth angle and revs
nu=np.array([Uref,0,0,0,0,0])
eta=np.array([0,0,0,0,0,0])
x=np.concatenate((eta,nu,thrust_state))
x_out=np.zeros([len(x),len(tvec)])
ForceAct = np.zeros([6,len(tvec)])


# =============================================================================
# run simulation
# =============================================================================
for i1, t in enumerate(tvec):
    
    #store results
    x_out[:,i1] = x
    
    #control system (same azimuth angle for both thrusters)
    azi_d = Kp*(km.rad2pipi(x[5]-psiref))+Kd*x[11]
    azi_d = np.max([np.min([azi_d,azi_sat]),-azi_sat]) 
    if t>=400:
        azi_d = azi_sat #execute evasive manuever
    
    if t>=200:
        revs_d=170 #ramp up propeller revs
    if t>=600:
        revs_d = 170
        
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
figxy = [10,5] #figure size

plt.figure(1,figsize=figxy, clear='True')
plt.plot(eta[0,:],eta[1,:])
plt.title('Trajectory')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')

titles = ['x position','y position','z position','roll','pitch','yaw',
          'surge velocity','sway velocity','heave velocity','roll velocity',
          'pitch velocity','yaw velocity','azimuth angle','thruster revs']
units = ['m','m','m','deg','deg','deg','m/s','m/s','m/s','deg/s','deg/s','deg/s','deg','RPM']
scale = [1,1,1,180/np.pi,180/np.pi,180/np.pi,1,1,1,180/np.pi,180/np.pi,180/np.pi,180/np.pi,1]
parP={'figsize':figxy,'titles':titles,'units':units}
parP['fig0'] = 2
parP['scale'] = scale
viz.plot_timeseries(tvec,x_out,parP)




