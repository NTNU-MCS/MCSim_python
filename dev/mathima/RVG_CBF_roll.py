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
import CBF_roll as CBFr

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

# =============================================================================
# simulation parameters
# =============================================================================
tmax=600
dt=0.01 #time step
tvec = np.linspace(0,tmax,int(tmax/dt)+1)

# =============================================================================
# Control parameters
# =============================================================================
Kp = 1 #heading proportional gain


azi_sat = np.pi/2
#azi_sat = np.pi

# =============================================================================
# Initial conditions and allocate memory
# =============================================================================

revs=200


x = [0,5,0,0,0]
x_out=np.zeros([len(x),len(tvec)])
azi=0
azi_out = np.zeros(len(tvec)) 
B1 = np.zeros(len(tvec)) 
B2= np.zeros(len(tvec)) 
dB1 = np.zeros(len(tvec)) 
dB2= np.zeros(len(tvec)) 

rref = 0
rd=0

parC = {'phi_max':10*np.pi/180,'t1':5,'t2':0.4}


for i2 in [0,1]:
    x = [0,5,0,0,0]
    rref = 0
    rd=0
# =============================================================================
# run simulation
# =============================================================================
    for i1, t in enumerate(tvec):
        
        #store results
        x_out[:,i1] = x
        
        rref=rref-(rref+rd)*dt
        
        azi = -Kp*(x[4]-rref)
        
        if t>100:
           rd=np.pi/6
        
        if t>300:
           rd=-rd
        


        
        if i2==1:
 #           if LfB2+LgB2*azi > -t2*B2:
 #               azi = (-t2*B2-LfB2)/LgB2
            
            azi,B,dB = CBFr.CBFroll_azi(x,azi,revs,parA,parV,parC)
            B1[i1] = B[0]
            B2[i1] = B[1]
            dB1[i1] = dB[0]
            dB2[i1] = dB[1]       
        

        azi = np.max([np.min([azi,azi_sat]),-azi_sat]) 
        
        x = x+CBFr.dx(x,azi,revs,parA,parV)*dt
        azi_out[i1] = azi

    
    # =============================================================================
    # plot
    # =============================================================================
    plt.figure(1)
    plt.plot(tvec,x_out[0,:]*180/np.pi)
    plt.xlabel('Time [s]')
    plt.ylabel('Roll angle [deg]')
    plt.title('Roll response CBF')


    plt.figure(2)
    plt.plot(tvec,x_out[1,:])
    plt.xlabel('Time [s]')  
    plt.ylabel('Surge velocity [m/s]')

    
    plt.figure(3)
    plt.plot(tvec,x_out[2,:])
    plt.xlabel('Time [s]')    
    plt.ylabel('Sway velocity [m/s]')

    
    plt.figure(4)
    plt.plot(tvec,x_out[3,:]*180/np.pi)
    plt.xlabel('Time [s]')   
    plt.ylabel('Roll velocity [deg/s]')


    
    plt.figure(5)
    plt.plot(tvec,x_out[4,:]*180/np.pi)
    plt.xlabel('Time [s]')   
    plt.ylabel('Yaw velocity [deg/s]')

    plt.figure(6)
    plt.plot(tvec,azi_out*180/np.pi)
    plt.xlabel('Time [s]')   
    plt.ylabel('Azimuth angle [deg]')    
    plt.title('Azimuth angle CBF')

plt.figure(7)
plt.plot(tvec,B1)
plt.xlabel('Time [s]')   
plt.ylabel('B1')  

plt.figure(8)
plt.plot(tvec,B2)
plt.xlabel('Time [s]')   
plt.ylabel('B2')      

plt.figure(9)
plt.plot(tvec,dB1)
plt.plot(tvec,np.gradient(B1)/dt,':')
plt.xlabel('Time [s]')   
plt.ylabel('dB1')  

plt.figure(10)
plt.plot(tvec,dB2)
plt.plot(tvec,np.gradient(B2)/dt,':')
plt.xlabel('Time [s]')   
plt.ylabel('dB2')     

fignums = [1,6]

for fignum in fignums:#plt.get_fignums():
    plt.figure(fignum).tight_layout()
    plt.legend(['Without CBF','With CBF'])
    plt.figure(fignum).savefig(plt.figure(fignum).axes[0].get_title()+'.pdf')