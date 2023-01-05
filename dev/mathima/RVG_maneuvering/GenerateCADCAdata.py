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
import Module_RVGManModel4DOF as model


def run_sim(parV,parS,parA):
    
    revs_d = parS['revs']
    thrust_state = np.array([0,revs_d]) #azimuth angle and revs
    nu=np.array([parS['v'],0,0,0]) #u v p r
    eta=np.array([0,0,0,0]) #x y phi psi
    x=np.concatenate((eta,nu,thrust_state))

    tvec = parS['tvec']
    x_out=np.zeros([len(x),len(tvec)])
    

    
    # =============================================================================
    # Control parameters
    # =============================================================================
    Kp = 1 #heading proportional gain
    Kd = 20 #heading derivative gain
    
    azi_sat = np.pi/8   
    
    azi_exec = parS['azi']
    Fx = tvec*0
# =============================================================================
# run simulation
# =============================================================================

    for i1, t in enumerate(tvec):
    
        
    
         #store results
         x_out[:,i1] = x
     
         #control system (same azimuth angle for both thrusters)
         azi_d = Kp*(km.rad2pipi(x[3]))+Kd*x[7]
         azi_d = np.max([np.min([azi_d,azi_sat]),-azi_sat]) 
         if t>=200:
             azi_d = azi_exec #execute evasive manuever
        

            
         u = np.array([azi_d,revs_d])
        
         #time integration
         Fw = np.zeros(4) #no disturbance force
         
         
         x, Fx[i1] = model.int_RVGMan4(x, u, Fw, parV, parA, parS)
         if np.abs(x[3])>2*np.pi:
             azi_exec=0
             
  
    #sort output
    eta=x_out[0:4,:]
#    nu=x_out[4:8,:]
#    thruststates = x_out[8:10,:]
    
    # =============================================================================
    # plot
    # =============================================================================
    figxy = [10,5] #figure size
    
    plt.figure(1)#,figsize=figxy, clear='True')
    plt.plot(eta[0,:],eta[1,:])
    plt.title('Trajectory')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axis('equal')
    
    titles = ['x position','y position','roll','yaw',
              'surge velocity','sway velocity','roll velocity',
                'yaw velocity','azimuth angle','thruster revs']
    units = ['m','m','deg','deg','m/s','m/s','deg/s','deg/s','deg','RPM']
    scale = [1,1,180/np.pi,180/np.pi,1,1,180/np.pi,180/np.pi,180/np.pi,1]
    parP={'titles':titles,'units':units}
    parP['fig0'] = 2
    parP['scale'] = scale
    #parP['figsize']=figxy
    
    viz.plot_timeseries(tvec,x_out,parP)

    return x_out, Fx

plt.close('all')


# =============================================================================
# load model data
# =============================================================================
#vessel data
parV,parA = model.GenModelData2(0)


# =============================================================================
# Cases
# =============================================================================


columns = ['code','azi','revs','v','kg']

symlist = pd.DataFrame(columns)


rev = np.array([140,170,200]) #np.array([140,170,200])

azi = np.array([5, 10, 15, 20, 25, 30])


revs = []
kgs = []
azis = []
codes = []
vs = []
Fn= []

for r in rev:
    for a in azi:
        
        kgs.append(3.7)
        codes.append('Azi'+str(a)+'Revs'+str(r))
        print(r)
        revs.append(r)
        azis.append(a)
        
        if r == 140: v=4.36
        if r == 170: v=5.29
        if r == 200: v=6.22
                
        
        vs.append(v)
        Fn.append(v/np.sqrt(9.81*parV['Lpp']))
        
            
symlist = pd.DataFrame.from_dict({'code': codes,'v':vs,'Fn':Fn,'azi':azis,
                        'revs':revs,'kg':kgs})
symlist.to_csv('symlist'+'.csv',index=False,float_format='%.2f')

# =============================================================================
# simulation parameters 
# =============================================================================
tmax=800
dt=0.2 #time step
tvec = np.linspace(0,tmax,int(tmax/dt)+1)

Uc=np.array(0) #current speed
betac = 0#current direction
parS = {'dt':dt, 'Uc': Uc, 'betac': betac} #dict containing simulation param
parS['tvec'] = tvec

# =============================================================================
# Run simulations
# =============================================================================

columns = ['Time[s]', 'Wave[m]', 'Vs[m/s]', 'Xg[m]', 'Yg[m]', 'Zg[m]', 
           'Fi[deg]','Theta[deg]','Psi[deg]','Del[deg]','Thrust[kN]']


for index, rows in symlist.iterrows():
    parS['azi'] = rows['azi']*np.pi/180
    parS['v'] = rows['v']
    parS['revs'] = rows['revs']
    X,Fx = run_sim(parV,parS,parA)
    

#    titles = ['x position','y position','roll','yaw',
#              'surge velocity','sway velocity','roll velocity',
#                'yaw velocity','azimuth angle','thruster revs']
#    units = ['m','m','deg','deg','m/s','m/s','deg/s','deg/s','deg','RPM']
#    scale = [1,1,180/np.pi,180/np.pi,1,1,180/np.pi,180/np.pi,180/np.pi,1]
    
    wave = tvec*0
    
    x = X[0,:]
    y = X[1,:]
    z = wave
    Vs = X[4,:]
    Fi = X[2,:]*180/np.pi
    Theta = wave
    Psi = X[3,:]*180/np.pi
    Del = X[8,:]*180/np.pi
    Thrust = Fx/1000
    
    res = pd.DataFrame.from_dict({'Time[s]': tvec, 'Wave[m]': wave, 
                                  'Vs[m/s]':Vs, 'Xg[m]':x, 'Yg[m]':y, 'Zg[m]':z, 
               'Fi[deg]': Fi,'Theta[deg]': Theta,'Psi[deg]': Psi,'Del[deg]': Del,'Thrust[kN]': Thrust})
    res.to_csv(rows.code+'.csv',index=False,float_format='%.2f')
    
    
