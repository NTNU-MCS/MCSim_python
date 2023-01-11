# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-12-12
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-12-12 M.Marley Checked that results make sense
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Numerical simulation of a 4DOF maneuvering model of RVG, with commanded
thruster states taken from sea trials performed Fall 2022 in Trondheim fjord. 

Default model parameters are calibrated towards sea trial results. 

The sea trials were performed for operating conditions slightly outside 
the validity range of the thruster model. This gives a warning. 
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: used together with Module_RVGManModel4DOF
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

cur_path = str(Path(os.path.dirname(__file__)).parents[0]) 


# =============================================================================
# Load general modules
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import kinematics as km

# =============================================================================
# Load model module
# =============================================================================
import Module_RVGManModel4DOF as model
import pandas as pd

# =============================================================================
# Functions
# =============================================================================

def Set_SimData(t,data):
    """Resets vessel state to sea trial recorded data. 
    """

    
    index = data.tvec.values==t
    
    u0 = data.u.values[index][0]
    v0 = data.v.values[index][0]
    psi0 = data.psi.values[index][0]*deg2rad
    r0 = data.r.values[index][0]*deg2rad

    E = data.E.values[index][0]
    N = data.N.values[index][0]

    azi = data.Azi.values*deg2rad
    revs = data.RPM.values

    thrust_state = np.array([azi[index][0],revs[index][0]]) #azimuth angle and revs
    nu=np.array([u0,v0,0,r0]) #u v p r
    eta=np.array([N,E,0,psi0]) #x y phi psi
    x=np.concatenate((eta,nu,thrust_state))
    
    return x

def Plot_ExpRes(data,channels,fignum):
    """Plots experimental results
    """
    for i,channel in enumerate(channels):
        plt.figure(fignum[i])
        plt.plot(data.tvec.values,data[channel].values,color='black')
        plt.title(channel)
        
def Plot_SimRes(data,channels,fignum,t_reset,xref,plotcolor):
    """Plots simulation results
    """
    data['v'] = data['v']+xref*data['r']*np.pi/180
    for i,channel in enumerate(channels):
        plt.figure(fignum[i])
        plt.plot(data.tvec.values,data[channel].values,color=plotcolor)  
        
    N = data['N']
    E = data['E']    

    tvec = data.tvec.values    
    
    for it in range(len(t_reset)-1):
        ind1 =  np.where(t_reset[it]==tvec)[0][0]+1
        ind2 =  np.where(t_reset[it+1]==tvec)[0][0]
        plt.figure(1)#,figsize=figxy, clear='True')
        plt.plot(E[ind1:ind2],N[ind1:ind2],color=plotcolor)
        plt.scatter(E[ind1],N[ind1],color=plotcolor)

    plt.title('Trajectory')
    plt.xlabel('East [m]')
    plt.ylabel('North [m]')
    plt.axis('equal')
    plt.legend(['Experiment','Simulation'])
    plt.tight_layout()
    plt.savefig('Trajectory'+'.pdf')
    
        
 
def Run_Sim(data,parA,parV,parS,t_reset):
    """Runs numerical simulation 
    """  
    tvec = data.tvec.values
    
    dt=tvec[1]-tvec[0] #time step 
    
    parS['dt'] = dt
    
    azi = data.Azi.values*deg2rad
    revs = data.RPM.values
    
    x = Set_SimData(0,data)
    
    x_out=np.zeros([len(x),len(tvec)])
    

    for i1, t in enumerate(tvec):
    
        
        #store results
        x_out[:,i1] = x
        
        if t in t_reset:
            x = Set_SimData(t,data)
        
        #control system (same azimuth angle for both thrusters)
            
        u = np.array([azi[i1],revs[i1]])
        
        #time integration
        Fw =  parS['Fw']
        x = model.int_RVGMan4(x, u, Fw, parV, parA, parS)
        x[3] = km.rad2pipi(x[3])
      
    
    N = x_out[0]
    E = x_out[1]
    phi = -x_out[2]/deg2rad
    psi = x_out[3]/deg2rad
    u = x_out[4]

    p = x_out[6]/deg2rad
    v = x_out[5]
    r = x_out[7]/deg2rad
    Azi = x_out[8]/deg2rad
    rpm = x_out[9]

    
    res_dict = {'tvec':tvec,'N':N,'E':E,'phi':phi,'psi':psi,'u':u,
                'v':v,'p':p,'r':r,'Azi':Azi,'RPM':rpm}
    
    res = pd.DataFrame.from_dict(res_dict)    
    return res
 
   
# =============================================================================
# Extract and plot experimental results
# =============================================================================

plt.close('all')

deg2rad = np.pi/180

#load experimental data
data = pd.read_csv('data\\RVG_ManeuveringTests.csv')
data['psi'] = km.rad2pipi(data['psi']*deg2rad)/deg2rad

#plot trajectory of experiments 
plt.figure(1)
plt.plot(data.E.values,data.N.values,color='black')
plt.axis('equal')

#plot experiment results
channels = ['phi','psi','u','v','r','Azi','RPM']
fignum = [2,3,4,5,6,7,8]
Plot_ExpRes(data,channels,fignum)

# =============================================================================
# Run numerical simulation
# =============================================================================

#timestamp for resetting vessel state. This is done to obtain correct heading
# relative to current direction.  
t_reset = [0, 450,750, 1100, 1450, 1600, 1950, 2250, 2600,2950,3250,3700,4100,
           4375,4650,5025,5300,5950,6250,6900,7100,7550,7800,8200,8500,
           data.tvec.values[-1]]

Uc=np.array(0.15) #estimated current speed during experiments
betac = 70*np.pi/180#estimated current direction

parS = {'Uc': Uc, 'betac': betac,'Fw':np.array([0,0,0,0])} #dict containing simulation param

parV,parA = model.DefaultModelData() #model parameters

xref=2 #model reference point is in COG. Experimental data reference point assumed
# to be located 2m in front of COG. 

res = Run_Sim(data,parA,parV,parS,t_reset)
Plot_SimRes(res,channels,fignum,t_reset,xref,'orange')











