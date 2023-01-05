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
Initialize example simulation of a 4DOF RVG maneuvering model moving in
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

cur_path = str(Path(os.path.dirname(__file__)).parents[0]) 

sys.path.append(cur_path+'\\RVG_maneuvering\\data')

# =============================================================================
# Load general modules
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import pickle
import kinematics as km
import visualization as viz
from scipy import interpolate

# =============================================================================
# Load model module
# =============================================================================
import Module_RVGManModel4DOF as model
import ManTestData as MTD
import pandas as pd
import GenerateRVGManeuveringModelData4DOF_rev2 as modpar

# =============================================================================
# Functions
# =============================================================================

def Set_SimData(t,data):
    
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
    for i,channel in enumerate(channels):
        plt.figure(fignum[i])
        plt.plot(data.tvec.values,data[channel].values,color='black')
        plt.title(channel)
        
def Plot_SimRes(data,channels,fignum,t_reset,xref,plotcolor):

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
        x, Fx = model.int_RVGMan4(x, u, Fw, parV, parA, parS)
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
# compare with test data
# =============================================================================

deg2rad = np.pi/180

plt.close('all')

data = pd.read_csv('RVG_ManeuveringTests.csv')

data['psi'] = km.rad2pipi(data['psi']*deg2rad)/deg2rad
 
plt.figure(1)
plt.plot(data.E.values,data.N.values,color='black')
plt.axis('equal')

channels = ['phi','psi','u','v','r','Azi','RPM']
fignum = [2,3,4,5,6,7,8]

Plot_ExpRes(data,channels,fignum)

t_reset = [0, 450,750, 1100, 1450, 1600, 1950, 2250, 2600,2950,3250,3700,4100,
           4375,4650,5025,5300,5950,6250,6900,7100,7550,7800,8200,8500,
           data.tvec.values[-1]]
t_reset = [0, 330, 725, 1000, data.tvec.values[-1]]

t_reset = [0, 450,750, 1100, 1450, 1600, 1950, 2250, 2600,2950,3250,3700,4100,
           4375,4650,5025,5300,5950,6250,6900,7100,7550,7800,8200,8500,
           data.tvec.values[-1]]

#t_reset = [0, 50, 370, 700, 1050, data.tvec.values[-1]]


# =============================================================================
# load model data
# =============================================================================
#vessel data
parV = pickle.load( open("data\\parV_RVG4DOF.pkl", "rb" ) )

#actuator data
parA = pickle.load( open("data\\parA_RVG.pkl", "rb" ) )

#data.Azi = data.Azi*0+20

Uc=np.array(0.15) #current speed
betac = 70*np.pi/180#current direction
parS = {'Uc': Uc, 'betac': betac,'Fw':np.array([0,0,0,0])} #dict containing simulation param
xref = 2




# res = Run_Sim(data,parA,parV,parS,t_reset)
# res['v'] = res['v']+np.mean(data['v'])
# res['phi'] = res['phi']+np.mean(data['phi'])
# Plot_SimRes(res,channels,fignum,t_reset,xref,'Orange')




parV,parA = modpar.DefaultModelData()

xref=2

res = Run_Sim(data,parA,parV,parS,t_reset)
#res['v'] = res['v']+np.mean(data['v'])
res['phi'] = res['phi']+np.mean(data['phi'])
Plot_SimRes(res,channels,fignum,t_reset,xref,'purple')



# parS['Uc'] = 0.2
# res = Run_Sim(data,parA,parV,parS,t_reset)
# res['v'] = res['v']+np.mean(data['v'])
# res['phi'] = res['phi']+np.mean(data['phi'])
# Plot_SimRes(res,channels,fignum,t_reset,xref,'blue')


# Dl = parV['Dl']
# Dl[1,2] =  4*Dl[1,1]
# Dl[2,1] = Dl[1,2]
# parV['Dl'] = Dl


# res = Run_Sim(data,parA,parV,parS,t_reset)
# res['v'] = res['v']+np.mean(data['v'])
# res['phi'] = res['phi']+np.mean(data['phi'])
# Plot_SimRes(res,channels,fignum,t_reset,xref,'blue')

# parV['Ma'][2,2] = parV['Ma'][2,2]*20
# res = Run_Sim(data,parA,parV,parS,t_reset)
# res['v'] = res['v']+np.mean(data['v'])
# res['phi'] = res['phi']+np.mean(data['phi'])
# Plot_SimRes(res,channels,fignum,t_reset,xref,'red')



# parV['z_visc_ref'] = -2.6

# res = Run_Sim(data,parA,parV,parS,t_reset)
# res['v'] = res['v']+np.mean(data['v'])
# res['phi'] = res['phi']+np.mean(data['phi'])
# Plot_SimRes(res,channels,fignum,t_reset,xref,'red')


# =============================================================================
# simulation parameters
# =============================================================================

# inp = {}
# freq_index = [35,25,20]
# Cdy = [0.5,0.5,0.5]

# colstring = ['Purple','Red','Blue']

# for i in [0,1,2]:
#     inp = {}
#     inp['freq_index'] = freq_index[i]
#     inp['Cdy'] = Cdy[i]

#     parV, parA  = modpar.GenModelData(inp)

#     res = Run_Sim(data,parA,parV,parS,t_reset)
#     res['v'] = res['v']+np.mean(data['v'])
#     res['phi'] = res['phi']+np.mean(data['phi'])
#     Plot_SimRes(res,channels,fignum,t_reset,xref,colstring[i])
    
    
# plt.figure(2)
# plt.legend(['Exp','case0','case1','case2'])    



# =============================================================================
# simulation parameters
# =============================================================================

# inp = {}
# freq_index = [35,25,20]
# Cdy = [0.5,0.5,0.5]

# colstring = ['Purple','Red','Blue']

# for i in [0,1,2]:
#     inp = {}
#     inp['freq_index'] = freq_index[i]
#     inp['Cdy'] = Cdy[i]

#     parV, parA  = modpar.GenModelData(inp)

#     res = Run_Sim(data,parA,parV,parS,t_reset)
#     res['v'] = res['v']+np.mean(data['v'])
#     res['phi'] = res['phi']+np.mean(data['phi'])
#     Plot_SimRes(res,channels,fignum,t_reset,xref,colstring[i])
    
    
# plt.figure(2)
# plt.legend(['Exp','case0','case1','case2'])    









