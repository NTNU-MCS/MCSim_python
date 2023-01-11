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

# =============================================================================
# compare with test data
# =============================================================================
plt.close('all')

exp_data = pickle.load( open("data\\maneuvering_data.pkl", "rb" ) )

revmax = exp_data['stbd_RPMOrder']['value'].values*0+205/2.185
exp_data['stbd_RPMOrder']['value'] = np.min([exp_data['stbd_RPMOrder']['value'].values,revmax],0)

channels = ['LonGroundSpeed','TransGroundSpeed','YawRate','Roll','port_AzimuthOrder','stbd_RPMOrder','Heading']
scale = np.array([1./1.9834,-1./1.9834,-1,1,-1,2.185,-1])
fignum = [6,7,9,4,10,11,5]

test1 = {'tstart':1150,'tend':1530,'name':'speed_test','description':'Speed Test 80%','channels':channels}

test2 = {'tstart':1400,'tend':1800,'name':'turning_circle','description':'Turning circle 30deg','channels':channels}

test3 = {'tstart':1850,'tend':2250,'name':'speed_test','description':'Speed Test 80%','channels':channels}

test4 = {'tstart':2150,'tend':2500,'name':'turning_circle','description':'Turning circle 30deg','channels':channels}

test5 = {'tstart':2600,'tend':2850,'name':'zig_zag','description':'Zig zag 10/10','channels':channels}

test6 = {'tstart':2900,'tend':3300,'name':'zig_zag','description':'Zig zag 10/20','channels':channels}

test7 = {'tstart':3300,'tend':3700,'name':'turning_circle','description':'Turning circle 10deg','channels':channels}

test8 = {'tstart':3600,'tend':4000,'name':'zig_zag','description':'Zig zag 5/10','channels':channels}

test9 = {'tstart':3950,'tend':4290,'name':'turning_circle','description':'Turning circle 10','channels':channels}

test10 = {'tstart':4300,'tend':4750,'name':'zig_zag','description':'Zig zag 10/20','channels':channels}

test11 = {'tstart':4750,'tend':5200,'name':'zig_zag','description':'Zig zag 5/10','channels':channels}

test12 = {'tstart':5200,'tend':5400,'name':'zig_zag','description':'Zig zag 20/20','channels':channels}

test13 = {'tstart':5400,'tend':5610,'name':'zig_zag','description':'Zig zag 10/10','channels':channels}

test14 = {'tstart':5650,'tend':5950,'name':'turning_circle','description':'Turning circle 30deg','channels':channels}

test15 = {'tstart':6000,'tend':6350,'name':'turning_circle','description':'Turning circle 30deg','channels':channels}

test16 = {'tstart':6400,'tend':7000,'name':'speed_test','description':'Speed test','channels':channels}

test17 = {'tstart':6900,'tend':7250,'name':'man_over_board','description':'Man over board','channels':channels}

test18 = {'tstart':7500,'tend':8000,'name':'speed_test','description':'Speed test','channels':channels}

test19 = {'tstart':8100,'tend':8600,'name':'turning circle','description':'Turning circle 30','channels':channels}

test20 = {'tstart':8550,'tend':8850,'name':'turning circle','description':'Turning circle 20','channels':channels}

test21 = {'tstart':8850,'tend':9250,'name':'speed_test','description':'Speed test','channels':channels}

test22 = {'tstart':9250,'tend':9500,'name':'turning_circle','description':'Turning circle 20','channels':channels}

test23 = {'tstart':9550,'tend':9900,'name':'speed_test','description':'Speed test','channels':channels}

testall = {'tstart':1050,'tend':9900,'name':'speed_test','description':'Speed test','channels':channels}

testmid = {'tstart':2600,'tend':7250,'name':'speed_test','description':'Speed test','channels':channels}
testmid = {'tstart':3000,'tend':4220,'name':'speed_test','description':'Speed test','channels':channels}

#testall['tend'] = 3000
test = testall
test = testmid
test['scale'] = scale
test['fignum'] = fignum




long,lat,res, t0 = MTD.get_timeseries(exp_data,test)

plt.figure(1)
plt.plot(long,lat)


# =============================================================================
# load model data
# =============================================================================
#vessel data
parV = pickle.load( open("data\\parV_RVG4DOF.pkl", "rb" ) )


#actuator data
parA = pickle.load( open("data\\parA_RVG.pkl", "rb" ) )

# =============================================================================
# simulation parameters
# =============================================================================

dt=0.5 #time step

df1 = res['port_AzimuthOrder']
df2 = res['stbd_RPMOrder']
ts,azidata,revsdata = MTD.interpolate_sampling(df1,df2)
ts = ts-t0

tmax = np.floor(ts[-1])
tmin = np.ceil(ts[0])
tvec = np.linspace(tmin,tmax,int((tmax-tmin)/dt)+1)



f1 = interpolate.interp1d(ts, azidata)
azi = -f1(tvec)*np.pi/180

f1 = interpolate.interp1d(ts, revsdata)
revs = f1(tvec)*2.185

Uc=np.array(0.15) #current speed
betac = (-10)*np.pi/180#current direction
parS = {'dt':dt, 'Uc': Uc, 'betac': betac} #dict containing simulation param


# =============================================================================
# Control parameters
# =============================================================================
Kp = 1 #heading proportional gain
Kd = 20 #heading derivative gain

azi_sat = np.pi/8

Uref=parV['reference_velocity']
#thrustforce = parV['Dl'][0,0]*Uref+0.5*parV['Cdx']*10*3*Uref**2

# =============================================================================
# Initial conditions and allocate memory
# =============================================================================

u0 = res['LonGroundSpeed'].value.values[0]/1.9834
v0 = -res['TransGroundSpeed'].value.values[0]/1.9834
psi0 = -res['Heading'].value.values[0]*np.pi/180+np.pi/2
r0 = -res['YawRate'].value.values[0]*np.pi/180


thrust_state = np.array([azi[0],revs[0]]) #azimuth angle and revs
nu=np.array([u0,v0,0,r0]) #u v p r
eta=np.array([0,0,0,psi0]) #x y phi psi
x=np.concatenate((eta,nu,thrust_state))
x_out=np.zeros([len(x),len(tvec)])
ForceAct = np.zeros([4,len(tvec)])


tvec = tvec

t_reset = [370,650, 1080, 1400, 1750, 2140, 2500, 2850, 3200, 3620, 4000, 4250, 4520, 4900, 5200, 5800, 6200, 6950, 7420, 7700, 8100, 8400]


# =============================================================================
# run simulation
# =============================================================================
for i1, t in enumerate(tvec):
    if t in t_reset:

        df = res['LonGroundSpeed']
        df = MTD.testdata(df,t+t0,t+t0+10)

        u0 = df.value.values[0]/1.9834
        v0 = -res['TransGroundSpeed'].value.values[0]/1.9834    
        psi0 = -res['Heading'].value.values[0]*np.pi/180+np.pi/2
        r0 = -res['YawRate'].value.values[0]*np.pi/180
        nu=np.array([u0,v0,0,r0]) #u v p r
        eta=np.array([0,0,0,psi0]) #x y phi psi
   #     x=np.concatenate((eta,nu,thrust_state))

    
    #store results
    x_out[:,i1] = x
    
    
    
    #control system (same azimuth angle for both thrusters)
        
    u = np.array([azi[i1],revs[i1]])

    #time integration
    Fw = np.zeros(4) #no disturbance force
    Fw[1] = -10**4
    x, Fx = model.int_RVGMan4(x, u, Fw, parV, parA, parS)

  
#sort output
eta=x_out[0:4,:]
nu=x_out[4:8,:]
thruststates = x_out[8:10,:]

# =============================================================================
# plot
# =============================================================================
figxy = [10,5] #figure size

plt.figure(1)#,figsize=figxy, clear='True')
plt.plot(eta[0,:],eta[1,:])
plt.title('Trajectory')
plt.xlabel('East [m]')
plt.ylabel('North [m]')
plt.axis('equal')
plt.legend(['Experiment','Simulation'])
plt.tight_layout()
plt.savefig('Trajectory'+'.pdf')


titles = ['x position','y position','roll','yaw',
          'surge velocity','sway velocity','roll velocity',
           'yaw velocity','azimuth angle','thruster revs']
units = ['m','m','deg','deg','m/s','m/s','deg/s','deg/s','deg','RPM']
scale = [1,1,180/np.pi,180/np.pi,1,1,180/np.pi,180/np.pi,180/np.pi,1]
parP={'titles':titles,'units':units}
parP['fig0'] = 2
parP['scale'] = scale
#parP['figsize']=figxy

v = x_out[5]
p = x_out[6]
r = x_out[7]

x_out[5] =v + r*0-p*0

viz.plot_timeseries(tvec,x_out,parP)

#x_out[5] =v + r*0-p*10

#viz.plot_timeseries(tvec,x_out,parP)


for num in fignum:
    
    plt.figure(num)
    plt.legend(['Experiment','Simulation','+r'])
    plt.tight_layout()
    plt.savefig(titles[num-2]+'.pdf')

