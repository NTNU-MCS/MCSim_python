# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley 
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
This script prepares model data for Gunnerus maneuvering models

Vessel parameters are loaded from MATLAB, original source VERES. 
Additional hydrodynamic data are added by engineering judgement.

Gunnerus has two azimuth thrusters.Thruster location crudely estimated from 
drawings, dynamic time constants are a (somewhat qualified) guess. 
"""

#Load modules
import numpy as np
import scipy.io
import pickle


# =============================================================================
# 6DOF vessel parameters from VERES, extracted using matlab
# =============================================================================
file_name = 'RVGManModelParam6DOF.mat'

vessel= scipy.io.loadmat(file_name)
L = vessel['Lpp'][0,0] #length between perpendiculars
B = vessel['Bm'][0,0] #Breadth middle
T = vessel['Tm'][0,0] #Draught middle
Mrb = np.array(vessel['Mrb']) #rigid body mass
A = np.array(vessel['A']) #added mass (frequency and velocity dependent)
freq_index = 35 #index, should use low-frequency for manuevering but values seem off
vel_index = 4#index, 5.2m/s=10knots, Gunnerus cruising speed
reference_velocity = vessel['velocities'][0,vel_index]
Ma = A[:,:,freq_index,vel_index]
K = np.array(vessel['C']) #hydrostatic stiffness
K[:,5] = 0
K[5,:] = 0
Dl = np.array(vessel['Bv']) #linear damping matrix

Ba = np.array(vessel['B'])
Ba = Ba[:,:,freq_index,vel_index]

#Ma[3,1]=Ma[3,1]*10
#Ma[1,3]=Ma[1,3]*10
#Ma[3,5]=0
#Ma[5,3]=0

#Ma[2,:] = Ma[2,:]*0 
#Ma[4,:] = Ma[4,:]*0
#Ma[4,:] = Ma[2,:]*0 
#Ma[2,:] = Ma[4,:]*0

#Reducing linear damping, since quadratic drag is added below. 

Dl[0,0] = 0.05*Dl[0,0]
Dl[1,1] = 0.2*Dl[1,1]
Dl[5,5] = 0.2*Dl[5,5]


#source does not include heave and pitch damping, so adding some
#large values (we are not interested in heave/pitch response)
Dl[2,2] = Dl[1,1]*100 #heave=100*sway 
Dl[4,4] = Dl[5,5]*100 #pitch = 100*yaw

#add some order of magnitude quadratic damping in surge/sway/yaw
Cdx=0.125 #surge drag coefficient
Cdy=0.5 #sway drag coefficient

Du = Dl*0
Du[0,0]=0.5*Cdx*B*T*10**3 #quadratic damping coefficient in surge
Dv = Dl*0
Dv[1,1]=0.5*Cdy*L*T*10**3 #quadratic damping coefficient in sway
Dr = Dl*0
Dr[5,5] = 0.75*Dv[1,1]*L**4/64 #quadratic damping coefficient in yaw


#parV = {'Mrb':Mrb,'Ma':Ma,'Dl':Dl,'Du':Du,'Dr':Dr,'Dv':Dv,
#        'reference_velocity':reference_velocity}




# =============================================================================
# Roll stiffness for use in 6DOF model. Based on VERES data, but COG modified 
# to a more realistic value.
# =============================================================================
K44_VERES = vessel['C'][3,3] #VERES uses COG at waterline, probably due to lack
#of other information. For reference
GM_veres = 2.304# metacenter height with COG at waterline, for reference
COB = -1.131# rel waterline
Mass = 573140# kg, rigid body mass
Volume = Mass/1025#m^3, displacement
BM = GM_veres-COB #used to calculate Iwp
Iwp =  Volume*BM #second area moment of waterplane
COG_lightship = 0.68# rel waterline, from stability report
COG = 2#COG_lightship #assume COG of full ship is equal to lightship COG.
K44 = 9.81*1025*(Iwp+Volume*(COB-COG)) #estimated roll stiffness

K[3,3] = K44

#Roll quadratic damping
Dp = Dl*0

Dp[3,3] = Dv[1,1]*B**4/64 #quadratic damping coefficient in roll



COB_rel_COG = COB-COG

Dl[1,3] = -Dl[1,1]*COB_rel_COG
Dl[3,1] = -Dl[1,1]*COB_rel_COG

Ma[3,1] = -Ma[1,1]*COB_rel_COG
Ma[1,3] = -Ma[1,1]*COB_rel_COG
Ma[3,3] = Ma[3,3]+Ma[1,1]*(COB_rel_COG**2-COB**2)

# =============================================================================
# Actuator model parameters (equivalent thruster placed at centerline)
# =============================================================================
#coordinates rel COG (assumed reference point for VERES data)
x = -vessel['CG'][0,0]-L/2 
y = 2.7
z = -2.4-COG #estimate


rt1 = np.array([x,y,z]) #location of thruster 1
rt2 = np.array([x,-y,z]) #location of thruster 1
rt = np.array([x,0,z]) #location of equivalent thruster at centreline
Tazi = np.array([1]) #thruster azimuth angle dynamics time constants 
Trevs = np.array([1]) #thruster revolutions dynamics time constants 
dazi_max = 10*np.pi/180 #max azimuth rate
drevs_max = 10 #max revolutions rate

#dict containing actuator model parameters
parA = {'rt':rt,'Tazi':Tazi,'Trevs':Trevs,'dazi_max':dazi_max,'drevs_max':drevs_max} 


# =============================================================================
# Store data
# =============================================================================
#store 6DOF vessel data

parV = {'Mrb':Mrb,'Ma':Ma,'Dl':Dl,'Du':Du,'Dr':Dr,'Dv':Dv*0,'Dp':Dp*0,'K':K,
        'dvv':Dv[1,1],'z_b':COB_rel_COG,
        'reference_velocity':reference_velocity}

f = open("parV_RVG6DOF.pkl","wb")
pickle.dump(parV,f)
f.close()

#store actuator data
f = open("parA_RVG.pkl","wb")
pickle.dump(parA,f)
f.close()

#extract and store 3DOF vessel data 
DOF3 = [0,1,5] #surge sway yaw DOFs included in 3DOF model
DOF3mat = np.ix_(DOF3,DOF3) #for extracting 3x3 matrix from 6x6 matrix

#store 3DOF data in parV containing vessel parameters
parV = {'Mrb':Mrb[DOF3mat],'Ma':Ma[DOF3mat],
        'Dl':Dl[DOF3mat],'Du':Du[DOF3mat],
        'Dv':Dv[DOF3mat], 'Dr':Dr[DOF3mat],
        'reference_velocity':reference_velocity} #save in dict structure
f = open("parV_RVG3DOF.pkl","wb")
pickle.dump(parV,f)
f.close()


# =============================================================================
# Hydrodynamic derivatives pre elongation of RVG. Unknown source. 
# =============================================================================

Xudot=-79320.5895 
Yvdot=-408192.6004 
Yrdot=331671.6384 
Nvdot=331671.6384 
Nrdot=-12245783.9277 
Xvv=2880.5065 
Xrr=-29267.3371 
Xrv=-323707.1228 
Xuvv=-2371.1792 
Xrvu=33926.4945 
Xurr=83774.6548 
Yuv=-31000.2779 
Yur=277765.7816 
Yuur=11112.4159 
Yuuv=777.3214 
Yvvv=-11198.6727 
Yrrr=21117955.8908 
Yrrv=-12912371.0982 
Yvvr=102679.0573 
Nuv=309051.5592 
Nur=-2413124.664 
Nuur=-187538.9302 
Nuuv=-14041.6851 
Nvvv=-56669.589 
Nrrr=-179720233.7453 
Nrrv=-585678.1567 
Nvvr=-4278941.6707 
Xumodv=619.643 
Ymodrv=485260.3423 
Ymodvv=-21640.6686 
Ymodrr=-1020386.1325 
Ymodvr=321944.5769 
Nmodrv=1626540.037 
Nmodvv=116643.7578 
Nmodrr=-9397835.7612 
Nmodvr=-2258197.6812 
Xuu=-2100

print('Compare hydrodynamic added mass between ShipX and hydro derivatives')
print('Surge VERES', round(Ma[0,0]))
print('Surge DERIV', round(-Xudot))
print('Sway VERES', round(Ma[1,1]))
print('Sway DERIV', round(-Yvdot))
print('Yaw VERES', round(Ma[5,5]))
print('Yaw DERIV', round(Mrb[5,5]))
print('Yaw rigid body', round(-Nrdot))
print('Sway-Yaw VERES', round(Ma[1,5]))
print('Sway-Yaw DERIV', round(-Yrdot))
print('Yaw-Sway VERES', round(Ma[5,1]))
print('Yaw-Sway DERIV', round(-Nvdot))

print('Calculate equivalent drag coefficients for hydro derivatives')
T=vessel['Tm'][0,0]
rho = 1025
Cd_surge = -Xuu*2/B/T/rho
Cd_sway = -Ymodvv*2/L/T/rho
print('Cd surge', Cd_surge)
print('Cd sway', Cd_sway)