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

Model reference point is at waterline above centre of buoyancy.

Gunnerus has two azimuth thrusters.Thruster location crudely estimated from 
drawings, dynamic time constants are a (somewhat qualified) guess. 
"""

#Load modules
import numpy as np
import scipy.io
import pickle

# =============================================================================
# Some inputs
# =============================================================================

GMT = 1.7 #ranges from 1.608 to 1.8895 depending on loading condition
z_b = -2.4 #attack point for quadratic roll-sway damping
# ============================================================================
# 6DOF vessel parameters from VERES, extracted using matlab
# =============================================================================


file_name = 'RVGManModelParam6DOF.mat'
vessel= scipy.io.loadmat(file_name)

L = vessel['Lpp'][0,0] #length between perpendiculars
B = vessel['Bm'][0,0] #Breadth middle
T = vessel['Tm'][0,0] #Draught middle

# =============================================================================
# Calculate stiffness and rigid body mass matrix wrt to true COG
# =============================================================================

CG_veres = vessel['CG'][0]
GMT_veres = vessel['GM_T'][0][0]  # metacenter height with COG at waterline
K = np.array(vessel['C']) #hydrostatic stiffness from VERES
Mrb_veres = np.array(vessel['Mrb']) #rigid body mass

K44_veres = vessel['C'][3,3] #VERES uses COG at waterline, probably due to lack
#of other information. For reference
CB_veres = vessel['CB'][0]# rel waterline
CB_rel_wl = CB_veres[2]-T
Mass = Mrb_veres[0,0]# kg, rigid body mass
Volume = Mass/1025#m^3, displacement
BMT = GMT_veres-CB_rel_wl #used to calculate Iwp
IwpT =  Volume*BMT #second area moment of waterplane
K44 = Mass*GMT*9.81
CG_rel_wl = CB_rel_wl+(IwpT-K44/(9.81*1025))/Volume

GML_veres = vessel['GM_L'][0] # metacenter height with COG at waterline
GML = GML_veres-CG_rel_wl
K55 = Mass*GML*9.81

K[3,3] = K44 
K[4,4] = K55


COB_rel_COG = CB_rel_wl-CG_rel_wl

Mrb = np.zeros((6,6))
Mrb[0,0] = Mass
Mrb[1,1] = Mass
Mrb[2,2] = Mass

Mrb[3,3] = Mrb_veres[3,3]+Mass*CG_rel_wl**2
Mrb[1,3] = -Mass*CG_rel_wl
Mrb[3,1] = -Mass*CG_rel_wl

Mrb[4,4] = Mrb_veres[4,4]+Mass*CG_rel_wl**2
Mrb[0,4] = Mass*CG_rel_wl
Mrb[4,0] = Mass*CG_rel_wl

Mrb[5,5] = Mrb_veres[5,5]


# =============================================================================
# Extract added mass and radiation damping
# =============================================================================


A = np.array(vessel['A']) #added mass (frequency and velocity dependent)
Btmp = np.array(vessel['B']) #added mass (frequency and velocity dependent)
freq_index = 35 #index, should use low-frequency for manuevering but values
#seem of so use other period instead
vel_index = 4#index, 5.2m/s=10knots, Gunnerus cruising speed
reference_velocity = vessel['velocities'][0,vel_index]


Ma = np.zeros((6,6))
Ba = np.zeros((6,6))

Ma_lf = A[:,:,35,vel_index] #inf added mass
Ba_lf = Btmp[:,:,1,vel_index] #low-freq damping. 


Ma[0,0] = Ma_lf[0,0] #surge
Ma[1,1] = Ma_lf[1,1] #sway-yaw subsystem
Ma[1,5] = Ma_lf[1,5] #sway-yaw subsystem
Ma[5,1] = Ma_lf[5,1] #sway-yaw subsystem
Ma[5,5] = Ma_lf[5,5] #sway-yaw subsystem

Ba[0,0] = Ba_lf[0,0] #surge
Ba[1,1] = Ba_lf[1,1] #sway-yaw subsystem
Ba[1,5] = Ba_lf[1,5] #sway-yaw subsystem
Ba[5,1] = Ba_lf[5,1] #sway-yaw subsystem
Ba[5,5] = Ba_lf[5,5] #sway-yaw subsystem

freq_index = 22 #corresponds to 8sec, use for stiff DOFs

Ma_wf = A[:,:,freq_index,vel_index]
Ba_wf = Btmp[:,:,freq_index,vel_index]

Ma[2,2] = Ma_wf[2,2] #heave
Ma[2,4] = Ma_wf[2,4]#0.5*(Ma_wf[2,4]+Ma_wf[4,2]) #heave-pitch
Ma[4,2] = Ma_wf[4,2]#Ma[2,4] #heave-pitch
Ma[2,4] = 0.5*(Ma_wf[2,4]+Ma_wf[4,2]) #heave-pitch
Ma[4,2] = Ma[2,4] #heave-pitch

Ma[4,4] = Ma_wf[4,4] #pitch

Ma[3,3] = Ma_wf[3,3] #roll
Ma[3,1] = Ma_wf[3,1] #roll-sway
Ma[1,3] = Ma_wf[1,3] #roll-sway
Ma[3,5] = Ma_wf[3,5] #roll-yaw
Ma[5,3] = Ma_wf[5,3] #roll-yaw

Ba[2,2] = Ba_wf[2,2]*5 #heave
Ba[2,4] = Ba_wf[2,4] #heave-pitch
Ba[4,2] = Ba_wf[4,2] #heave-pitch
Ba[4,4] = Ba_wf[4,4]*5 #pitch

Ba[3,3] = Ba_wf[3,3]*2 #roll
Ba[3,1] = Ba_wf[3,1] #roll-sway
Ba[1,3] = Ba_wf[1,3] #roll-sway
Ba[3,5] = Ba_wf[3,5] #roll-yaw
Ba[5,3] = Ba_wf[5,3] #roll-yaw


Dl = Ba #anp.array(vessel['Bv']) #linear damping matrix


#source does not include heave and pitch damping, so adding some
#large values (we are not interested in heave/pitch response)
#Dl[2,2] = Dl[1,1]*100 #heave=100*sway 
#Dl[4,4] = Dl[5,5]*100 #pitch = 100*yaw

#add some order of magnitude quadratic damping in surge/sway/yaw
Cdx=0.12 #surge drag coefficient
Cdy=0.4 #sway drag coefficient

Du = Dl*0
Du[0,0]=0.5*Cdx*B*T*10**3 #quadratic damping coefficient in surge
Dv = Dl*0
Dv[1,1]=0.5*Cdy*L*T*10**3 #quadratic damping coefficient in sway
Dr = Dl*0
Dr[5,5] = 0.75*Dv[1,1]*L**4/64 #quadratic damping coefficient in yaw


#parV = {'Mrb':Mrb,'Ma':Ma,'Dl':Dl,'Du':Du,'Dr':Dr,'Dv':Dv,
#        'reference_velocity':reference_velocity}





#Roll quadratic damping
Dp = Dl*0

Dp[3,3] = Dv[1,1]*B**4/64 #quadratic damping coefficient in roll


# =============================================================================
# Actuator model parameters (equivalent thruster placed at centerline)
# =============================================================================
#coordinates rel waterline
x = vessel['CG'][0,0]-L/2+2
y = 2.7
z = -2.2 #estimate


rt1 = np.array([x,y,z]) #location of thruster 1
rt2 = np.array([x,-y,z]) #location of thruster 1
rt = np.array([x,0,z]) #location of equivalent thruster at centreline
Tazi = np.array([1]) #thruster azimuth angle dynamics time constants 
Trevs = np.array([1]) #thruster revolutions dynamics time constants 
dazi_max = 10*np.pi/180 #max azimuth rate
drevs_max = 10 #max revolutions rate

#dict containing actuator model parameters
parA = {'rt':rt,'rt1':rt1,'rt2':rt2,'Tazi':Tazi,'Trevs':Trevs,'dazi_max':dazi_max,'drevs_max':drevs_max} 


# =============================================================================
# Store data
# =============================================================================
#store 6DOF vessel data



parV = {'Mrb':Mrb,'Ma':Ma,'Dl':Dl,'Du':Du,'Dr':Dr,'Dv':Dv*0,'Dp':Dp*0,'K':K,
        'dvv':Dv[1,1],'z_b':z_b,
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
f = open("parV_RVG3DOF_infA.pkl","wb")
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