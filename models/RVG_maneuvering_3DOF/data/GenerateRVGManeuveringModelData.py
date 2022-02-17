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
Additional hydrodynamic data are added by engineering judgement, 
for example purposes only.

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
L = vessel['Lpp'] #length between perpendiculars
B = vessel['Bm'] #Breadth middle
T = vessel['Tm'] #Draught middle
Mrb = np.array(vessel['Mrb']) #rigid body mass
A = np.array(vessel['A']) #added mass (frequency and velocity dependent)
freq_index = 0 #index, use low-frequency for manuevering
vel_index = 4#index, 5.2m/s=10knots, Gunnerus cruising speed
reference_velocity = vessel['velocities'][0,vel_index]
Ma = A[:,:,freq_index,vel_index]
K = np.array(vessel['C']) #hydrostatic stiffness
Dl = np.array(vessel['Bv']) #linear damping matrix

#source does not include heave and pitch damping, so adding some
#large values (we are not interested in heave/pitch response)
Dl[2,2] = Dl[1,1]*10 #heave=10*sway 
Dl[4,4] = Dl[5,5]*10 #pitch = 10*yaw

#add some order of magnitude quadratic damping in surge/sway/yaw
Cdx=0.5 #surge drag coefficient
Cdy=1 #sway drag coefficient

Du = Dl*0
Du[0,0]=0.5*Cdx*B*T*10**3 #quadratic damping coefficient in surge
Dv = Dl*0
Dv[1,1]=0.5*Cdx*L*T*10**3 #quadratic damping coefficient in sway
Dr = Dl*0
Dr[5,5] = Dv[1,1]*L**4/64 #quadratic damping coefficient in yaw


parV = {'Mrb':Mrb,'Ma':Ma,'Dl':Dl,'Du':Du,'Dr':Dr,'Dv':Dv,
        'reference_velocity':reference_velocity}



# =============================================================================
# Actuator model parameters (equivalent thruster placed at centerline)
# =============================================================================
#coordinates rel COG (assumed reference point for VERES data)
x = -vessel['CG'][0,0]-L/2 
y = 2.7
z = -2.4 #estimate

rt1 = np.array([x[0,0],y,z]) #location of thruster 1
rt2 = np.array([x[0,0],-y,z]) #location of thruster 1
T = np.array([[1,0],[0,1]]) #thruster dynamics time constants 
#dict containing actuator model parameters
parA = {'rt1':rt1,'rt2':rt2,'T':T} 


# =============================================================================
# Store data
# =============================================================================
#store 6DOF vessel data
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





    
 
