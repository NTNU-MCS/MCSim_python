# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley input/output relations for individual functions
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Library of kinetics functions 
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
# ---------------------------------------------------------------------------

import numpy as np
import kinematics as km

# =============================================================================
# Functions
# =============================================================================
def Cor3(nu,M):
    """3DOF Coriolis matrix 
       Input 
           nu: 3x1 vector of body-fixed velocities 
           M: 3x3 mass matrix 
       Output 
           S: 3x3 matrix 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley 
    Cor3= [[0, 0, -M[1,1]*nu[1]-0.5*(M[1,2]+M[2,1])*nu[2]],
                [0, 0, M[0,0]*nu[0]],
                [M[1,1]*nu[1]+0.5*(M[1,2]+M[2,1])*nu[2], -M[0,0]*nu[0], 0]]
   
    return Cor3


def dot_nu3_man_lq(psi,nu,Uc,betaC,F,parV):
    """Derivative of 3DOF body-fixed velocity vector of ship moving in uniform 
    and steady currents. Linear+quadratic viscous load formulation modelled as
    dvisc=(Dl+Du*|u|+Dv*|v|+Dr*|r|)*nu, where each Di are matrices
    of coefficients.
    Input: 
            psi: ship heading
            nu: 3DOF body-fixed velocity vector
            Uc: Current speed
            betaC: current direction (going towards)
            F: external force vector (thruster forces, wave disturbance etc)
            parV: dict containing system matrices: Mrb, Ma, Dl, Du, Dv, Dr.
    Output: 
            d_nu3: body-fixed accelerations
    """   
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley 
    Mrb = parV['Mrb']
    Ma = parV['Ma']
    Dl = parV['Dl']
    Du = parV['Du']
    Dv = parV['Dv']
    Dr = parV['Dr']

    invM = np.linalg.inv(Mrb+Ma)
    nu_c_n = Uc*np.array([np.cos(betaC),np.sin(betaC),0]) #inertial frame current velocity
    nu_c = np.transpose(km.Rotz(psi))@nu_c_n #body-fixed current velocity
    S=km.Smat(np.array([0,0,nu[2]]))
    dnu_c = np.transpose(S@km.Rotz(psi))@nu_c_n
    nu_r = nu-nu_c
    
    D=Dl+Du*abs(nu_r[0])+Dv*abs(nu_r[1])+Dr*abs(nu[2])
    Ca = Cor3(nu_r,Ma)
    Crb = Cor3(nu,Mrb)
    
    d_nu3 = invM@(F-D@nu_r-Crb@nu-Ca@nu_r+Ma@dnu_c)
    return d_nu3    
  
def dot_nu6_man_lq(eta,nu,Uc,betaC,F,parV):
    """Derivative of 6DOF body-fixed velocity vector of ship moving in uniform 
    and steady currents. Linear+quadratic viscous load formulation modelled as
    dvisc=(Dl+Du*|u|+Dv*|v|+Dr*|r|+Dp*|p|)*nu, where each Di are matrices
    of coefficients. Assumes small angles in roll and pitch.
    Input: 
            eta: ship position and orientation
            nu: 6DOF body-fixed velocity vector
            Uc: Current speed
            betaC: current direction (going towards)
            F: external force vector (thruster forces, wave disturbance etc)
            parV: dict containing system matrices: Mrb, Ma, Dl, Du, Dv, Dr, Dp
    Output: 
            d_nu6: body-fixed accelerations
    """   
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley 
    Mrb = parV['Mrb']
    Ma = parV['Ma']
    Dl = parV['Dl']
    Du = parV['Du']
    Dv = parV['Dv']
    Dr = parV['Dr']
    Dp = parV['Dp']
    K = parV['K']
    
    invM = np.linalg.inv(Mrb+Ma)
    
    #calculate current kinematics and Coriolis forces in 3DOF horizontal plane
    DOF3 = [0,1,5] #surge sway yaw DOFs in 3DOF model
    DOF3mat = np.ix_(DOF3,DOF3) #for extracting 3x3 matrix from 6x6 matrix

    psi = eta[5]
    
    nu_c_n_3 = Uc*np.array([np.cos(betaC),np.sin(betaC),0]) #inertial frame current velocity
    nu_c_3 = np.transpose(km.Rotz(psi))@nu_c_n_3 #body-fixed current velocity
    S=km.Smat(np.array([0,0,nu[5]]))
    
    nu3 = nu[DOF3]
    nu_r_3 = nu3-nu_c_3
    Ca3 = Cor3(nu_r_3,Ma[DOF3mat])

    Crb3 = Cor3(nu3,Mrb[DOF3mat])        
    Fcor3 = -np.array(Crb3)@np.array(nu3)-np.array(Ca3)@np.array(nu_r_3)
    Fcor = F*0
    Fcor[DOF3] = Fcor3
    
    dnu_c_3 = np.transpose(S@km.Rotz(psi))@nu_c_n_3  
    
    Fcur_inertia3 = Ma[DOF3mat]@dnu_c_3
    Fcur_inertia = F*0
    Fcur_inertia[DOF3] = Fcur_inertia3

    nu_c = nu*0
    nu_c[DOF3] = nu_c_3
    nu_r = nu-nu_c

    #Nonlinear damping matrix
    D=Dl+Du*abs(nu_r[0])+Dv*abs(nu_r[1])+Dp*abs(nu[3])+Dr*abs(nu[5])
#    print(D@nu_r)

    d_nu6 = invM@(F+Fcor+Fcur_inertia-D@nu_r-K@eta)
    return d_nu6