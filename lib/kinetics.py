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

def Cor4(nu,M,zref):
    """Simplified 4DOF Coriolis force
       Input 
           nu: 4x1 vector of body-fixed velocities 
           M: 4x4 inertia matrix 
           zref: Reference point for roll moment
       Output 
           F: 4x1 force vector
    """
    # Created: 2022-11-28 M.Marley 
    # Tested: -
    u = nu[0] #DOF1 surge
    v = nu[1] #DOF2 sway
    p = nu[2] #DOF4 yaw
    r = nu[3] #DOF6 roll
    
    M11 = M[0,0]
    M22 = M[1,1]
    M44 = M[2,2]
    M66 = M[3,3]

    M24 = M[1,2]
    M42 = M[2,1]

    M26 = M[1,3]
    M62 = M[3,1]

    M46 = M[2,3]
    M64 = M[3,2]
    
    F1 = -M22*v*r-0.5*(M26+M62)*r**2
    F2 = M11*u*r
    F4 = -F2*zref
    F6 = (M22-M11)*u*v+0.5*(M26+M62)*u*r
    
    F = np.array([F1,F2,F4,F6])

    return F

def crossflowdrag(v,r,parV):
    """Crossflow drag model
       Input 
           v: sway fluid relative speed 
           r: yaw rate
           parV: dict containing parameters
       Output 
           F2,F6: sway and yaw force
    """    
    x = parV['x_visc']
    dx = x[1]-x[0]
    
    Cd = parV['Cdy']
    Tm = parV['Tm'] #draft (moulded)
    
    vl = v+x*r #sway speed at each section
    dF = 0.5*1025*Cd*Tm*np.abs(vl)*vl*dx #force at each section
    F2 = np.sum(dF)
    F6 = np.sum(dF*x)
    
    
    return F2,F6


