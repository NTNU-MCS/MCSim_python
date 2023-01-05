# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: 2022-04-29	M. Marley Updated thruster model
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley Checked that results make sense
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Library of functions for running the 3DOF RVG maneuvering model. 
May also be used to model any vessel with two azimuth thrusters, by loading
desired vessel and actuator data. 
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
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
# Import modules
# =============================================================================
import numpy as np
import kinematics as km
import kinetics as kt
import thrusters as th

# =============================================================================
# Functions
# =============================================================================    

    
def dot_RVG_Man_4DOF(x,tau,w,parV,parA,parS):
    """Time derivative of RVG 6DOF vessel model, moving in uniform
    and steady currents. Linear stiffness matrix, consistent with small angle
    assumption in roll and pitch. Focus is on horizontal DOFs + roll. 
    Uses a linear+quadratic damping formulation. 
    Uses the thrusters.RVGazimuth_man function, with the two RVG azimuth 
    thrusters are modelled as an equivalent thruster located at centreline. 
    Thruster states are azimuth angle and propeller revs. 
    Control input is desired thruster states, while disturbance input is 
    body-fixed forces (e.g. wind and wave loads). 
    Input: 
        x=[eta,nu,azi,revs] in R^14: state vector, where 
            eta in R^6: position and orientation
            nu in R^6: body-fixed velocities 
            azi in R: thruster azimuth angle
            revs in R: propeller revs  [RPM] 
        tau =[azi_d, revs_d] in R^2: desired thruster states
        w in R^3: external force vector acting on body   
        parV: dict containing vessel parameters 
        parA: dict containing actuator parameters          
        parS: dict containing simulation parameters 
    Output: 
        dot_x: 14x1 vector of time derivatives   
    """
    # Created: 2022-11-28 M.Marley 
    # Tested: 2022-11-28 M.Marley  
    
    # =============================================================================
    # Kinematics
    # =============================================================================
    
    eta = x[0:4] #[0:6] 
    psi = x[3]
    nu=x[4:8]  
    
    dx = np.zeros(len(x))
    
    dx[0:4] = km.dot_eta4(psi,nu)    
    
    # =============================================================================
    # Current kinematics
    # =============================================================================    
    
    u = nu[0] #DOF1 surge
    v = nu[1] #DOF2 sway
    p = nu[2] #DOF4 yaw
    r = nu[3] #DOF6 roll
    
    azi = x[8]
    revs = x[9]
    rt1 = parA['rt1'] #thruster 1 location
    rt2 = parA['rt2'] #thruster 2 location
    
    nu_c_n_2 = parS['Uc']*np.array([np.cos(parS['betac']),np.sin(parS['betac'])]) #inertial frame current velocity
    
    R = km.Rot_planar(psi)
    
    nu_c_2 = np.transpose(R)@nu_c_n_2 #body-fixed current velocity
    u_c = nu_c_2[0]
    v_c = nu_c_2[1]

    ur = u-u_c
    vr = v-v_c

    S = np.array([[0,-1],[1,0]])

    dnu_c_2 = r*np.transpose(S*R)@nu_c_n_2
    dnu_c = np.array([dnu_c_2[0],dnu_c_2[1],0,0])

    nu_r = nu-np.array([u_c,v_c,0,0])

    #two thruster model
    u_t1 = ur-r*rt1[1] #surge velocity at thruster location
    v_t1 = vr +r*rt1[0]#sway velocity at thruster location 


    Fx1, Fy1 = th.RVGazimuth_man(u_t1,v_t1,azi,revs) 
    ForceAct1 = np.array([Fx1,Fy1,Fy1*rt1[2],Fy1*rt1[0]-Fx1*rt1[1]])        
    
    u_t2 = ur-r*rt2[1] #surge velocity at thruster location
    v_t2 = vr +r*rt2[0]#sway velocity at thruster location 

    Fx2, Fy2 = th.RVGazimuth_man(u_t2,v_t2,azi,revs) 
    ForceAct2 = np.array([Fx2,Fy2,Fy2*rt2[2],Fy2*rt2[0]-Fx2*rt2[1]])       

    Fact = (ForceAct1+ForceAct2)
    
    Fx = Fact[0]
    
    #Coriolis forces
    
    Mrb = parV['Mrb']

    Fcorr_RB = kt.Cor4(nu,Mrb,parV['CG_rel_wl'])
    
    Ma = parV['Ma']

    Fcorr_a = kt.Cor4(nu_r,Ma,parV['z_a_ref'])
      
    #viscous damping in sway and heave
    
    Cdx = parV['Cdx0']+parV['Cdx1']*np.abs(np.arctan2(vr,ur))
    #print(Cdx)
    
    F1 = 0.5*1025*Cdx*parV['Tm']*parV['Bm']*ur*np.abs(ur)
    
    F2,F6 = kt.crossflowdrag(vr-parV['z_visc_ref']*p,r,parV)
    
    F4 = -F2*parV['z_visc_ref']
    
    F_visc = np.array([F1,F2,F4,F6])

    
    
    invM = np.linalg.inv(Mrb+Ma)
    D = parV['Dl']
    K = parV['K']
    
    dx[4:8] = invM@(Fact-Fcorr_RB-Fcorr_a-F_visc-D@nu_r-K@eta+Ma@dnu_c+w)
    
    
    dx[8] = -(1/parA['Tazi'])*(x[8]-tau[0])
    dx[8] = np.sign(dx[8])*np.min([np.abs(dx[8]), parA['dazi_max']])
    dx[9] = -(1/parA['Trevs'])*(x[9]-tau[1])
    dx[9] = np.sign(dx[9])*np.min([np.abs(dx[9]), parA['drevs_max']])
    return dx, Fx  

def int_RVGMan4(x,u,w,parV,parA,parS):
    """Time integration of RVG 6DOF vessel model, moving in uniform
    and steady currents. Linear stiffness matrix, consistent with small angle
    assumption in roll and pitch. Focus is on horizontal DOFs + roll. 
    Uses a linear+quadratic damping formulation. 
    Uses the thrusters.RVGazimuth_man function, with the two RVG azimuth 
    thrusters are modelled as an equivalent thruster located at centreline. 
    Thruster states are azimuth angle and propeller revs. 
    Control input is desired thruster states, while disturbance input is 
    body-fixed forces (e.g. wind and wave loads). 
    Input: 
        x=[eta,nu,azi,revs] in R^14: state vector, where 
            eta in R^6: position and orientation 
            nu in R^6: body-fixed velocities 
            azi in R: thruster azimuth angle
            revs in R: propeller revs  [RPM] 
        u =[azi_d, revs_d] in R^2: desired thruster states
        w in R^3: external force vector acting on body   
        parV: dict containing vessel parameters 
        parA: dict containing actuator parameters          
        parS: dict containing simulation parameters 
    Output: 
        x_next: 14x1 vector of vessel state at next time step  
    """
    # Created: 2022-05-02 M.Marley 
    # Tested: 2022-05-02 M.Marley  

    dt = parS['dt']
    k1, Fx = dot_RVG_Man_4DOF(x,u,w,parV,parA,parS)
    k2, Fx = dot_RVG_Man_4DOF(x+k1*dt/2,u,w,parV,parA,parS)
    k3, Fx = dot_RVG_Man_4DOF(x+k2*dt/2,u,w,parV,parA,parS)
    k4, Fx = dot_RVG_Man_4DOF(x+k3*dt/2,u,w,parV,parA,parS)
    x_next = x+(k1+2*k2+2*k3+k4)*dt/6
    x_next[5]=km.rad2pipi(x_next[5])

    return x_next, Fx
