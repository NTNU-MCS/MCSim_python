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
def dot_RVG_Man_3DOF_lq(x,u,w,parV,parA,parS):
    """Time derivative of RVG 3DOF vessel model, moving in uniform
    and steady currents. Uses a linear+quadratic damping formulation. 
    Uses the thrusters.RVGazimuth_man function, with the two RVG azimuth 
    thrusters are modelled as an equivalent thruster located at centreline. 
    Thruster states are azimuth angle and propeller revs. 
    Control input is desired thruster states, while disturbance input is 
    body-fixed forces (e.g. wind and wave loads). 
    Input: 
        x=[eta,nu,azi,revs] in R^8: state vector, where 
            eta in R^3: position and heading 
            nu in R^3: body-fixed velocities 
            azi in R: thruster azimuth angle
            revs in R: propeller revs  [RPM] 
        u =[azi_d, revs_d] in R^2: desired thruster states
        w in R^3: external force vector acting on body   
        parV: dict containing vessel parameters 
        parA: dict containing actuator parameters          
        parS: dict containing simulation parameters 
    Output: 
        dot_x: 8x1 vector of time derivatives   
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley  
    psi=x[2]
    nu=x[3:6]
    azi = x[6]
    revs = x[7]
    rt = parA['rt'] #thruster location
    
    u_t = nu[0] #surge velocity at thruster location
    v_t = nu[1]+nu[2]*rt[0]#sway velocity at thruster location
    
    
    Fx, Fy = th.RVGazimuth_man(u_t,v_t,azi,revs)
    
    
#    ForceAct = th.forceAzi3(x[6],x[7],parA['rt1']) 
#    ForceAct = ForceAct+th.forceAzi3(x[8],x[9],parA['rt2'])
    ForceAct = np.array([Fx,Fy,Fy*rt[0]])    
    F=ForceAct+w

    dx = np.zeros(len(x))
    dx[0:3] = km.dot_eta3(psi, nu)
    
    dx[3:6] = kt.dot_nu3_man_lq(psi, nu,parS['Uc'],parS['betac'], F, parV)
    dx[6] = -parA['Tazi']*(x[6]-u[0])
    dx[7] = -parA['Trevs']*(x[7]-u[1])
    return dx


def int_RVGMan3_lq(x,u,w,parV,parA,parS):
    """Time integration of RVG 3DOF vessel model, moving in uniform
    and steady currents. Uses a linear+quadratic damping formulation. 
    Uses the thrusters.RVGazimuth_man function, with the two RVG azimuth 
    thrusters are modelled as an equivalent thruster located at centreline. 
    Thruster states are azimuth angle and propeller revs. 
    Control input is desired thruster states, while disturbance input is 
    body-fixed forces (e.g. wind and wave loads). 
    Input: 
        x=[eta,nu,azi,revs] in R^8: state vector, where 
            eta in R^3: position and heading 
            nu in R^3: body-fixed velocities 
            azi in R: thruster azimuth angle
            revs in R: propeller revs  [RPM] 
        u =[azi_d, revs_d] in R^2: desired thruster states
        w in R^3: external force vector acting on body   
        parV: dict containing vessel parameters 
        parA: dict containing actuator parameters          
        parS: dict containing simulation parameters 
    Output: 
        x_next: 8x1 vector of vessel state at next time step  
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley  
    dt = parS['dt']
    k1 = dot_RVG_Man_3DOF_lq(x,u,w,parV,parA,parS)
    k2 = dot_RVG_Man_3DOF_lq(x+k1*dt/2,u,w,parV,parA,parS)
    k3 = dot_RVG_Man_3DOF_lq(x+k2*dt/2,u,w,parV,parA,parS)
    k4 = dot_RVG_Man_3DOF_lq(x+k3*dt/2,u,w,parV,parA,parS)
    x_next = x+(k1+2*k2+2*k3+k4)*dt/6
    x_next[2]=km.rad2pipi(x_next[2])
    
    return x_next

    
    

