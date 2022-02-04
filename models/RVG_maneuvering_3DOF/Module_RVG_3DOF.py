# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# Created By: M. Marley
# Created Date: 2022-02-04
# Version = '1.0'
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 input/output relations for individual functions
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Library of functions for running the 3DOF RVG maneuvering model
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
# ---------------------------------------------------------------------------


import numpy as np
import kinematics as km
import kinetics as kt
import thrusters as th

    
def dot_GunMan3_lq(x,u,w,parV,parA,parS):
    """Time derivative of Gunnerus 3DOF maneuvering model with
        linear+quadratic damping formulation
    Input: 
            x=[eta,nu,thrust1states,thrust2states] in R^10: state vector
                eta in R^3: position and heading
                nu in R^3: body-fixed velocities
                thrust1states in R^2: thrustforce and angle of thruster 1
                thrust2states in R^2: thrustforce and angle of thruster 2
         
            thrust_d in R^4: desired thruster states
            Fw in R^3: external force vector acting on body
                
            parV: dict containing vessel parameters
            parA: dict containing actuator parameters               
            parS: dict containing simulation parameters

    Output: 
            dot_x   
    """
    Uc = parS['Uc']
    betac=parS['betac']
    
    #inertial frame current velocity:
    nu_c_n = Uc*[np.cos(betac),np.sin(betac),0] 
    psi=x[2]
    nu=x[3:6]
    ForceAct = ml.forceAzi3(x[6],x[7],parA['rt1']) 
    ForceAct = ForceAct+ml.forceAzi3(x[8],x[9],parA['rt2'])
    F=ForceAct+w
    
    dot_x = np.zeros(len(x))
    dot_x[0:3] = ml.dot_eta3(psi, nu)
    dot_x[3:6] = ml.dot_nu3_man_lq(psi, nu, nu_c_n, F, parV)
    dot_x[6:8] = -parA['T']@(x[6:8]-u[0:2])
    dot_x[8:10] = -parA['T']@(x[8:10]-u[2:4])
    
    return dot_x


def int_RVGMan3_lq(x,u,w,parV,parA,parS):
    """Time integration of Gunnerus 3DOF maneuvering model with
        linear+quadratic damping formulation. Uses Runge-Kutta 4
        integration. 
    Input: 
            x=[eta,nu,thrust1states,thrust2states] in R^10: state vector
                eta in R^3: position and heading
                nu in R^3: body-fixed velocities
                thrust1states in R^2: force and angle of thruster 1
                thrust2states in R^2: force and angle of thruster 2
         
            u in R^4: desired thruster states
            w in R^3: disturbance force vector acting on body
                
            parV: dict containing vessel parameters
            parA: dict containing actuator parameters               
            parS: dict containing simulation parameters

    Output: 
            x_next: integrated state
    """
    dt = parS['dt']
    k1 = dot_GunMan3_lq(x,u,w,parV,parA,parS)
    k2 = dot_GunMan3_lq(x+k1*dt/2,u,w,parV,parA,parS)
    k3 = dot_GunMan3_lq(x+k2*dt/2,u,w,parV,parA,parS)
    k4 = dot_GunMan3_lq(x+k3*dt/2,u,w,parV,parA,parS)
    x_next = x+(k1+2*k2+2*k3+k4)*dt/6
    x_next[2]=ml.rad2pipi(x_next[2])
    
    return x_next
   
    
    

