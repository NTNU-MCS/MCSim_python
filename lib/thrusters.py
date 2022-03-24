# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: 2022-03-24	M. Marley added RVG thruster model
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley input/output relations for individual functions
#          2022-03-24 M.Marley see RVGazimuth_man
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Library of thruster functions. 
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
# ---------------------------------------------------------------------------

import numpy as np
import kinematics as km

# =============================================================================
# Functions
# =============================================================================
def forceAzi3(force,azi,loc):
    """
    3DOF body-fixed forces from azimuth thruster    
    Input: 
         force: thruster force 
         azi: azimuth angle 
         loc: location of thruster in body-fixed coordinate system 
    Output: 
         F: 3DOF force vector 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley  
    
    eps1 = np.array([1,0,0])
    S = km.Smat(loc)
    R = km.Rotz(azi)   
    F6 =  force*np.concatenate((R@eps1, S@R@eps1)) #6DOF vector
    F = F6[[0,1,5]] #3DOF vector
    
    return F

def forceAzi6(force,azi,loc):
    """
    3DOF body-fixed forces from azimuth thruster    
    Input: 
         force: thruster force 
         azi: azimuth angle 
         loc: location of thruster in body-fixed coordinate system 
    Output: 
         F: 6DOF force vector 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley  
    eps1 = np.array([1,0,0])
    S = km.Smat(loc)
    R = km.Rotz(azi)   
    F =  force*np.concatenate((R@eps1, S@R@eps1)) #6DOF vector
    
    return F

def RVGazimuth_man(u,v,angle,revs):

    """
    Simplified formula for azimuth thruster forces of RVG, calibrated towards
    fmu data for maneuvering speeds (u=5m/s, revs=165), and small angles of
    attack (up to 30deg azimuth angle, small sway speeds).
    Models both thrusters as a single equivalent thruster at midship. Actuator
    loads are modelled as a propeller thrust force combined with drag and lift 
    forces on the foil, then decomposed in body-fixed coordinates. 
    
    Input: 
         u: surge speed (at thruster location) [m/s] 
         v: sway speed (at thruster location) [m/s] 
         angle: azimuth angle [rad] 
         revs: rpm 
    Output: 
         Fx, Fy: surge and sway force [N]
    """
    # Created: 2022-03-24 M.Marley 
    # Tested: 2022-03-24 M.Marley
    # Run verify_RVGazimuth_thruster.py in test folder to verify. 
    # Coefficients below (Cd,Cl,A,Tc) are guesstimates, and roughly calibrated
    # towards fmu data. 
    
    V = np.sqrt(u**2+v**2) #total speed
    
    inflow_angle = np.arctan2(v,u) #inflow angle

    aoa = -inflow_angle+angle #foil angle of attack
  
    Cd = 0.2+np.abs(aoa)*0.4 #drag coefficient
    Cl = aoa #lift coefficient
    A=9 #rudder area
    rho = 1000 #fluid density

    Fdrag = 0.5*rho*A*Cd*V**2 #drag force on foil (parallel to fluid velocity)
    Flift = 0.5*rho*A*Cl*V**2 #lift force on foil (normal to fluid velocity)
    Ffoilx = -Fdrag*np.cos(aoa)+Flift*np.sin(aoa) #force in foil x direction
    Ffoily = Flift*np.cos(aoa)++Fdrag*np.sin(aoa) #force in foil y direction
    
    Tc=1.8 #thruster coefficient (just a guess)   
    Fthrust = Tc*revs**2 #propeller thrust force
    
    #decompose loads in body-fixed surge and sway force
    Fx = Fthrust*np.cos(angle)+Ffoilx*np.cos(angle)-Ffoily*np.sin(angle)
    Fy = Fthrust*np.sin(angle)+Ffoilx*np.sin(angle)+Ffoily*np.cos(angle)
    return Fx, Fy


    
