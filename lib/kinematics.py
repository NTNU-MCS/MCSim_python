# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Version = '1.0'
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley input/output relations for individual functions
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Library of kinematics functions 
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
# ---------------------------------------------------------------------------

import numpy as np

# =============================================================================
# Functions
# =============================================================================
def rad2pipi(angle): 
    """Maps angle to [-pi,pi] 
       Input 
           angle: in radians 
       Output 
           angle: in [-pi,pi] 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley  
    angle = np.arctan2(np.sin(angle),np.cos(angle))
    return angle 

def Rotx(angle):
    """Rotation matrix about x-axis 
       Input 
           angle: in radians  
       Output 
           Rx: 3x3 matrix 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley 
    Rx = np.array([[0, 0, 1],
          [0, np.cos(angle), -np.sin(angle)],
          [0, np.sin(angle), np.cos(angle)]])
    return Rx

def Roty(angle):
    """Rotation matrix about y-axis 
       Input 
           angle: in radians 
       Output 
           Ry: 3x3 matrix 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley 
    Ry = np.array([[np.cos(angle),0, np.sin(angle)],
          [0,1,0],
          [-np.sin(angle), 0, np.cos(angle)]])
    return Ry

def Rotz(angle):
    """Rotation matrix about z-axis 
       Input 
           angle: in radians  
       Output 
           Rz: 3x3 matrix 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley   
    Rz = np.array([[np.cos(angle), -np.sin(angle), 0],
          [np.sin(angle), np.cos(angle), 0],
          [0, 0, 1]])
    return Rz

def Smat(x):
    """Skew-symmetric cross-product operator matrix 
       Input 
           x: 3x1 vector 
       Output 
           S: 3x3 matrix 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley 
    S = np.array([[0, -x[2], x[1]],
          [x[2], 0, -x[0]],
          [-x[1], x[0], 0]])
    return S

def dot_eta3(psi,nu):
    """Derivative of 3DOF pose vector p=[x,y,psi] 
       Input 
           psi: heading 
           nu: 3DOF body-fixed velocities 
       Output 
           d_eta3: 3x1 vector 
    """
    # Created: 2022-02-04 M.Marley 
    # Tested: 2022-02-04 M.Marley  
    d_eta3 = Rotz(psi)@nu
    return d_eta3
