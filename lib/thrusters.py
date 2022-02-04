# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 input/output relations for individual functions
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Library of thruster functions
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
# ---------------------------------------------------------------------------

import numpy as np
import kinematics as km


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
    
