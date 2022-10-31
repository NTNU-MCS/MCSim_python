# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley Checked that results make sense
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Initialize example simulation of a pseudo-6DOF RVG maneuvering model moving in
uniform and steady currents, using a linear+quadratic damping formulation. The
model captures the 3DOF horizontal plane motion together with roll, while
heave and pitch motion is effectively suppressed by large damping and
excluding certain load effects.

Vessel parameters are loaded from pickle files, generated in
GenerateRVGManeuveringModelData.py

Example simulation uses a PD-controller to maintain constant heading 
before executing a hard turn. Propeller revs are ramped up and down.  

Control inputs are the commanded thruster states: azimuth angle and propeller
revs of an equivalent thruster located at centreline. 
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: used together with Module_RVG_3DOF
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
model_path = par_path+'\\models\\RVG_maneuvering'
sys.path.append(model_path)

# =============================================================================
# Load general modules
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import thrusters as th
cmap = plt.get_cmap("tab10")
plt.close('all')

azivec = np.linspace(-np.pi/6,np.pi/6,100)
vvec = np.linspace(0,3,100)

Fxr = azivec*0
Fyr = azivec*0
Fxs = azivec*0
Fys = azivec*0

revs=200
uvec=[3,4,5,6]

for i2, u in enumerate(uvec):

    for i1, azi in enumerate(azivec):
        v=1
        Fxr[i1],Fyr[i1] = th.RVGazimuth_man(u,v,azi,revs)
        Fxs[i1],Fys[i1] = th.RVGazimuth_man_simpl(u,v,azi,revs)    

        
    plt.figure(1)    
    plt.plot(azivec*180/np.pi,Fxr,'--',color=cmap(i2))    
    plt.plot(azivec*180/np.pi,Fxs,':',color=cmap(i2))
    
    plt.figure(2)    
    plt.plot(azivec*180/np.pi,Fyr,'--',color=cmap(i2))    
    plt.plot(azivec*180/np.pi,Fys,':',color=cmap(i2))
        
    for i1, v in enumerate(vvec):
        azi=np.pi/8
        Fxr[i1],Fyr[i1] = th.RVGazimuth_man(u,v,azi,revs)
        Fxs[i1],Fys[i1] = th.RVGazimuth_man_simpl(u,v,azi,revs)    

        
    plt.figure(3)    
    plt.plot(vvec,Fxr,'--',color=cmap(i2))    
    plt.plot(vvec,Fxs,':',color=cmap(i2))
    
    plt.figure(4)    
    plt.plot(vvec,Fyr,'--',color=cmap(i2))    
    plt.plot(vvec,Fys,':',color=cmap(i2))