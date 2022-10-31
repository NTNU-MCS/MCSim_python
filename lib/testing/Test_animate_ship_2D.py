# -*- coding: utf-8 -*-
"""
Test script of the animate_ship_2D() function.
Created on 2022-10-08
@author: Roger Skjetne
"""

# =============================================================================
# Set path
# =============================================================================
import os
from pathlib import Path
import sys

library_path = str(Path(os.path.dirname(__file__)).parents[0])
sys.path.append(library_path)

# =============================================================================
# Load general modules
# =============================================================================import numpy as np
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as anim
# from draw_ship_lib import *
from visualization import *

# Ship motion parameters
Loa1 = 2.5  # Ship 1 length overall
Loa2 = 1.0  # Ship 2 length overall
x20 = 0.0   # Ship 2 initial x-postion 
y20 = -20.0 # Ship 2 initial y-postion 
v2 = 1.5    # Speed of Ship 2
Loa3 = 1.5  # Ship 3 length overall
x30 = -20.0 # Ship 3 initial x-postion 
y30 =  0.0  # Ship 3 initial y-postion 
v3 = 2.0    # Speed of Ship 3

# Simualation data
N = 101  # Number of samples
t = np.linspace(0, 20, N)

# Position Arrays
x1 = 10 * np.cos(2*np.pi/10 * t)  # x' = -10*2*np.pi/10*np.sin(2*np.pi/10 * t) = -2*np.pi/10 * y
y1 = 10 * np.sin(2*np.pi/10 * t)  # y' =  10*2*np.pi/10*np.cos(2*np.pi/10 * t) =  2*np.pi/10 * x
psi1 = np.arctan2(x1, -y1)
eta1 = np.array([x1, y1, psi1])  # Combining our position/heading coordinates

x2 = x20 + 0.0 * t
y2 = y20 + v2 * t
psi2 = np.arctan2(v2*np.ones_like(t), 0.0*t)
eta2 = np.array([x2, y2, psi2])  # Combining our position/heading coordinates

x3 = x30 + v3 * t
y3 = y30 + 0.0 * t
psi3 = np.arctan2(0.0*t, v3*np.ones_like(t))
eta3 = np.array([x3, y3, psi3])  # Combining our position/heading coordinates

# Stacking the eta vector for our 2 vessels
eta = np.vstack((eta1, eta2, eta3))

fig1, ax1 = plt.subplots()  # a figure with a single axes
parP = {'fig': fig1, 'ax': ax1, 'Loa': [Loa1, Loa2, Loa3], 'type': ['ship', 'arrow', 'ship'], 
        'setlim': True, 'grid': False, 'frame_delay': 50, 'color': ['blue','red','green']}
ship_ani = animate_ship_2D(t,eta,parP)

plt.show() 