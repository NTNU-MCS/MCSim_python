# -*- coding: utf-8 -*-
"""
Test script of the draw_ship_2D() function.
Created on 2022-10-07
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
# from draw_ship_lib import draw_ship_2D
from visualization import draw_ship_2D

# Ship motion parameters
psi = np.array([-45, 22.5])*np.pi/180 # Headings of ship
eta = np.array([[1,2],[2,3],psi])

fig1, ax1 = plt.subplots()  # a figure with a single Axes
parP = {'ax': ax1, 'Loa': [1.5, 0.9], 'type': ['ship', 'arrow'], 'color': 'blue'}
draw_ship_2D(eta,parP)

ax1.set_aspect('equal', 'box')
plt.grid()
plt.show()
