# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  
# 
# Copyright (C) 2022: NTNU, Department of marine technology, Trondheim, Norway
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Library of plotting functions
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
# ---------------------------------------------------------------------------


import matplotlib.pyplot as plt
import numpy as np



# =============================================================================
# Functions
# =============================================================================


def plot_timeseries(t,x,parP): 
# ----------------------------------------------------------------------------
# Created by: M. Marley on 2022-02-04
# Revised: N/A
# ---------------------------------------------------------------------------
    """Plots timeseries in individual plots. 
       Input 
          t: time vector of length m   
          x: n times m matrix of time series
          parP: dict containing plotting parameters:
             figsize = [figx figy] figure size
             titles: list of titles
             units: list of units
             scale: for converting, e.g. from rad to deg
    """         
    scale = parP['scale']
    # Created: 2022-02-04 M.Marley 
    for i1 in range(np.shape(x)[0]):
        plt.figure(i1+parP['fig0'])#,figsize=parP['figsize'], clear='True')
        plt.plot(t,x[i1,:]*scale[i1])
        plt.title(parP['titles'][i1])
        plt.ylabel(parP['units'][i1])
        plt.xlabel('Time [s]')

    return 

