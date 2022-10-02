# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: 2022-10-02 R. Skjetne Added draw_ship_2D(eta,parP)
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
    """Plots timeseries in individual plots. 
       Input 
          t: time vector of length m   
          x: n times m matrix of time series
          parP: dict containing plotting parameters:
             figsize = [figx figy] figure size
             titles: list of titles
             units: list of units
             scale: for converting, e.g. from rad to deg
    ----------------------------------------------------------------------------
    Created by: M. Marley on 2022-02-04
    Revised: N/A
    ---------------------------------------------------------------------------
    """         
    scale = parP['scale']
 
    for i1 in range(np.shape(x)[0]):
        plt.figure(i1+parP['fig0'])#,figsize=parP['figsize'], clear='True')
        plt.plot(t,x[i1,:]*scale[i1])
        plt.title(parP['titles'][i1])
        plt.ylabel(parP['units'][i1])
        plt.xlabel('Time [s]')

    return 


def draw_ship_2D(eta,parP): 
    """ Draws the 2D extent of a ship at location (x,y) and heading (psi).  
        Input 
            eta: 3xN matrix of positions x,y, and headings psi of N vessels.
            parP: dict containing plotting parameters:
                ax: Matplotlib axis handle to plot on 
                Loa: 1xN array of Length overall of ship
                type: 'ship' or 'arrow'
        An error message will be issued if dimensions of inputs are wrong.
    ----------------------------------------------------------------------------
    Created by: R. Skjetne on 2022-10-02
    Revised: N/A
    ---------------------------------------------------------------------------
    """         
    [M1,N1] = eta.shape
    N2 = len(parP['Loa'])

    if ( N1 != N2 ) or M1 != 3:
        raise ValueError('Wrong dimensions of eta or Loa inputs.')
    else:
        c1 = np.arcsin(1/8) 
        c2 = 3*np.pi/4

        ax1 = parP['ax']    # Axis to plot the ship in.

        for ii in range(N1):
            r1 = 0.5*parP['Loa'][ii]
            R = np.array([[np.cos(eta[2,ii]),-np.sin(eta[2,ii])],[np.sin(eta[2,ii]),np.cos(eta[2,ii])]])

            if parP['type'] == 'arrow':
                arrow0 = np.array([eta[0,ii]+[r1, r1*np.cos(c2), 0, r1*np.cos(-c2), r1], 
                            eta[1,ii]+[0, r1*np.sin(c2), 0, r1*np.sin(-c2), 0]])
                boat = np.dot(R,arrow0)
            else:
                boat0 = np.array([eta[0,ii]+[r1, r1/2, -3*r1/4, r1*np.cos(np.pi-c1), r1*np.cos(np.pi+c1), -3*r1/4, r1/2, r1], 
                            eta[1,ii]+[0, r1/4, r1/4, r1*np.sin(np.pi-c1), r1*np.sin(np.pi+c1), -r1/4, -r1/4, 0]])
                boat = np.dot(R,boat0)
    
            # Setting up plots and animation
            ax1.plot(boat[0,:],boat[1,:])
        

