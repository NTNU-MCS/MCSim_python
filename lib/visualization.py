# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: 2022-10-19 R. Skjetne Added draw_ship_2D() and animate_ship_2D()
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

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as anim



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
        
        if 'figsize' in parP:
            plt.figure(i1+parP['fig0'],figsize=parP['figsize'], clear='True')
        else:
            plt.figure(i1+parP['fig0'])#
        plt.plot(t,x[i1,:]*scale[i1])
        plt.title(parP['titles'][i1])
        plt.ylabel(parP['units'][i1])
        plt.xlabel('Time [s]')
        plt.tight_layout()
    return 



def draw_ship_2D(eta,parP): 
    """ Draws the 2D extent of a ship at location (x,y) and heading (psi).  
        Inputs: 
            eta: 3xM matrix of positions x,y, and headings psi of M ships.
            parP: dict containing plotting parameters:
                ax: Matplotlib axis handle to plot on 
                Loa: 1xM array of Length overall of ship
                type: 'ship' or 'arrow'
                color: color of ship contour (color codes by matplotlib.plot)
        An error message will be issued if dimensions of inputs are wrong.
    ----------------------------------------------------------------------------
    Created by: R. Skjetne on 2022-10-19
    Revised: N/A
    ---------------------------------------------------------------------------
    """         
    [K1,M1] = eta.shape
    M2 = len(parP['Loa'])

    if ( M1 != M2 ) or K1 != 3:
        raise ValueError('Wrong dimensions of eta or Loa inputs.')
    else:
        c1 = np.arcsin(1/8) 
        c2 = 3*np.pi/4

        ax1 = parP['ax']    # Axis to plot the ship in.

        for ii in range(M1):
            r1 = 0.5*parP['Loa'][ii]
            R = np.array([[np.cos(eta[2,ii]),-np.sin(eta[2,ii])],[np.sin(eta[2,ii]),np.cos(eta[2,ii])]])

            if parP['type'][ii] == 'arrow':
                arrow0 = np.array([[r1, r1*np.cos(c2), 0, r1*np.cos(-c2), r1], 
                                   [0, r1*np.sin(c2), 0, r1*np.sin(-c2), 0]])
                boat = np.dot(R,arrow0)
            else:
                boat0 = np.array([[r1, r1/2, -3*r1/4, r1*np.cos(np.pi-c1), r1*np.cos(np.pi+c1), -3*r1/4, r1/2, r1], 
                                  [0, r1/4, r1/4, r1*np.sin(np.pi-c1), r1*np.sin(np.pi+c1), -r1/4, -r1/4, 0]])
                boat = np.dot(R,boat0)
    
            # Setting up plots and animation
            ax1.plot(eta[1,ii]+boat[1,:], eta[0,ii]+boat[0,:], c=parP['color'])
        


def animate_ship_2D(t,eta,parP): 
    """ Animates the 2D motion of M ships along trajectories (x,y) with heading (psi).  
        Inputs: 
            t: 1xN array of N increasing time instances. 
            eta: 3MxN matrix of M positions x,y, and headings psi for M vessels.
            parP: dict containing plotting parameters:
                fig: Matplotlib figure handle to plot in 
                ax: Matplotlib axis handle to plot on 
                Loa: 1xM array of Length overall of vessel
                type: List of 'ship' or 'arrow' entries for M vessels.
                color: List of colors of ship contours (color codes by matplotlib.plot)
                setlim: True = set axis limits; False = don't
                grid: True = grid on; False = grid off
                frame_delay: Delay between frames in milliseconds
        An error message will be issued if dimensions of inputs are wrong.
        E.g.: 
            fig1, ax1 = plt.subplots()  # a figure with a single axes
            parP = {'fig': fig1, 'ax': ax1, 'Loa': [12, 8], 'type': ['ship', 'arrow'], 
                    'setlim': True, 'grid': False, 'frame_delay': 50}
            ship_ani = animate_ship_2D(t,eta,parP)
    ----------------------------------------------------------------------------
    Created by: R. Skjetne on 2022-10-19
    Revised: N/A
    ---------------------------------------------------------------------------
    """     
    [M0,N0] = eta.shape
    N = len(t)  
    if ( M0 % 3 ) != 0 or (M0 < 3) or (N0 != N):
        raise ValueError('Wrong dimensions of eta or t inputs.')
    else:
        M = int(M0/3)
        ax0 = parP['ax']    # Axis to plot the ship in.
        fig0 = parP['fig']    # Fig to animate the ship in.
        parP0 = parP.copy()

        def animate_func(k):
            minNorth = float("inf")
            maxNorth = -float("inf")
            minEast = float("inf")
            maxEast = -float("inf")
            ax0.clear()  # Clears the figure to update the line, point, title, and axes

            # Updating trajectory of ship
            for ii in range(M):
                ax0.plot(eta[3*ii+1, :k+1], eta[3*ii, :k+1], c='blue')
                # Adding each ship contour 
                parP0['Loa'] = [parP['Loa'][ii]]
                parP0['type'] = [parP['type'][ii]]
                parP0['color'] = parP['color'][ii]
                draw_ship_2D(eta[3*ii:3*ii+3, k:k+1],parP0)
                # Adding initial position
                ax0.plot(eta[3*ii+1, 0], eta[3*ii, 0], c='black', marker='o')
                # Finding limits of axes
                minNorth = min(minNorth, np.min(eta[3*ii, :]))
                maxNorth = max(maxNorth, np.max(eta[3*ii, :]))
                minEast = min(minEast, np.min(eta[3*ii+1, :]))
                maxEast = max(maxEast, np.max(eta[3*ii+1, :]))

            # Setting limits of axes
            if parP['setlim']:
                Loa_max = np.max(parP['Loa'])
                ax0.set_xlim([minEast-Loa_max, maxEast+Loa_max])
                ax0.set_ylim([minNorth-Loa_max, maxNorth+Loa_max])

            # Adding figure labels
            ax0.set_title('Ship trajectories \nTime = ' + str(np.round(t[k],    
                        decimals=2)) + ' sec.')
            ax0.set_xlabel('East (m)')
            ax0.set_ylabel('North (m)')
            if parP['grid']:
                ax0.grid()
            ax0.set_aspect('equal', 'box')

        # Plotting the animation
        ship_ani = anim.FuncAnimation(fig0, animate_func, interval=parP['frame_delay'], frames=N)

        return ship_ani
  