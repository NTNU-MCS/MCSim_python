# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-03-24
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
#
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Verification script for thrusters.RVGazimuth_man: the simplified RVG azimuth
thruster actuator model of RVG, valid for perturbations about cruising speeds. 
Assumes a propeller load + foil load model, with parameters roughly calibrated
towards "black box" RVG  azimuth fmu.

fmu data provided by motoyasu.kanazawa@ntnu.no.
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
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

# =============================================================================
# Import modules
# =============================================================================
import thrusters as th 

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


varname = ['u','v','r','angle','revs','Fx','Fy','Ftot','forceangle']
varunit = ['m/s','m/s','deg/s','deg','RPM','N','N','N','deg']

df = pd.read_csv('RVG_fmu_thruster_dataset_maneuvering.csv') 

df.columns = varname[0:7]
df['Ftot'] = np.sqrt(df.Fx**2+df.Fy**2)
df['forceangle'] = np.arctan2(df.Fy,df.Fx)*180/np.pi
df = df.drop(df[df.r != 0].index)


def SelectEntry(u,v,angle,revs):
    #finds index in dataframe
    ii=df[df.u==u].index
    ii=ii.intersection(df[df.v==v].index)
    ii=ii.intersection(df[df.revs==revs].index)
    ii=ii.intersection(df[df.angle==angle].index)    
    return ii


# =============================================================================
# Plots versus surge speed
# =============================================================================
us = [4,5,5.5,6,6.5]

Fxv1 = np.zeros(np.size(us))
Fyv1 = np.zeros(np.size(us))
Fxv2 = np.zeros(np.size(us))
Fyv2 = np.zeros(np.size(us))


angle=10
v=0
revss=df.revs.unique()
revss = [150,165,180]

prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']
legstr = []

for i1, revs in enumerate(revss):
    for i2, u in enumerate(us):

        Fxv1[i2] = df.Fx[SelectEntry(u,v,angle,revs)]
        Fyv1[i2] = df.Fy[SelectEntry(u,v,angle,revs)]
        Fxv2[i2], Fyv2[i2] =  th.RVGazimuth_man(u,v,angle*np.pi/180,revs)

    legstr.append('fmu data: revs='+str(revs)+'rpm')
    legstr.append('simplified function: revs='+str(revs)+'rpm')


    plt.figure(1)
    plt.plot(us,Fxv1,'-o',color=colors[i1])
    plt.plot(us,Fxv2,'-*',color=colors[i1])


    plt.figure(2)
    plt.plot(us,Fyv1,'-o',color=colors[i1])
    plt.plot(us,Fyv2,'-*',color=colors[i1])


titstr = 'force versus surge speed, \n v='+str(v)+'m/s, azimuth angle='+str(angle)+'deg'

plt.figure(1)
plt.legend(legstr)
plt.xlabel('u [m/s]')
plt.ylabel('Fx [N]')
plt.title(titstr)

plt.figure(2)
plt.legend(legstr)
plt.xlabel('u [m/s]')
plt.ylabel('Fy [N]')
plt.title(titstr)

# =============================================================================
# Plots versus azimuth angle
# =============================================================================

u=6
v=1
revss=df.revs.unique()
revss = [150,165,180]
angles=df.angle.unique()
Fxv1 = np.zeros(np.size(angles))
Fyv1 = np.zeros(np.size(angles))
Fxv2 = np.zeros(np.size(angles))
Fyv2 = np.zeros(np.size(angles))

for i1, revs in enumerate(revss):
    for i2, angle in enumerate(angles):

        Fxv1[i2] = df.Fx[SelectEntry(u,v,angle,revs)]
        Fyv1[i2] = df.Fy[SelectEntry(u,v,angle,revs)]
        Fxv2[i2], Fyv2[i2] =  th.RVGazimuth_man(u,v,angle*np.pi/180,revs)

    legstr.append('fmu data: revs='+str(revs)+'rpm')
    legstr.append('simplified function: revs='+str(revs)+'rpm')

    plt.figure(3)
    plt.plot(angles,Fxv1,'-o',color=colors[i1])
    plt.plot(angles,Fxv2,'-*',color=colors[i1])


    plt.figure(4)
    plt.plot(angles,Fyv1,'-o',color=colors[i1])
    plt.plot(angles,Fyv2,'-*',color=colors[i1])

titstr = 'force versus azimuth angle, \n v='+str(v)+'m/s, u='+str(u)+'m/s'

plt.figure(3)
plt.legend(legstr)
plt.xlabel('azimuth angle [deg]')
plt.ylabel('Fx [N]')
plt.title(titstr)
plt.tight_layout()
plt.savefig('Fxversusangle.pdf')
plt.figure(4)
plt.legend(legstr)
plt.xlabel('azimuth angle [deg]')
plt.ylabel('Fy [N]')
plt.title(titstr)
plt.tight_layout()
plt.savefig('Fyversusangle.pdf')

# =============================================================================
# Plots versus sway speed
# =============================================================================

u=5
vs=df.v.unique()
revs=180
angles = [0, 10, 20]
Fxv1 = np.zeros(np.size(vs))
Fyv1 = np.zeros(np.size(vs))
Fxv2 = np.zeros(np.size(vs))
Fyv2 = np.zeros(np.size(vs))
legstr = []

for i1, angle in enumerate(angles):
    for i2, v in enumerate(vs):

        Fxv1[i2] = df.Fx[SelectEntry(u,v,angle,revs)]
        Fyv1[i2] = df.Fy[SelectEntry(u,v,angle,revs)]
        Fxv2[i2], Fyv2[i2] =  th.RVGazimuth_man(u,v,angle*np.pi/180,revs)

    legstr.append('fmu data: angle='+str(angle)+'deg')
    legstr.append('simplified function: angle='+str(angle)+'deg')

    plt.figure(5)
    plt.plot(vs,Fxv1,'-o',color=colors[i1])
    plt.plot(vs,Fxv2,'-*',color=colors[i1])


    plt.figure(6)
    plt.plot(vs,Fyv1,'-o',color=colors[i1])
    plt.plot(vs,Fyv2,'-*',color=colors[i1])

titstr = 'force versus sway speed, \n u='+str(u)+'m/s, revs='+str(revs)+'rpm'

plt.figure(5)
plt.legend(legstr)
plt.xlabel('v m/s')
plt.ylabel('Fx [N]')
plt.title(titstr)

plt.figure(6)
plt.legend(legstr)
plt.xlabel('v m/s')
plt.ylabel('Fy [N]')
plt.title(titstr)
plt.tight_layout()
plt.savefig('Fyversusswayspeed.pdf')








