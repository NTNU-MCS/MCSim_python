# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-12-08
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley Checked that results make sense
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Extract sea trials results of RVG and interpolate to common sampling time. 
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

par_path = str(Path(os.path.dirname(__file__)).parents[2])  
library_path = par_path + '\\lib'
sys.path.append(library_path)

# =============================================================================
# Load general modules
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import pickle


# =============================================================================
# Load model module
# =============================================================================

import ManTestData as MTD
import pandas as pd

# =============================================================================
# compare with test data
# =============================================================================
plt.close('all')

exp_data = pickle.load( open("maneuvering_data.pkl", "rb" ) )

revmax = exp_data['stbd_RPMOrder']['value'].values*0+205/2.185
exp_data['stbd_RPMOrder']['value'] = np.min([exp_data['stbd_RPMOrder']['value'].values,revmax],0)

channels = ['Roll','Heading','LonGroundSpeed','TransGroundSpeed','YawRate','port_AzimuthOrder','stbd_RPMOrder']
channelnames = ['phi','psi','u','v','r','Azi','RPM']


scale = np.array([1,1,1./1.9834,1./1.9834,1,1,2.185])

tstart = 1050
#tstart = 1050+1900
tend = 9900
tend = 9900
#tend = tstart+1350
dt = 0.5

tvec = np.linspace(tstart,tend,int((tend-tstart)/dt)+1)


long = MTD.interpolate_sampling2(exp_data['Longitude'],tvec)
lat = MTD.interpolate_sampling2(exp_data['Latitude'],tvec)

lat0 =  63.47 #63.438631 
long0 = 10.54 #10.409383 
tmp = MTD.metres_per_deg(lat0)
meters_per_latdeg = tmp[0]
meters_per_longdeg  = tmp[1]

East = np.array((long-long0)*meters_per_longdeg)  
North = np.array((lat-lat0)*meters_per_latdeg)
    
plt.figure(1)
plt.plot(East,North)
plt.axis('equal')

data_dict = {'tvec':tvec,'N':North,'E':East}




for i,channel in enumerate(channels):
    val = MTD.interpolate_sampling2(exp_data[channel],tvec)
    data_dict[channelnames[i]] = val*scale[i]
    plt.figure()
    plt.plot(tvec,val*scale[i])
    plt.title(channel)

mantests = pd.DataFrame.from_dict(data_dict)

mantests.tvec.values[:] = mantests.tvec.values-mantests.tvec.values[0]

for (columnName, columnData) in mantests.iteritems():
    plt.figure()
    plt.plot(mantests.tvec.values,columnData)


mantests.to_csv('RVG_ManeuveringTests.csv')





# testall = {'tstart':1050,'tend':9900,'name':'speed_test','description':'Speed test','channels':channels}

# fignum = [3, 12, 6,7,9,4,10,11,5]

# #testall['tend'] = 3000
# test = testall
# test['scale'] = scale
# test['fignum'] = fignum



# long,lat,res, t0 = MTD.get_timeseries(exp_data,test)

# plt.figure(1)
# plt.plot(long,lat)
# plt.axis('equal')






