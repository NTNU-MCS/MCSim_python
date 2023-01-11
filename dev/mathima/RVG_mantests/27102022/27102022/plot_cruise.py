
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-11-01
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley Checked that results make sense
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
# """
# Reads RVG data stream into pandas dataframes and saves in more readable csv
# """



from datetime import datetime
import time


# =============================================================================
# Load general modules
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import interpolate

def metres_per_deg(lat):
    
    lat = lat*np.pi/180
    
    meters_per_latdeg = 111132.92-559.82*np.cos(2*lat) +1.175*np.cos(4*lat) -0.0023*np.cos(6*lat)
    meters_per_longdeg =111412.84*np.cos(lat) -93.5*np.cos(3*lat) +0.118*np.cos(5*lat)
    return meters_per_latdeg,meters_per_longdeg


lat0 =  63.438631 #63.50238 
long0 = 10.409383 #10.540440
tmp = metres_per_deg(lat0)

meters_per_latdeg = tmp[0]
meters_per_longdeg  = tmp[1]
 
for i in np.arange(47,142):
    
    if i<100:
        filename = 'df_log0'+str(i)+'.csv'   
    else:
        filename = 'df_log'+str(i)+'.csv'           

    df = pd.read_csv(filename)
    
    lat = df.loc[df.subchannel=='Latitude']
    lat.reset_index(drop=True,inplace=True)
    long = df.loc[df.subchannel=='Longitude']
    long.reset_index(drop=True,inplace=True)
    
    
    Nlat = len(lat)
    Nlong = len(long)
    
    if Nlat==Nlong:
        if (lat.timestamp.values == long.timestamp.values).all():
            y =np.array((lat.value-lat0)*meters_per_latdeg)
            x = np.array((long.value-long0)*meters_per_longdeg)  
    elif Nlat>Nlong:      
       y =np.array((lat.value-lat0)*meters_per_latdeg)
       f = interpolate.interp1d(long.timestamp.values, long.value.values,fill_value='extrapolate')
       longnew = f(lat.timestamp.values)
       x = np.array((longnew-long0)*meters_per_longdeg)
    elif Nlong>Nlat:
       x = np.array((long.value-long0)*meters_per_longdeg)  
       f = interpolate.interp1d(lat.timestamp.values, lat.value.values,fill_value='extrapolate')
       latnew = f(long.timestamp.values)
       y =np.array((latnew-lat0)*meters_per_latdeg)
    
    plt.figure(1)   
    plt.plot(x,y,'b')
    plt.axis('equal')
         
        

    
#roll = df.loc[df.subchannel=='Roll']

#plt.plot(roll.timestamp-roll.timestamp.iloc[0],roll.value)


#lat = df.loc[df.subchannel=='Latitude']
#long = df.loc[df.subchannel=='Longitude']



