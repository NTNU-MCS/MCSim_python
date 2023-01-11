
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
# Gets RVG cruise data from 27th october 2022 
# """
# =============================================================================
# Load general modules
# =============================================================================
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pickle
from scipy import interpolate

def metres_per_deg(lat):
    
    lat = lat*np.pi/180
    
    meters_per_latdeg = 111132.92-559.82*np.cos(2*lat) +1.175*np.cos(4*lat) -0.0023*np.cos(6*lat)
    meters_per_longdeg =111412.84*np.cos(lat) -93.5*np.cos(3*lat) +0.118*np.cos(5*lat)
    return meters_per_latdeg,meters_per_longdeg

def interpolate_sampling(df1,df2):
    
    ts1 = df1.timestamp.values
    ts2 = df2.timestamp.values
    ts = np.append(ts1,ts2)
    ts = np.unique(ts)
    ts = np.sort(ts)
    
    
    f1 = interpolate.interp1d(ts1, df1.value.values,fill_value='extrapolate')
    val1 = f1(ts)
    f2 = interpolate.interp1d(ts2, df2.value.values,fill_value='extrapolate')
    val2 = f2(ts)
    return [val1,val2]

def plot_test(data,test):
    
    tstart = test['tstart']
    tend = test['tend']
    
    
    
    [long,lat] = interpolate_sampling(data['Longitude'],data['Latitude'])

    x = np.array((long-long0)*meters_per_longdeg)  
    y = np.array((lat-lat0)*meters_per_latdeg)

    plt.figure()   
    plt.plot(x,y,'-o')
    plt.axis('equal')
    plt.title('Trajectory')
    plt.xlabel('East [m]')
    plt.ylabel('North [m]')

    return 

    
plt.close('all')

subchannels_to_keep =['Latitude', 'Longitude', 'LonGroundSpeed', 'TransGroundSpeed', 
       'CourseTrue','SpeedKmHr', 'SpeedKnots', 'Heading', 'Heave', 'Pitch', 
       'Roll','PitchRate', 'RollRate', 'VertVel', 'YawRate', 
       'Wind_Direction', 'Wind_Speed',
       'AzimuthOrder', 'RPMOrder']

#Origin North of Lade:
lat0 =  63.47 #63.438631 
long0 = 10.54 #10.409383 
tmp = metres_per_deg(lat0)
meters_per_latdeg = tmp[0]
meters_per_longdeg  = tmp[1]
 
data = {}

for i in np.arange(84,95): #whole cruise: np.arange(83,108)
    
    if i<100:
        filename = 'df_log0'+str(i)+'.csv'   
    else:
        filename = 'df_log'+str(i)+'.csv'           

    df = pd.read_csv(filename)
    

    
    
    for subchan in subchannels_to_keep:
        tmp = df.loc[df.subchannel==subchan]
        if i==84:
            data[subchan] = tmp  
        else:
            data[subchan] = data[subchan].append(tmp)



t0 = data['Latitude'].timestamp.values[0]

for subchan in subchannels_to_keep:
    data[subchan].timestamp = data[subchan].timestamp.values-t0 

thruster_channels = ['AzimuthOrder', 'RPMOrder']

for subchan in thruster_channels:
    newsubchan = 'port_'+subchan
    data[newsubchan] = data[subchan].loc[data[subchan].channel=='hcx_port_mp']
    data[newsubchan].reset_index(drop=True,inplace=True)
    plt.figure()
    plt.plot(data[newsubchan].timestamp,data[newsubchan].value,'-o')
    plt.title(subchan)
    newsubchan = 'stbd_'+subchan
    data[newsubchan] = data[subchan].loc[data[subchan].channel=='hcx_stbd_mp']
    data[newsubchan].reset_index(drop=True,inplace=True)
    plt.plot(data[newsubchan].timestamp,data[newsubchan].value,'-o')
    plt.title(subchan)

other_channels =['Latitude', 'Longitude', 'LonGroundSpeed', 'TransGroundSpeed', 
       'CourseTrue','SpeedKmHr', 'SpeedKnots', 'Heading', 'Heave', 'Pitch', 
       'Roll','PitchRate', 'RollRate', 'VertVel', 'YawRate', 
       'Wind_Direction', 'Wind_Speed']

for subchan in other_channels:
    data[subchan].reset_index(drop=True,inplace=True)
    plt.figure()
    plt.plot(data[subchan].timestamp,data[subchan].value,'-o')
    plt.title(subchan)
    
f = open("maneuvering_data.pkl","wb")
pickle.dump(data,f)


# [long,lat] = interpolate_sampling(data['Longitude'],data['Latitude'])

# x = np.array((long-long0)*meters_per_longdeg)  
# y = np.array((lat-lat0)*meters_per_latdeg)

# plt.figure()   
# plt.plot(x,y,'-o')
# plt.axis('equal')
         

# test = {'tstart':1020,'tend':1530,'name':'speed_test','description':'none'}

# plot_test(data,test)




