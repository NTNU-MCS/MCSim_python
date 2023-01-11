
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
from scipy import interpolate
from datetime import datetime, timedelta

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
    val1[np.isnan(val1)] = np.mean(val1[~np.isnan(val1)])
    
    
    f2 = interpolate.interp1d(ts2, df2.value.values,fill_value='extrapolate')
    val2 = f2(ts)
    val2[np.isnan(val2)] = np.mean(val2[~np.isnan(val2)])
    return ts,val1,val2

def interpolate_sampling2(df,tvec):
    
    tsamp = df.timestamp.values

    f = interpolate.interp1d(tsamp, df.value.values,fill_value='extrapolate')
    val = f(tvec)

    return val

def testdata(df,tstart,tend):

    df = df[df.timestamp.values>=tstart]
    df = df[df.timestamp.values<=tend]

    return df

def get_timeseries(data,test):
    
    tstart = test['tstart']
    tend = test['tend']
    
    long = testdata(data['Longitude'],tstart,tend)
    
    t0 = long.timestamp.values[0]
    
    lat = testdata(data['Latitude'],tstart,tend)
    
    dt_start = datetime.strptime(long.date_time.values[0],"%Y-%m-%d %H:%M:%S.%f")\
                +timedelta(hours=2)
    dt_end = datetime.strptime(long.date_time.values[-1],"%Y-%m-%d %H:%M:%S.%f")\
                +timedelta(hours=2)
    dt_start = dt_start.strftime('%H:%M')
    dt_end = dt_end.strftime('%H:%M')
    
    ts,long,lat = interpolate_sampling(long,lat)

    lat0 =  63.47 #63.438631 
    long0 = 10.54 #10.409383 
    tmp = metres_per_deg(lat0)
    meters_per_latdeg = tmp[0]
    meters_per_longdeg  = tmp[1]

    x = np.array((long-long0)*meters_per_longdeg)  
    y = np.array((lat-lat0)*meters_per_latdeg)
    
 #   x = x-x[0]
 #   y = y-y[0]
    
    res = {}
    

    for i,channel in enumerate(test['channels']):
        df = testdata(data[channel],tstart,tend)
        res[channel] = df
        plt.figure(test['fignum'][i])
        
        plt.plot(df.timestamp-t0,df.value*test['scale'][i])
        plt.title(channel+': '+test['description']+', '+dt_start+'-'+dt_end)        
        plt.xlabel('Time [s]')
        plt.ylabel(df.unit.values[0])
         
    return x,y,res, t0

# # =============================================================================
# # Plot tests
# # =============================================================================

    
# plt.close('all')

# data = pickle.load( open("maneuvering_data.pkl", "rb" )) 

# #Origin North of Lade:
# lat0 =  63.47 #63.438631 
# long0 = 10.54 #10.409383 
# tmp = metres_per_deg(lat0)
# meters_per_latdeg = tmp[0]
# meters_per_longdeg  = tmp[1]

# channels = ['SpeedKnots','Heading','YawRate','Roll','Pitch','port_AzimuthOrder',
#             'stbd_RPMOrder','LonGroundSpeed','TransGroundSpeed','CourseTrue','Wind_Speed']


# test1 = {'tstart':1150,'tend':1530,'name':'speed_test','description':'Speed Test 80%','channels':channels}

# test2 = {'tstart':1400,'tend':1800,'name':'turning_circle','description':'Turning circle 30deg','channels':channels}

# test3 = {'tstart':1850,'tend':2250,'name':'speed_test','description':'Speed Test 80%','channels':channels}

# test4 = {'tstart':2150,'tend':2500,'name':'turning_circle','description':'Turning circle 30deg','channels':channels}

# test5 = {'tstart':2600,'tend':2850,'name':'zig_zag','description':'Zig zag 10/10','channels':channels}

# test6 = {'tstart':2900,'tend':3300,'name':'zig_zag','description':'Zig zag 10/20','channels':channels}

# test7 = {'tstart':3300,'tend':3700,'name':'turning_circle','description':'Turning circle 10deg','channels':channels}

# test8 = {'tstart':3600,'tend':4000,'name':'zig_zag','description':'Zig zag 5/10','channels':channels}

# test9 = {'tstart':3950,'tend':4290,'name':'turning_circle','description':'Turning circle 10','channels':channels}

# test10 = {'tstart':4300,'tend':4750,'name':'zig_zag','description':'Zig zag 10/20','channels':channels}

# test11 = {'tstart':4750,'tend':5200,'name':'zig_zag','description':'Zig zag 5/10','channels':channels}

# test12 = {'tstart':5200,'tend':5400,'name':'zig_zag','description':'Zig zag 20/20','channels':channels}

# test13 = {'tstart':5400,'tend':5610,'name':'zig_zag','description':'Zig zag 10/10','channels':channels}

# test14 = {'tstart':5650,'tend':5950,'name':'turning_circle','description':'Turning circle 30deg','channels':channels}

# test15 = {'tstart':6000,'tend':6350,'name':'turning_circle','description':'Turning circle 30deg','channels':channels}

# test16 = {'tstart':6400,'tend':7000,'name':'speed_test','description':'Speed test','channels':channels}

# test17 = {'tstart':6900,'tend':7250,'name':'man_over_board','description':'Man over board','channels':channels}

# test18 = {'tstart':7500,'tend':8000,'name':'speed_test','description':'Speed test','channels':channels}

# test19 = {'tstart':8100,'tend':8600,'name':'turning circle','description':'Turning circle 30','channels':channels}

# test20 = {'tstart':8550,'tend':8850,'name':'turning circle','description':'Turning circle 20','channels':channels}

# test21 = {'tstart':8850,'tend':9250,'name':'speed_test','description':'Speed test','channels':channels}

# test22 = {'tstart':9250,'tend':9500,'name':'turning_circle','description':'Turning circle 20','channels':channels}

# test23 = {'tstart':9550,'tend':9900,'name':'speed_test','description':'Speed test','channels':channels}


#plot_test(data,test6)
# plot_test(data,test1)
# plot_test(data,test3)
# plot_test(data,test16)
# plot_test(data,test18)
# plot_test(data,test21)
# plot_test(data,test23)

# plot_test(data,test7)









