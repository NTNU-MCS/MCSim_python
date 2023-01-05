
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


colnames = ['channel','subchannel','date_time','timestamp','value','unit','timestring']



for i in np.arange(47,142):
    
    if i<100:
        filename = 'log0'+str(i)   
    else:
        filename = 'log'+str(i)           
#    print(filename)
    data = pd.read_csv(filename,header=None,sep=';')
    Nrows = data.shape[0]

#   df = pd.DataFrame(columns=colnames)
    channels = []
    subchannels = []
    date_times = []
    timestamps = []
    values = []
    units = []
    timestrings = []
    print(filename+': Reading file')    
    for index, rows in data.iterrows():
#        print(filename+': Reading line ' +str(index)+' of '+str(Nrows))
        string = rows[0]
        tmp = string.split('/')
        notused = tmp[0]
        channel = tmp[1]
        string = tmp[2]
        tmp = string.split('"')
        subchannel = tmp[0]
        string = tmp[4]
        tmp = string.split(',')
        
        if tmp[0]=='None':
            value = np.nan
            unit = np.nan
            date_time=np.nan
            timestamp=np.nan
        elif channel == 'SeapathGPSGga':
            degs = float(tmp[0][0:2])
            mins = float(tmp[0][2:])
            value = degs+mins/60
        else:
            value = float(tmp[0])
            timestring= tmp[1]
            date_time = datetime.strptime(timestring[0:-1],"%Y-%m-%dT%H:%M:%S.%f")
            timestamp = date_time.timestamp()    
            unit = tmp[2]
#        df.loc[len(df)] = [channel,subchannel,date_time,timestamp,value,unit,timestring]

        channels.append(channel)
        subchannels.append(subchannel)
        date_times.append(date_time)
        timestamps.append(timestamp)
        values.append(value)
        units.append(unit)
        timestrings.append(timestring)      
    print(filename+': creating dataframe')  
    df = pd.DataFrame.from_dict({'channel': channels,'subchannel':subchannels,'date_time':date_times,
                        'timestamp':timestamps,'value': values, 'unit': units,'timestring': timestrings})

#        df = df.append({'channel': channel,'subchannel':subchannel,'date_time':date_time,
#            'timestamp':timestamp,'value': value, 'unit': unit,'timestring': timestring},ignore_index=True)
    print(filename+': Saving file')        
    df.to_csv('df_'+filename+'.csv',index=False)
    
#    lat = df.loc[df.subchannel=='Latitude']
#    long = df.loc[df.subchannel=='Longitude']
#    plt.plot(lat.value,long.value)
    
#roll = df.loc[df.subchannel=='Roll']

#plt.plot(roll.timestamp-roll.timestamp.iloc[0],roll.value)


#lat = df.loc[df.subchannel=='Latitude']
#long = df.loc[df.subchannel=='Longitude']




