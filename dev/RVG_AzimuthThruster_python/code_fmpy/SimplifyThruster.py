# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  <date>	<developer> <description>
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

varname = ['u','v','r','angle','revs','Fx','Fy','Ftot','forceangle']
varunit = ['m/s','m/s','deg/s','deg','RPM','N','N','N','deg']

df = pd.read_csv('output.csv')

# =============================================================================
# df.columns = varname[0:7]
# df['Ftot'] = np.sqrt(df.Fx**2+df.Fy**2)
# df['forceangle'] = np.arctan2(df.Fy,df.Fx)*180/np.pi
# 
# df = df.drop(df[df.r != 0].index)
# 
# 
# ii = df[df.revs==165].index
# plt.scatter(df.angle[ii],df.Fx[ii])
# =============================================================================



