# -*- coding: utf-8 -*-
"""
Created on Tue Dec 13 10:09:31 2022

@author: mathi
"""

import numpy as np




g=9.81
rho = 1.025

#
Tm = 2.786
Bm = 9.6
Lpp = 33.9


#rigid body data
Mass = 531 
KG = 3.54
GM = 2.03
Fsc =0.31
GM_corr = GM-Fsc
KB = 1.56

K44 = Mass*g*GM_corr
R44_rb = 3
M44_rb = Mass*R44_rb**2

R66_rb = 9
M66_rb = Mass*R66_rb**2

Mrb = np.zeros([4,4])
Mrb[0,0] = Mass
Mrb[1,1] = Mass
Mrb[2,2] = M44_rb
Mrb[3,3] = M66_rb

K = np.zeros([4,4])
K[2,2] = K44

#added mass data
a11 = 0.1
M11_a = Mass*a11
a22 = 0.4
M22_a = Mass*a22

z_a_ref = KG-KB
R44_a = 3
M44_a =M22_a*(R44_a**2+z_a_ref**2)
M24_a = -M22_a*z_a_ref
M42_a = M24_a

x_a_ref = -3
R66_a = 9
M66_a = M22_a*(R44_a**2+x_a_ref**2)
M26_a = x_a_ref*M22_a
M62_a = M26_a

Ma = np.zeros([4,4])
Ma[0,0] = M11_a
Ma[1,1] = M22_a
Ma[2,2] = M44_a
Ma[3,3] = M66_a
Ma[1,2] = M24_a
Ma[2,1] = M42_a
Ma[1,3] = M26_a
Ma[3,1] = M62_a

#linear damping
Dl = np.zeros([4,4])
Dl[1,1] = 2*10**5
Dl[2,2] = 2*10**6
Dl[3,3] = 2*10**6

#quadratic damping
Cdx0 = 0.12
Cdx1 = 0.2

x_ap = -Lpp/2 #aft perpendicular
x_fp = Lpp/2 #fore perpendicular
    
x_cf = np.linspace(x_ap,x_fp,15)
    
Cdy_inp = 1
Cdy=Cdy_inp*np.ones(15) #sway drag coefficient

z_visc_ref = -KG+0.2

scale = 10**3

parV = {'Mrb':Mrb*scale,'Ma':Ma*scale,'K':K*scale,
            'Dl':Dl,'Cdx0':Cdx0,'Cdx1':Cdx1,'Cdy':Cdy,'x_visc':x_cf,'Tm':Tm,'Bm':Bm,'Lpp':Lpp,
            'z_a_ref':z_a_ref,'z_visc_ref':z_visc_ref,'CG_rel_wl':0} #save in dict structure

#Thruster parameters
x = Lpp/2
y = 2.7
z_rel_keel = 0.2 #estimate
z = KG-z_rel_keel    
    
rt1 = np.array([x,y,z]) #location of thruster 1
rt2 = np.array([x,-y,z]) #location of thruster 1
Tazi = np.array([1]) #thruster azimuth angle dynamics time constants 
Trevs = np.array([1]) #thruster revolutions dynamics time constants 
dazi_max = 10*np.pi/180 #max azimuth rate
drevs_max = 10 #max revolutions rate
    

parA = {'rt1':rt1,'rt2':rt2,'Tazi':Tazi,'Trevs':Trevs,'dazi_max':dazi_max,'drevs_max':drevs_max}     





