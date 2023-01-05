# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: 2022-04-29	M. Marley Updated thruster model
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley Checked that results make sense
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
Library of functions for running the 3DOF RVG maneuvering model. 
May also be used to model any vessel with two azimuth thrusters, by loading
desired vessel and actuator data. 
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
import numpy as np
import kinematics as km
import kinetics as kt
import thrusters as th

# =============================================================================
# Functions
# =============================================================================    

    
def dot_RVG_Man_4DOF(x,tau,w,parV,parA,parS):
    """Time derivative of RVG 6DOF vessel model, moving in uniform
    and steady currents. Linear stiffness matrix, consistent with small angle
    assumption in roll and pitch. Focus is on horizontal DOFs + roll. 
    Uses a linear+quadratic damping formulation. 
    Uses the thrusters.RVGazimuth_man function, with the two RVG azimuth 
    thrusters are modelled as an equivalent thruster located at centreline. 
    Thruster states are azimuth angle and propeller revs. 
    Control input is desired thruster states, while disturbance input is 
    body-fixed forces (e.g. wind and wave loads). 
    Input: 
        x=[eta,nu,azi,revs] in R^14: state vector, where 
            eta in R^6: position and orientation
            nu in R^6: body-fixed velocities 
            azi in R: thruster azimuth angle
            revs in R: propeller revs  [RPM] 
        tau =[azi_d, revs_d] in R^2: desired thruster states
        w in R^3: external force vector acting on body   
        parV: dict containing vessel parameters 
        parA: dict containing actuator parameters          
        parS: dict containing simulation parameters 
    Output: 
        dot_x: 14x1 vector of time derivatives   
    """
    # Created: 2022-11-28 M.Marley 
    # Tested: 2022-11-28 M.Marley  
    
    # =============================================================================
    # Kinematics
    # =============================================================================
    
    eta = x[0:4] #[0:6] 
    psi = x[3]
    nu=x[4:8]  
    
    dx = np.zeros(len(x))
    
    dx[0:4] = km.dot_eta4(psi,nu)    
    
    # =============================================================================
    # Current kinematics
    # =============================================================================    
    
    u = nu[0] #DOF1 surge
    v = nu[1] #DOF2 sway
    p = nu[2] #DOF4 yaw
    r = nu[3] #DOF6 roll
    
    azi = x[8]
    revs = x[9]
    rt1 = parA['rt1'] #thruster 1 location
    rt2 = parA['rt2'] #thruster 2 location
    
    nu_c_n_2 = parS['Uc']*np.array([np.cos(parS['betac']),np.sin(parS['betac'])]) #inertial frame current velocity
    
    R = km.Rot_planar(psi)
    
    nu_c_2 = np.transpose(R)@nu_c_n_2 #body-fixed current velocity
    u_c = nu_c_2[0]
    v_c = nu_c_2[1]

    ur = u-u_c
    vr = v-v_c

    S = np.array([[0,-1],[1,0]])

    dnu_c_2 = r*np.transpose(S*R)@nu_c_n_2
    dnu_c = np.array([dnu_c_2[0],dnu_c_2[1],0,0])

    nu_r = nu-np.array([u_c,v_c,0,0])

    #two thruster model
    u_t1 = ur-r*rt1[1] #surge velocity at thruster location
    v_t1 = vr +r*rt1[0]#sway velocity at thruster location 


    Fx1, Fy1 = th.RVGazimuth_man(u_t1,v_t1,azi,revs) 
    ForceAct1 = np.array([Fx1,Fy1,-Fy1*rt1[2],Fy1*rt1[0]-Fx1*rt1[1]])        
    
    u_t2 = ur-r*rt2[1] #surge velocity at thruster location
    v_t2 = vr +r*rt2[0]#sway velocity at thruster location 

    Fx2, Fy2 = th.RVGazimuth_man(u_t2,v_t2,azi,revs) 
    ForceAct2 = np.array([Fx2,Fy2,-Fy2*rt1[2],Fy2*rt2[0]-Fx2*rt2[1]])       

    Fact = (ForceAct1+ForceAct2)
    
    Fx = Fact[0]
    
    #Coriolis forces
    
    Mrb = parV['Mrb']

    Fcorr_RB = kt.Cor4(nu,Mrb,parV['CG_rel_wl'])
    
    Ma = parV['Ma']

    Fcorr_a = kt.Cor4(nu_r,Ma,parV['z_a_ref'])
      
    #viscous damping in sway and heave
    
    Cdx = parV['Cdx0']+parV['Cdx1']*np.abs(np.arctan2(vr,ur))
    #print(Cdx)
    
    F1 = 0.5*1025*Cdx*parV['Tm']*parV['Bm']*ur*np.abs(ur)
    
    F2,F6 = kt.crossflowdrag(vr-parV['z_visc_ref']*p,r,parV)
    
    F4 = -F2*parV['z_visc_ref']
    
    F_visc = np.array([F1,F2,F4,F6])

    
    
    invM = np.linalg.inv(Mrb+Ma)
    D = parV['Dl']
    K = parV['K']
    
    dx[4:8] = invM@(Fact-Fcorr_RB-Fcorr_a-F_visc-D@nu_r-K@eta+Ma@dnu_c+w)
    
    
    dx[8] = -(1/parA['Tazi'])*(x[8]-tau[0])
    dx[8] = np.sign(dx[8])*np.min([np.abs(dx[8]), parA['dazi_max']])
    dx[9] = -(1/parA['Trevs'])*(x[9]-tau[1])
    dx[9] = np.sign(dx[9])*np.min([np.abs(dx[9]), parA['drevs_max']])
    return dx, Fx  

def int_RVGMan4(x,u,w,parV,parA,parS):
    """Time integration of RVG 6DOF vessel model, moving in uniform
    and steady currents. Linear stiffness matrix, consistent with small angle
    assumption in roll and pitch. Focus is on horizontal DOFs + roll. 
    Uses a linear+quadratic damping formulation. 
    Uses the thrusters.RVGazimuth_man function, with the two RVG azimuth 
    thrusters are modelled as an equivalent thruster located at centreline. 
    Thruster states are azimuth angle and propeller revs. 
    Control input is desired thruster states, while disturbance input is 
    body-fixed forces (e.g. wind and wave loads). 
    Input: 
        x=[eta,nu,azi,revs] in R^14: state vector, where 
            eta in R^6: position and orientation 
            nu in R^6: body-fixed velocities 
            azi in R: thruster azimuth angle
            revs in R: propeller revs  [RPM] 
        u =[azi_d, revs_d] in R^2: desired thruster states
        w in R^3: external force vector acting on body   
        parV: dict containing vessel parameters 
        parA: dict containing actuator parameters          
        parS: dict containing simulation parameters 
    Output: 
        x_next: 14x1 vector of vessel state at next time step  
    """
    # Created: 2022-05-02 M.Marley 
    # Tested: 2022-05-02 M.Marley  

    dt = parS['dt']
    k1, Fx = dot_RVG_Man_4DOF(x,u,w,parV,parA,parS)
    k2, Fx = dot_RVG_Man_4DOF(x+k1*dt/2,u,w,parV,parA,parS)
    k3, Fx = dot_RVG_Man_4DOF(x+k2*dt/2,u,w,parV,parA,parS)
    k4, Fx = dot_RVG_Man_4DOF(x+k3*dt/2,u,w,parV,parA,parS)
    x_next = x+(k1+2*k2+2*k3+k4)*dt/6
    x_next[5]=km.rad2pipi(x_next[5])

    return x_next, Fx


def DefaultModelData():
    
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
    Fsc =0.3
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
    a11 = 0.05
    M11_a = Mass*a11
    a22 = 0.4
    M22_a = Mass*a22
    
    z_a_ref = -KG+KB
    R44_a = 3
    M44_a =M22_a*(R44_a**2+z_a_ref**2)
    M24_a =  -M22_a*z_a_ref #8.92E+04/10**3# 8.92E+04/10**3#
    M42_a = M24_a
    
    x_a_ref = -3
    R66_a = 9
    a66 = 0.65
    
    M66_a = M22_a*(R66_a**2+x_a_ref**2)
#    M66_a = M66_rb*a66
    
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
    Dl[1,1] = 1*10**5
    Dl[2,2] = 10*10**6
    Dl[1,2] = KG*Dl[1,1]
    Dl[2,1] = Dl[1,2]
    Dl[3,3] = 0
    
    

  
    #quadratic damping
    Cdx0 = 0.12
    Cdx1 = 0.2
    
    x_ap = -Lpp/2 #aft perpendicular
    x_fp = Lpp/2 #fore perpendicular
        
    x_cf = np.linspace(x_ap,x_fp,15)
        
    Cdy_inp = 1.5
    Cdy=Cdy_inp*np.ones(15) #sway drag coefficient
    
    z_visc_ref = -KG
    
    scale = 10**3

    dampratio = 0.2
    Dl[2,2] = dampratio*2*np.sqrt(K[2,2]*(Mrb[2,2]+Ma[2,2]))*scale
    
    parV = {'Mrb':Mrb*scale,'Ma':Ma*scale,'K':K*scale,
                'Dl':Dl,'Cdx0':Cdx0,'Cdx1':Cdx1,'Cdy':Cdy,'x_visc':x_cf,'Tm':Tm,'Bm':Bm,'Lpp':Lpp,
                'z_a_ref':z_a_ref,'z_visc_ref':z_visc_ref,'CG_rel_wl':0} #save in dict structure
    
    #Thruster parameters
    x = -Lpp/2
    y = 2.7
    z_rel_keel = -1 #estimate
    z =-KG+z_rel_keel    
        
    rt1 = np.array([x,y,z]) #location of thruster 1
    rt2 = np.array([x,-y,z]) #location of thruster 1
    Tazi = np.array([1]) #thruster azimuth angle dynamics time constants 
    Trevs = np.array([1]) #thruster revolutions dynamics time constants 
    dazi_max = 30*np.pi/180 #max azimuth rate
    drevs_max = 30 #max revolutions rate
        
    
    parA = {'rt1':rt1,'rt2':rt2,'Tazi':Tazi,'Trevs':Trevs,'dazi_max':dazi_max,'drevs_max':drevs_max}     

    return parV,parA
