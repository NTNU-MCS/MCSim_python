# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-02-04
# Revised: <date>	<developer> <description>
#          <date>	<developer> <description>
# Tested:  2022-02-04 M.Marley 
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
This script prepares model data for Gunnerus maneuvering models

Vessel parameters are loaded from MATLAB, original source VERES. 
Additional hydrodynamic data are added by engineering judgement.

Model reference point is at waterline above centre of buoyancy.

Gunnerus has two azimuth thrusters.Thruster location crudely estimated from 
drawings, dynamic time constants are a (somewhat qualified) guess. 
"""

#Load modules
import numpy as np
import scipy.io
import pickle

def GenModelData(inp):

# =============================================================================
# Some inputs
# =============================================================================

    freq_index = inp['freq_index']
    Cdy_inp = inp['Cdy']
    
    
    GMT = 1.7 #ranges from 1.608 to 1.8895 depending on loading condition
    z_visc_ref = -2.6 #reference point for quadratic roll-sway damping, rel waterline
    z_a_ref = -2.5  #reference point for surge added mass, rel waterline
    
    # ============================================================================
    # 6DOF vessel parameters from VERES, extracted using matlab
    # =============================================================================
    
    
    file_name = 'data\\RVGManModelParam6DOF.mat'
    vessel= scipy.io.loadmat(file_name)
    
    L = vessel['Lpp'][0,0] #length between perpendiculars
    B = vessel['Bm'][0,0] #Breadth middle
    T = vessel['Tm'][0,0] #Draught middle
    
    # =============================================================================
    # Calculate stiffness and rigid body mass matrix wrt to true COG
    # =============================================================================
    
    CG_veres = vessel['CG'][0]
    GMT_veres = vessel['GM_T'][0][0]  # metacenter height with COG at waterline
    K = np.array(vessel['C']) #hydrostatic stiffness from VERES
    Mrb_veres = np.array(vessel['Mrb']) #rigid body mass
    
    K44_veres = vessel['C'][3,3] #VERES uses COG at waterline, probably due to lack
    #of other information. For reference
    CB_veres = vessel['CB'][0]# rel waterline
    CB_rel_wl = CB_veres[2]-T
    Mass = Mrb_veres[0,0]# kg, rigid body mass
    Volume = Mass/1025#m^3, displacement
    BMT = GMT_veres+CB_rel_wl #used to calculate Iwp
    IwpT =  Volume*BMT #second area moment of waterplane
    K44 = Mass*GMT*9.81
    CG_rel_wl = 2 #-CB_rel_wl+(IwpT-K44/(9.81*1025))/Volume
    
    GML_veres = vessel['GM_L'][0] # metacenter height with COG at waterline
    GML = GML_veres-CG_rel_wl
    K55 = Mass*GML*9.81
    
    K=K*0
    
    K[3,3] = K44
    K[4,4] = K55
    
    
    
    COB_rel_COG = CB_rel_wl-CG_rel_wl
    
    Mrb = np.zeros((6,6))
    Mrb[0,0] = Mass
    Mrb[1,1] = Mass
    Mrb[2,2] = Mass
    
    Mrb[3,3] = Mrb_veres[3,3]+Mass*CG_rel_wl**2
    Mrb[1,3] = -Mass*CG_rel_wl
    Mrb[3,1] = -Mass*CG_rel_wl
    
    Mrb[4,4] = Mrb_veres[4,4]+Mass*CG_rel_wl**2
    Mrb[0,4] = Mass*CG_rel_wl
    Mrb[4,0] = Mass*CG_rel_wl
    
    Mrb[5,5] = Mrb_veres[5,5]
    
    
    # =============================================================================
    # Extract added mass and radiation damping
    # =============================================================================
    
    
    A = np.array(vessel['A']) #added mass (frequency and velocity dependent)

    #seem of so use other period instead
    vel_index = 4#index, 5.2m/s=10knots, Gunnerus cruising speed
    reference_velocity = vessel['velocities'][0,vel_index]
    
    
    Ma = np.zeros((6,6))
    
    Ma_lf = A[:,:,freq_index,vel_index] #inf added mass
    
    Ma_scale = 1
    
    Ma[0,0] = Ma_lf[0,0]*Ma_scale #surge
    Ma[1,1] = Ma_lf[1,1]*Ma_scale #sway-yaw subsystem
    Ma[1,5] = Ma_lf[1,5] #sway-yaw subsystem
    Ma[5,1] = Ma_lf[5,1] # Ma_lf[5,1] #sway-yaw subsystem
    
    #Ma[1,5] = (Ma_lf[1,5]+Ma_lf[5,1])*0.5*Ma_scale #sway-yaw subsystem
    #Ma[5,1] = Ma[1,5] # Ma_lf[5,1] #sway-yaw subsystem
    
    Ma[5,5] = Ma_lf[5,5]*Ma_scale #sway-yaw subsystem
    
    
    Ma_wf = A[:,:,freq_index,vel_index]
    
    
    Ma[3,3] = Ma_wf[3,3]*Ma_scale #roll
    Ma[3,1] = Ma_wf[3,1]*Ma_scale #roll-sway
    Ma[1,3] = Ma_wf[1,3]*Ma_scale #roll-sway
    Ma[3,5] = Ma_wf[3,5]*Ma_scale #roll-yaw
    Ma[5,3] = Ma_wf[5,3]*Ma_scale #roll-yaw
    
    
    
    Dl = np.zeros((6,6)) #anp.array(vessel['Bv']) #linear damping matrix
    Dl[3,3] = 2*10**6
    Dl[5,5] = 2*10**6
    Dl[1,1] = 2*10**5
    
    #source does not include heave and pitch damping, so adding some
    #large values (we are not interested in heave/pitch response)
    #Dl[2,2] = Dl[1,1]*100 #heave=100*sway 
    #Dl[4,4] = Dl[5,5]*100 #pitch = 100*yaw
    
    #Viscous loads
    #add some order of magnitude quadratic damping in surge/sway/yaw
    Cdx0=0.12 #surge drag coefficient
    Cdx1 = 0.2
    
    x_ap = -L/2+vessel['CG'][0,0] #aft perpendicular
    x_fp = L/2+vessel['CG'][0,0] #fore perpendicular
    
    x_cf = np.linspace(x_ap,x_fp,15)
    
    Cdy=Cdy_inp*np.ones(15) #sway drag coefficient
    
    Cdy[0:2] = Cdy[0]
    Cdy[12:14] = Cdy[14]
    
    # =============================================================================
    # Actuator model parameters (equivalent thruster placed at centerline)
    # =============================================================================
    #coordinates rel waterline
    x = vessel['CG'][0,0]-L/2+2
    y = 2.7
    z = -2.5 #estimate
    
    
    rt1 = np.array([x,y,z]) #location of thruster 1
    rt2 = np.array([x,-y,z]) #location of thruster 1
    rt = np.array([x,0,z]) #location of equivalent thruster at centreline
    Tazi = np.array([1]) #thruster azimuth angle dynamics time constants 
    Trevs = np.array([1]) #thruster revolutions dynamics time constants 
    dazi_max = 10*np.pi/180 #max azimuth rate
    drevs_max = 10 #max revolutions rate
    
    #dict containing actuator model parameters
    parA = {'rt':rt,'rt1':rt1,'rt2':rt2,'Tazi':Tazi,'Trevs':Trevs,'dazi_max':dazi_max,'drevs_max':drevs_max} 
    
    
    #extract and store 4DOF vessel data 
    DOF4 = [0,1,3, 5] #surge sway roll yaw DOFs included in 4DOF model
    DOF4mat = np.ix_(DOF4,DOF4) #for extracting 4x4 matrix from 6x6 matrix
    
    #store 4DOF data in parV containing vessel parameters
    parV = {'Mrb':Mrb[DOF4mat],'Ma':Ma[DOF4mat],'K':K[DOF4mat],
            'Dl':Dl[DOF4mat],'Cdx0':Cdx0,'Cdx1':Cdx1,'Cdy':Cdy,'x_visc':x_cf,'Tm':T,'Bm':B,'Lpp':L,
            'z_a_ref':z_a_ref,'z_visc_ref':z_visc_ref,'CG_rel_wl':CG_rel_wl,
            'reference_velocity':reference_velocity} #save in dict structure

    return parV, parA


def GenModelData2(inp):
    
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
