# -*- coding: utf-8 -*-

# ----------------------------------------------------------------------------
# This code is part of the MCSim_python toolbox and repository.
# Created By: M. Marley
# Created Date: 2022-05-08
# Tested:  
# 
# Copyright (C) 202x: <organization>, <place>
# Licensed under GPL-3.0-or-later
# ---------------------------------------------------------------------------
"""
CBF functions for roll stabilization of RVG
"""
# ---------------------------------------------------------------------------
# Imports/dependencies: self-explanatory
# ---------------------------------------------------------------------------

import numpy as np
import kinematics as km
import kinetics as kt

def gx(x,revs,parA,parV):
    
    """
    dx = f(x)+g(x)u, 
    u: azimuth angle, 
    x: (phi,u,v,p,r) 
    revs: parameter
    """
    
    rt = parA['rt'] #thruster location
    
    phi = x[0]
    u = x[1]
    v = x[2]
    p = x[3]
    r = x[4]
    

    v_t = v +r*rt[0]#sway velocity at thruster location
    V = np.sqrt(u**2+v_t**2) #total speed    
    A=9 #rudder area
    rho = 1000 #fluid density
    Cl = 0.5*rho*A*V**2*0.9
    
   
    
    Ct=2.2 #thruster coefficient (just a guess)   
    Fthrust = Ct*revs**2 #propeller thrust force
    
    
    Cazi = Fthrust+Cl
    
    gx = np.zeros(np.shape(x))    
    

    M = parV['Ma']+parV['Mrb']
    DOF4 = [0,1,3,5] #surge sway yaw DOFs in 3DOF model
    DOF4mat = np.ix_(DOF4,DOF4) #for extracting 3x3 matrix from 6x6 matrix   
    M = M[DOF4mat]
    
    invM=np.linalg.inv(M)
    
    gx[1:5] = invM@np.array([0,1,-rt[2],rt[0]])*Cazi

    return gx  

def fx(x,revs,parA,parV):
    
    """
    dx = f(x)+g(x)u, 
    u: azimuth angle, 
    x: (phi,u,v,p,r) 
    revs: parameter
    """
    phi = x[0]
    u = x[1]
    v = x[2]
    p = x[3]
    r = x[4]

    M = parV['Ma']+parV['Mrb']
    Dl = parV['Dl']
    Du = parV['Du']
    Dv = parV['Dv']
    Dr = parV['Dr']
    Dp = parV['Dp']
    K = parV['K']
    
    #Actuator load
    rt = parA['rt'] #thruster location
    
    v_t = v +r*rt[0]#sway velocity at thruster location

   
    Cd = 0.4

    A=9 #rudder area
    rho = 1000 #fluid density
    
    Ct=2.2 #thruster coefficient (just a guess)   
    Fthrust = Ct*revs**2 #propeller thrust force
    
    #decompose loads in body-fixed surge and sway force
    Fx = Fthrust-0.5*rho*A*Cd*u**2#+Ffoilx-Ffoily*angle
    Fy = -0.5*rho*A*u*v_t*1.4
    
    Fact = np.array([Fx,Fy,-rt[2]*Fy,rt[0]*Fy])

    #Sway-roll viscous load
    dvv = parV['dvv'] #sway quadratic force coefficient
    z_b = parV['z_b'] #z-coordinate of buoyancy center
    
    v_b = v-p*z_b #relative velocity at buoyancy center including roll component
    f_vv = dvv*np.abs(v_b)*v_b #viscous sway damping
    
    Fvv = -np.array([0,f_vv,-f_vv*z_b,0]) #sway/roll visc force vector
    
    #Coriolis load    
    DOF3 = [0,1,5] #surge sway yaw DOFs in 3DOF model
    DOF3mat = np.ix_(DOF3,DOF3) #for extracting 3x3 matrix from 6x6 matrix   
    
    nu3 = [u,v,r]
    C3 = kt.Cor3(nu3,M[DOF3mat])      
    Fcor3 = -np.array(C3)@np.array(nu3)
    Fcor = np.array([Fcor3[0],Fcor3[1],0,Fcor3[2]])

    #Nonlinear damping matrix
    D=Dl+Du*abs(u)+Dv*abs(v)+Dp*abs(p)+Dr*abs(r)    

    DOF4 = [0,1,3,5] #surge sway yaw DOFs in 3DOF model
    DOF4mat = np.ix_(DOF4,DOF4) #for extracting 3x3 matrix from 6x6 matrix        
    M = M[DOF4mat]
    D = D[DOF4mat]
    
    #Roll stiffness load
    Fstiff = np.array([0,0,-K[3,3]*phi,0])
    
    fx = np.zeros(np.shape(x))
    fx[0] = p
    invM=np.linalg.inv(M)
    fx[1:5] = invM@(-D@x[1:5]+Fact+Fvv+Fstiff+Fcor)

    return fx 

def dx(x,azi,revs,parA,parV):
    
    dx = fx(x,revs,parA,parV)+gx(x,revs,parA,parV)*azi
    return dx

def CBFroll_azi(x,azi,revs,parA,parV,parC):
    
        phi_max = parC['phi_max']
        t1 = parC['t1']
        t2 = parC['t2']
        
        B1 = x[0]-phi_max
        Fx = fx(x,revs,parA,parV)
        LfB1 = Fx[0]
        B2 = LfB1+t1*B1
        LfB2 = Fx[3]+t1*LfB1
        Gx = gx(x,revs,parA,parV)
        LgB2 = Gx[3]


        azi = np.min([azi,(-t2*B2-LfB2)/LgB2])


        
      

        B1 = -x[0]-phi_max
        LfB1 = -Fx[0]
        B2 = LfB1+t1*B1
        LfB2 = -Fx[3]+t1*LfB1
        LgB2 = -Gx[3]

        azi = np.max([azi,(-t2*B2-LfB2)/LgB2])

        B = np.array([B1,B2])
        dB = np.array([LfB1,LfB2+LgB2*azi])  

        return azi,B,dB
