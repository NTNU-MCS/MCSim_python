# -*- coding: utf-8 -*-
"""
MCsim library of general functions

Created on: Jan 26 2022

@author: M. Marley
"""

import numpy as np

def rad2pipi(angle): 
    "Maps angle to [-pi,pi]"
    angle = np.arctan2(np.sin(angle),np.cos(angle))
    return angle 


# def Cor6(nu,M):
#     """Returns 6DOF Coriolis and centripetal force matrix
#     Input: 
#             nu: 6DOF body-fixed velocity vector
#             M: inertia matrix
#     Output: 
#             Cor6: 6x6 matrix such that Cor6*nu are the Coriolis and centripetal
#             forces
#     """
    
#     Cor6 = np.array([[np.zeros(3,3)],-Smat(M[0:2,0:2]@nu[0:2])])
                     
    
#     return Cor6


def Cor3(nu,M):
    """Returns 3DOF Coriolis and centripetal force matrix
    Input: 
            nu: 3DOF body-fixed velocity vector
            M: inertia matrix
    Output: 
            Cor3: 3x3 matrix such that Cor3*nu are the Coriolis and centripetal
            forces
    """
    Cor3= [[0, 0, -M[1,1]*nu[1]-0.5*(M[1,2]+M[2,1])*nu[2]],
                [0, 0, M[0,0]*nu[0]],
                [M[1,1]*nu[1]+0.5*(M[1,2]+M[2,1])*nu[2], -M[0,0]*nu[0], 0]]
   
    return Cor3

def Rotx(phi):
    "Outputs 3DOF rotation matrix about x axis"
    Rx = np.array([[0, 0, 1],
          [0, np.cos(phi), -np.sin(phi)],
          [0, np.sin(phi), np.cos(phi)]])
    return Rx

def Roty(theta):
    "Outputs 3DOF rotation matrix about z axis"
    Ry = np.array([[np.cos(theta),0, np.sin(theta)],
          [0,1,0],
          [-np.sin(theta), 0, np.cos(theta)]])
    return Ry

def Rotz(psi):
    "Outputs 3DOF rotation matrix about z axis"
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0],
          [np.sin(psi), np.cos(psi), 0],
          [0, 0, 1]])
    return Rz

def Smat(x):
    "Outputs 3x3 skew-symmetric cross-product matrix"
    S = np.array([[0, -x[2], x[1]],
          [x[2], 0, -x[0]],
          [-x[1], x[0], 0]])
    return S


def dot_eta3(psi,nu):
    "Derivative of 3DOF pose eta=[x y psi]"
    deta3 = Rotz(psi)@nu
    return deta3

def dot_nu3_man_lq(psi,nu,nu_c_n,F,parV):
    """Derivative of 3DOF body-fixed velocity vector for manuevering model,
       with linear+quadratic damping formulation. Ship moving in uniform and
       steady currents
    Input: 
            psi: ship heading
            nu: body-fixed velocity
            nu_c_n: current velocity vector in inertial frame
            F: external force vector (thruster forces, wave disturbance etc)
            parV: dict containing system matrices: Mrb, Ma, Dl, Du, Dv, Dr
    Output: 
            dnu3: body-fixed accelerations
    """   
    Mrb = parV['Mrb']
    Ma = parV['Ma']
    Dl = parV['Dl']
    Du = parV['Du']
    Dv = parV['Dv']
    Dr = parV['Dr']

    invM = np.linalg.inv(Mrb+Ma)
    nu_c = np.transpose(Rotz(psi))@nu_c_n
    S=np.array([[0, -1, 0],[1, 0, 0],[0, 0, 1]])
    S=Smat(np.array([0,0,nu[2]]))
    dnu_c = np.transpose(S@Rotz(psi))@nu_c_n
    nu_r = nu-nu_c
    
    D=Dl+Du*abs(nu[0])+Dv*abs(nu[1])+Dr*abs(nu[2])
    Ca = Cor3(nu_r,Ma)
    Crb = Cor3(nu,Mrb)
    
    dnu3 = invM@(F-D@nu_r-Crb@nu-Ca@nu_r+Ma@dnu_c)
    return dnu3    
    
def forceAzi3(force,azi,loc):
    """"3DOF body-fixed forces from azimuth thruster
    
    Input: 
            force: thruster force
            azi: azimuth angle
            loc: location of thruster in body-fixed coordinate system
    Output: 
            F: 3DOF force vector
    """
    eps1 = np.array([1,0,0])
    S = Smat(loc)
    R = Rotz(azi)   
    F6 =  force*np.concatenate((R@eps1, S@R@eps1)) #6DOF vector
    F = F6[[0,1,5]]
    
    return F

def forceAzi6(force,azi,rt):
    """6DOF body-fixed forces from azimuth thruster
    
    Input: 
            force: thruster force
            azi: azimuth angle
            rt: location of thruster in body-fixed coordinate system
    Output: 
            F: 6DOF force vector
    """
    eps1 = np.array([1,0,0])
    S = Smat(rt)
    R = Rotz(azi)   
    F =  force*np.concatenate((R@eps1, S@R@eps1)) #6DOF vector
    
    return F
    

