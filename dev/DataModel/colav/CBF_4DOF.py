import math
import os
from pathlib import Path
import sys
import numpy as np 
import copy 
from simulation.SimulationTransform import SimulationTransform 
from colav.CBF import CBF

from time import time

par_path = str(Path(os.path.dirname(__file__)).parents[2])
library_path = par_path + '\\lib'
sys.path.append(library_path)

module_path = par_path + '\\models\\RVG_maneuvering4DOF'
sys.path.append(module_path)

import kinematics as km
import visualization as viz
import Module_RVGManModel4DOF as model


class CBF_4DOF(CBF):
    def __init__(self, safety_radius_m ,k1 = 1, k2 = 1, k3= 1, lam = 0.5, dt = 0.2, gamma_2 = 40,
                gamma_1 = 0.2, t_tot = 600, rd_max = 1, n_ub = 20, max_rd = 0.18
                , transform=SimulationTransform(), ): 
        super(CBF_4DOF, self).__init__( safety_radius_m ,k1 = 1, lam = 0.5, dt = 0.2, gamma_2 = 40,
                gamma_1 = 0.2, t_tot = 600, rd_max = 1, n_ub = 20, max_rd = 0.18
                , transform=SimulationTransform())
        self.dt = dt
        self.parV, self.parA = model.DefaultModelData() 
        self.k2 = k2
        self.k3 = k3
        self._max_azi = 30 * math.pi / 180
        self._max_azi_d = 1 * math.pi / 180
        self._rvg_origo = {}

    def infer_azi_revs(self, u, r):
        revs = 100#u*5.14/300 # revs assuming linear behavior, should replace this
        azi = 0
        return azi, revs
        
    def _get_azi(self, r, r_safe, p_azi):
        ad = -self.k2*(r-r_safe) + self.k3 * r_safe
        ad = float(ad[0])  
        if abs(p_azi - ad) > self._max_azi_d:
            ad = p_azi + np.sign(ad) * self._max_azi_d

        if ad > self._max_azi: ad =  self._max_azi
        elif ad < - self._max_azi: ad = - self._max_azi
        
        return ad

    def _process_data(self, p ,u ,z ,tq, po, zo, uo, ret_var):  
        self._running = True
        start_time = time()
        maneuver_start = None
        
        t = 0 
        h_p = np.zeros((2, self._hist_len)) 

        po_dot = np.multiply(zo, uo)  
        po_vec = po.T.reshape((-1,1)) + np.arange(self._hist_len) * po_dot.T.reshape((-1,1)) * self._dt 
        
        parS = {'dt':self.dt, 'Uc': 0, 'betac':  0} 
        # initialize eta and nu 
        yaw = math.atan2(z[0], z[1])  
        eta = np.array([0,0,0,yaw]) #North East Yaw Roll
        nu=np.array([u,0,0,0]) #surge sway yaw roll velocities
        azi, revs = self.infer_azi_revs(u,z)
        thrust_state = np.array([azi,revs])
        x=np.concatenate((eta, nu, thrust_state)) 

        for t in range(self._hist_len): 
            if not self._running: return None 
            h_p[:,t] = p.T 
            rd_n = self._get_nominal_control(z, tq)
            pe = p - po_vec[:, t].reshape(2,-1, order='F') 
            pe_norm = np.linalg.norm(pe, axis=0)
            closest = np.argmin(pe_norm)
            ei = pe[:,closest].reshape((2,1))
            norm_ei = pe_norm[closest]
            zi = zo[:,closest].reshape((2,1))
            ui = uo[closest]  
            B1 = self._safety_radius_m - norm_ei 
            LfB1 = -(ei.T @ (u*z - ui*zi)) / norm_ei 
            B2 = LfB1 + (1/self._gamma_1)*B1 
            LfB2 = ((ei.T@(u*z - ui*zi))**2)/norm_ei**3 - (np.linalg.norm((u*z - ui*zi), axis=0)**2)/norm_ei + (1/self._gamma_1)*LfB1 
            LgB2 = (-u*ei.T@self._S@z)/norm_ei 
            B2_dot = LfB2 + LgB2*rd_n 

            if B2_dot <= -(1/self._gamma_2)*B2:
                rd = rd_n
            else:
                a = LfB2 + LgB2*rd_n + (1/self._gamma_2)*B2
                b = LgB2
                rd = rd_n - (a@b.T)/(b*b.T + self._epsilon)   
                if maneuver_start is None:
                    maneuver_start = t * self._dt
            
            azi = self._get_azi(x[8],rd, azi)  
            thrust_state = [azi,revs]
            Fw = np.zeros(4)
            x = model.int_RVGMan4(x, thrust_state, Fw, self.parV, self.parA, parS)
            p[0,0] = x[1] 
            p[1,0] = x[0]
            z[0,0] = math.sin(x[3])
            z[1,0] = math.cos(x[3])
            z = z/np.linalg.norm(z)  

        if maneuver_start is not None:
            start_maneuver_at = start_time + maneuver_start
        else:
            start_maneuver_at = -1 
        cbf_data =  {
            "p": h_p,
            "maneuver_start" : start_maneuver_at
                    } 
        ret_var.put(cbf_data) 
        return cbf_data