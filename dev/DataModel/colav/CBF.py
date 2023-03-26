import math
import numpy as np
import time  
import copy
from utils.Point import Point
from tqdm import tqdm

class CBF:
    def __init__(self, safety_radius_m ,k1 = 1, lam = 0.5, dt = 1, gamma_2 = 1.5,
                sigma = 27.0, t = 0, t_t = 0, t_tot = 60, rd_max = 1, n_ub = 20): 
        self._safety_radius_m = safety_radius_m
        self._gunn_data = {}
        self._ais_data = {} 
        self._ais_data_len = 0 
        self._S = np.mat('0 -1; 1 0')
        self._k1 = k1
        self._lam = lam
        self._dt = dt
        self._gamma_2 = gamma_2
        self._sigma = sigma  
        self._t_tot = t_tot
        self._rd_max = rd_max
        self._n_ub = n_ub
        self._hist_len = int(t_tot / dt) 
        self._running = False
        
    def update_cbf_data(self, arpa_gunn_data, arpa_data): 
        self._gunn_data = copy.deepcopy(arpa_gunn_data) 
        self._ais_data = copy.deepcopy(arpa_data)
        self._ais_data_len = len(self._ais_data)
        return 
    
    def _sort_data(self): 
        p = self._gunn_data['p']
        u = self._gunn_data['u']
        z = self._gunn_data['z']
        tq = self._gunn_data['tq']
        po = np.empty((2, self._ais_data_len))
        zo = np.empty((2, self._ais_data_len)) 
        uo = np.empty((self._ais_data_len))

        for idx, ais_item in enumerate(self._ais_data):
            po[0, idx] = ais_item["po_x"]
            po[1, idx] = ais_item["po_y"] 
            uo[idx] = ais_item["uo"]  
            zo[:, idx] = ais_item["zo"].T 

        return p ,u ,z ,tq, po, zo, uo
    
    def _get_f(self, z, zo, u, uo):
        p_dot = z*u
        po_dot = np.multiply(zo, uo)
        return p_dot, po_dot

    def _get_g(self, z):  
        return self._S @ z
    
    def _get_B1_dot(self, pe, u, z):
        B1_dot = []
        for col in pe.T: 
            res = -(col/np.linalg.norm(col)) @ z* u 
            B1_dot = np.append(B1_dot, res)  
        return B1_dot
    
    def _get_B2_dot(self, pe, z, B1, u, rdc): 
        B2_dot  = np.empty((self._ais_data_len))
        norm_pe = np.linalg.norm(pe, axis=0) 
        e = pe / norm_pe  
        for i in range(self._ais_data_len):   
            _e = np.ravel(e[:,i])  
            B2_dot[i] = (-(u**2/norm_pe[i])*(_e @ self._S.T @ z)**2 -
            (u*self._sigma/(self._sigma**2 + B1[i]**2)) * _e @ z * u  -
            _e @ np.concatenate((z, u*self._S @ z), axis=1) @ 
            np.array([[0], [rdc]]))
        return B2_dot 
            
    def _get_nominal_control(self, z, tq): 
        z_tilde = np.concatenate((tq, self._S @ tq), axis=1).T @ z
        rd = (-self._k1 * z_tilde[1]) / math.sqrt(1 - self._lam**2 * z_tilde[0]**2)
        return rd

    def _process_data(self, p ,u ,z ,tq, po, zo, uo):  
        self._running = True
        t = 0
        t_t = 0
        h_z = np.empty((2, self._hist_len))
        h_p = np.empty((2, self._hist_len))
        h_rd = np.empty((self._hist_len))
        h_po = np.empty((self._ais_data_len*2, self._hist_len))  

        for t in tqdm(range(self._hist_len)): 

            if not self._running: return None

            h_z[:,t] = z.T
            h_p[:,t] = p.T
            h_rd[t] = 0
            h_po[:, t] = np.reshape(po, self._ais_data_len*2)
            rd_n = self._get_nominal_control(z, tq)
            rd_c = np.linspace(-self._rd_max, self._rd_max, self._n_ub)
            min_rd = 0
            pe = p - po 
            B1 = self._safety_radius_m -np.linalg.norm(pe, axis=0) 
            B1_dot = self._get_B1_dot(pe, u, z) 
            alpha_1 = u*np.arctan(B1/self._sigma) 
            B2 = B1_dot + alpha_1 
            for rd in rd_c: 
                B2_dot = self._get_B2_dot(pe, z, B1, u, rd) 
                if all(B2_dot <= self._gamma_2 * B2) and (abs(rd_n - rd) < abs(rd_n - min_rd)):
                    min_rd = rd

            p_dot, po_dot = self._get_f(z, zo, u, uo) 
            z_dot = self._get_g(z)
            p = p + p_dot*self._dt
            po = po + po_dot*self._dt
            z = z + z_dot*min_rd*self._dt   
        cbf_data =  {"z": h_z, "p": h_p, "rd": h_rd, "po": h_po}
        return cbf_data

    def get_cbf_data(self):
        p, u, z, tq, po, zo, uo = self._sort_data()
        cbf_data = self._process_data(p ,u ,z ,tq, po, zo, uo)
        return cbf_data
    
    def stop(self):
        self._running = False
