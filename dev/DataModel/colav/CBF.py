import math
import numpy as np 
import copy 
from simulation.SimulationTransform import SimulationTransform 
from time import time


class CBF:
    def __init__(self, safety_radius_m ,k1 = 1, lam = 0.5, dt = 1, gamma_2 = 1,
                sigma = 1.0, t = 0, t_t = 0, t_tot = 150, rd_max = 1, n_ub = 20, 
                P=np.diagflat([0,1]), transform=SimulationTransform()): 
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
        self._P = P
        self._transform = transform
        
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
        return p_dot

    def _get_g(self, z):  
        return self._S @ z
    
    def _get_B1_dot(self, pe_unit, u, z): 
        B1_dot = -pe_unit.T  @ z.reshape((2,1)) * u
        return  np.ravel(B1_dot)
    
    def _get_B2_dot(self, pe_unit, pe_norm, z, B1, u, rdc): 
        B2_dot  = np.empty((self._ais_data_len))
        L_g_B2 =  np.empty([self._ais_data_len, 2]) 

        for i in range(self._ais_data_len):   
            _e = np.ravel(pe_unit[:,i])  
            L_g_B2[i,:] = _e @ np.concatenate((z, u*self._S @ z), axis=1)
            B2_dot[i] = (-(u**2/pe_norm[i])*(_e @ self._S.T @ z)**2 -
            (u*self._sigma/(self._sigma**2 + B1[i]**2)) * _e @ z * u  - 
            L_g_B2[i,:]@ np.array([[0], [rdc]]))
        return B2_dot, L_g_B2 
            
    def _get_nominal_control(self, z, tq): 
        z_tilde = np.concatenate((tq, self._S @ tq), axis=1).T @ z
        rd = (-self._k1 * z_tilde[1]) / math.sqrt(1 - self._lam**2 * z_tilde[0]**2)
        return rd

    def _get_safe_control(self, pe_unit, pe_norm, z, B1, u, rd_n): 
        alpha_B1 = u*np.arctan(B1/self._sigma)
        B1_dot = self._get_B1_dot(pe_unit, u, z)
        B2 = B1_dot.T + alpha_B1
        B2_dot, L_g_B2 = self._get_B2_dot(pe_unit, pe_norm, z, B1, u, rd_n)  
        closest = np.argmin(pe_norm) 
        if np.all(B2_dot <= self._gamma_2 * B2): 
            return rd_n
        else:
            a = B2_dot + self._gamma_2 * B2 
            b = np.array([0, L_g_B2[closest,1]])
            rds = rd_n - ((a[closest] * b.T) /(b @ b.T))  
            return rds[0,1] 

    def _process_data(self, p ,u ,z ,tq, po, zo, uo):  
        self._running = True
        
        t = 0 
        h_p = np.empty((2, self._hist_len)) 

        po_dot = np.multiply(zo, uo) 
        po_vec = po.T.reshape((-1,1)) + np.arange(self._hist_len) * po_dot.T.reshape((-1,1)) * self._dt 
        for t in range(self._hist_len): 
            if not self._running: return None 
            h_p[:,t] = p.T 
            # find safe input
            rd_n = self._get_nominal_control(z, tq)
            pe = p - po_vec[:, t].reshape(2,-1, order='F')
            pe_norm = np.linalg.norm(pe, axis=0) 
            pe_unit = pe / pe_norm 
            B1 = self._safety_radius_m -pe_norm
            rd = self._get_safe_control(pe_unit, pe_norm, z, B1, u, rd_n) 
            p_dot = self._get_f(z, zo, u, uo) 
            z_dot = self._get_g(z) 
            p = p + p_dot*self._dt  
            z = z + z_dot*rd*self._dt 
            z = z/np.linalg.norm(z)  
        cbf_data =  {"p": h_p}
        return cbf_data
    
    def convert_data(self, cbf_data):
        lat_o = self._gunn_data['lat']
        lon_o = self._gunn_data['lon']
        geo = []
        converted_data={}
        for col in range(cbf_data['p'].shape[1]):
            x = cbf_data['p'][0,col]
            y = cbf_data['p'][1,col]
            lat, lon = self._transform.xyz_to_coords(x, y, lat_o, lon_o)
            geo.append([lon, lat])
        converted_data['cbf'] = geo 
        return converted_data

    def get_cbf_data(self): 
        p, u, z, tq, po, zo, uo = self._sort_data() 
        cbf_data = self._process_data(p ,u ,z ,tq, po, zo, uo)
        return cbf_data
    
    def stop(self):
        self._running = False
