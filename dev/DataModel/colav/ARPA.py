import math
import numpy as np 
from simulation.SimulationTransform import SimulationTransform 
import copy

class ARPA:
    def __init__(self, safety_radius_m, safety_radius_tol=1.5, 
                transform=SimulationTransform(), gunnerus_mmsi ='' , 
                dummy_gunnerus = None, dummy_vessel = None):
        
        self._gunnerus_mmsi = gunnerus_mmsi
        self._gunnerus_data = {}
        self._ais_data = {}
        self._safety_radius_m = safety_radius_m
        self._safety_radius_tol = safety_radius_tol
        self._transform = transform
        self.dummy_gunnerus = dummy_gunnerus
        self.dummy_vessel = dummy_vessel
        pass

    def update_gunnerus_data(self, data):
        self._gunnerus_data = data 
    
    def update_ais_data(self, data): 
        self._ais_data = data 

    def _has_coords(self, message): 
        msg_keys = message.keys() 
        has_data = ('lat' in msg_keys and 'lon' in msg_keys)
        return has_data
    
    def _has_course(self, message): 
        msg_keys = message.keys() 
        has_data = ('course' in msg_keys and 'speed' in msg_keys)
        return has_data

    # extract AIS parameters for ARPA 
    def _get_ais_data(self, ais_message, gunnerus_data):
        ais_has_coords = self._has_coords(ais_message)
        lon, lat, course, speed_kn = (None, None, None, None)

        if ais_has_coords:
            lon = ais_message['lon']
            lat = ais_message['lat']
            ais_has_course = self._has_course(ais_message)

            if ais_has_course:
                course = ais_message['course']
                speed_kn = ais_message['speed'] 
            else:
                course = 0
                speed_kn = 0

            po_x, po_y, _ = self._transform.coords_to_xyz(
            northings = lat, 
            eastings = lon, 
            altitude = 0, 
            y_o = gunnerus_data['lat'], 
            x_o = gunnerus_data['lon'], 
            z_o = 0) 
            
            uo = self._transform.kn_to_mps(speed_kn)
            zo = np.array([
                [math.sin(math.radians(course))], 
                [math.cos(math.radians(course))]
                ])

            uo_x = zo[0][0] * uo
            uo_y = zo[1][0] * uo

            ais_data = { 'po_x': po_x, 'po_y': po_y, 'uo': uo, 
                        'zo': zo, 'uo_x': uo_x, 'uo_y': uo_y }
            
            return ais_data
        return None
    
    # extract gunnerus parameters for ARPA 
    def _get_gunnerus_data(self):
        gunn_speed = None
        gunn_course = None

        gunnerus_data = copy.deepcopy(self._gunnerus_data) 

        if not self._has_coords(self._gunnerus_data): return None

        if (self.dummy_gunnerus is not None): 
            gunn_speed = self.dummy_gunnerus['spd_over_grnd']
            gunn_course = self.dummy_gunnerus['true_course']
        else: 
            gunn_speed = self._gunnerus_data['spd_over_grnd']
            gunn_course = self._gunnerus_data['true_course']
        
        gunn_lat = self._transform.deg_2_dec(
            gunnerus_data['lat'], 
            gunnerus_data['lat_dir']
            )
        
        gunn_lon = self._transform.deg_2_dec(
            gunnerus_data['lon'], 
            gunnerus_data['lon_dir']
            )
        
        z = np.array([
            [math.sin(math.radians(gunn_course))], 
            [math.cos(math.radians(gunn_course))]
            ])
        
        tq = np.array([
            [math.sin(math.radians(gunn_course))], 
            [math.cos(math.radians(gunn_course))]
            ])
        
        p = np.array([[0],[0]])
        u = self._transform.kn_to_mps(gunn_speed)
        ux = u*z[0][0]
        uy = u*z[1][0] 
        gunn_data =  {'p': p, 'u': u, 'ux': ux, 'uy': uy, 'z': z, 
                    'tq': tq, 'lon': gunn_lon, 'lat': gunn_lat}
        return gunn_data
    
    # get cpa for an ais vessel, ais has to be pre processed
    def _get_cpa(self, gunn_data, ais_data_item): 
        ux = float(gunn_data["ux"])
        uy = float(gunn_data["uy"])
        p =  gunn_data["p"]

        po_x = float(ais_data_item["po_x"])
        po_y = float(ais_data_item["po_y"])
        uo_x = float(ais_data_item["uo_x"])
        uo_y = float(ais_data_item["uo_y"])  

        urx = uo_x - ux
        ury = uo_y - uy
        ur = math.sqrt(urx**2 + ury**2)

        if np.isclose(ur, 0):
            d_at_cpa = math.sqrt((p[0][0]-po_x)**2 + (p[1][0]-po_y)**2)
            t_2_cpa = 0
        else: 
            d_at_cpa = abs((po_x*ury - po_y*urx)/ur)
            t_2_cpa = -(po_x*urx + po_y*ury) / ur**2

        #self coords at cpa
        x_at_cpa = p[0][0] + ux * t_2_cpa
        y_at_cpa = p[1][0] + uy * t_2_cpa

        # target coords at cpa
        o_x_at_cpa = po_x + t_2_cpa* uo_x
        o_y_at_cpa = po_y + t_2_cpa* uo_y

        cpa = {
            "d_at_cpa": d_at_cpa,
            "t_2_cpa": t_2_cpa,
            "x_at_cpa": x_at_cpa,
            "y_at_cpa": y_at_cpa,
            "o_x_at_cpa": o_x_at_cpa,
            "o_y_at_cpa": o_y_at_cpa,
            }
        return cpa

    # get safety params for an ais vessel, ais has to be pre processed
    def _get_safety_params(self, gunn_data, ais_data_item):
        ux = float(gunn_data["ux"])
        uy = float(gunn_data["uy"])
        p =  gunn_data["p"]

        po_x = float(ais_data_item["po_x"])
        po_y = float(ais_data_item["po_y"])
        uo_x = float(ais_data_item["uo_x"])
        uo_y = float(ais_data_item["uo_y"])  

        urx = uo_x - ux
        ury = uo_y - uy
        ur = math.sqrt(urx**2 + ury**2)

        if np.isclose(ur, 0):
            t_2_r = 0
        else:
        # algebraic solution for time to safety radius, trust me ;)
            t_2_r_a = (- self._safety_radius_m**2 + po_x**2 + po_y**2)/(math.sqrt(self._safety_radius_m**2*uo_x**2 -
                    2*self._safety_radius_m**2*uo_x*ux + self._safety_radius_m**2*uo_y**2 - 
                    2*self._safety_radius_m**2*uo_y*uy + self._safety_radius_m**2*ux**2 + 
                    self._safety_radius_m**2*uy**2 - uo_x**2*po_y**2 + 2*uo_x*uo_y*po_x*po_y +
                    2*uo_x*ux*po_y**2 - 2*uo_x*uy*po_x*po_y - uo_y**2*po_x**2 - 
                    2*uo_y*ux*po_x*po_y + 2*uo_y*uy*po_x**2 - ux**2*po_y**2 + 
                    2*ux*uy*po_x*po_y - uy**2*po_x**2) - uo_x*po_x + ux*po_x -
                    po_y*(uo_y - uy))
            
            t_2_r_b = -(- self._safety_radius_m**2 + po_x**2 + po_y**2)/(math.sqrt(self._safety_radius_m**2*uo_x**2 - 
                    2*self._safety_radius_m**2*uo_x*ux + self._safety_radius_m**2*uo_y**2 - 
                    2*self._safety_radius_m**2*uo_y*uy + self._safety_radius_m**2*ux**2 + 
                    self._safety_radius_m**2*uy**2 - uo_x**2*po_y**2 + 2*uo_x*uo_y*po_x*po_y +
                    2*uo_x*ux*po_y**2 - 2*uo_x*uy*po_x*po_y - uo_y**2*po_x**2 - 
                    2*uo_y*ux*po_x*po_y + 2*uo_y*uy*po_x**2 - ux**2*po_y**2 + 
                    2*ux*uy*po_x*po_y - uy**2*po_x**2) + uo_x*po_x - ux*po_x +
                    po_y*(uo_y - uy))
            
            if t_2_r_a > t_2_r_b:
                t_2_r = t_2_r_b
            else: 
                t_2_r = t_2_r_a

        # target coords at dq
        t_x_at_r = po_x + t_2_r*uo_x
        t_y_at_r = po_y + t_2_r*uo_y

        # self coords at dq
        x_at_r = p[0][0] + t_2_r*ux
        y_at_r = p[1][0] + t_2_r*uy

        safety_params = {
            "t_2_r": t_2_r,
            "t_x_at_r": t_x_at_r,
            "t_y_at_r": t_y_at_r,
            "x_at_r": x_at_r,
            "y_at_r": y_at_r
        }

        return safety_params

    def _process_data(self):
        processed_data = []
        gunn_data = self._get_gunnerus_data()
        if gunn_data is None: return None

        ais_data = copy.deepcopy(self._ais_data)

        if (self.dummy_vessel is not None):
            ais_data['dummy'] =  self.dummy_vessel
        
        ais_msg_ids = ais_data.keys()

        print(gunn_data)

        for ais_id in ais_msg_ids:
            ais_message = ais_data[ais_id]

            if str(ais_message['mmsi']) == self._gunnerus_mmsi: continue

            ais_data_item = self._get_ais_data(ais_message, gunn_data)

            if ais_data_item is None: continue

            cpa = self._get_cpa(gunn_data, ais_data_item)
            is_within_safety_distance = cpa['d_at_cpa'] <= (self._safety_radius_m * self._safety_radius_tol)
            
            if  is_within_safety_distance and cpa['t_2_cpa'] >= 0:
                ais_data_item["cpa"] = cpa
                ais_data_item["safety_params"] = self._get_safety_params(gunn_data, ais_data_item)
                processed_data.append(ais_data_item)

        return gunn_data, processed_data

    def get_ARPA_parameters(self):
        processed_data, gunn_data = self._process_data()
        