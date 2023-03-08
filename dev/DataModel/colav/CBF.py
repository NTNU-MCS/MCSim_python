import math
import numpy as np
import time  
from utils.Point import Point

class CBF:
    def __init__(self) -> None:
        self.p = np.mat([],[])
        self.S = np.mat('0 -1; 1 0')
        self.yaw = np.mat([])
        
        pass

    def R(self,z):
        Sz = self.S @ z
        _R = np.array()
        return  
