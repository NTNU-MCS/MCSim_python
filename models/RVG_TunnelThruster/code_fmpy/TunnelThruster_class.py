import numpy as np
import math
import matplotlib.pyplot as plt
import shutil

import fmpy
from fmpy import *
from fmpy.fmi1 import FMU1Slave
from fmpy.util import plot_result, download_test_file

class TunnelThruster():

    def __init__(self, fmu, vrs, globaltime, ncooldown, v, revs, h):

        self.fmu = fmu
        self.vrs = vrs
        self.globaltime = globaltime
        self.ncooldown = ncooldown
        self.v = v
        self.revs = revs
        self.h = h

    def Force(self):

        # constant values
        self.fmu.setReal([self.input_x_rel_ap], [26.3])
        self.fmu.setReal([self.input_y_rel_cl], [0])
        self.fmu.setReal([self.input_z_rel_bl], [1.637])
        self.fmu.setReal([self.input_prop_diam], [1.0])
        self.fmu.setReal([self.input_max_pitch], [1.0])
        self.fmu.setReal([self.input_rho], [1025])
        self.fmu.setReal([self.input_lpp], [28.9])

        # input
        self.fmu.setReal([self.input_act_revs], [self.revs])
        self.fmu.setReal([self.input_cg_surge_vel], [0.0])
        self.fmu.setReal([self.input_cg_sway_vel], [self.v])
        self.fmu.setReal([self.input_yaw_vel], [0.0])

        self.fmu.doStep(currentCommunicationPoint=self.globaltime, communicationStepSize=self.h)

        self.globaltime = self.globaltime + self.h

        self.Fx, self.Fy = self.fmu.getReal([self.output_force_surge, self.output_force_sway])

    def fmusetparameters(self):

        self.input_x_rel_ap = self.vrs['input_x_rel_ap']
        self.input_y_rel_cl = self.vrs['input_y_rel_cl']
        self.input_z_rel_bl = self.vrs['input_z_rel_bl']
        self.input_prop_diam = self.vrs['input_prop_diam']
        self.input_max_pitch = self.vrs['input_max_pitch']
        self.input_rho = self.vrs['input_rho']
        self.input_lpp = self.vrs['input_lpp']

        self.input_act_revs = self.vrs['input_act_revs']
        self.input_cg_surge_vel = self.vrs['input_cg_surge_vel']
        self.input_cg_sway_vel = self.vrs['input_cg_sway_vel']
        self.input_yaw_vel = self.vrs['input_yaw_vel']

        self.output_force_surge = self.vrs['output_force_surge']
        self.output_force_sway = self.vrs['output_force_sway']

    def fmucooldown(self):

        for icooldown in range(self.ncooldown):

            # constant values
            self.fmu.setReal([self.input_x_rel_ap], [26.3])
            self.fmu.setReal([self.input_y_rel_cl], [0])
            self.fmu.setReal([self.input_z_rel_bl], [1.637])
            self.fmu.setReal([self.input_prop_diam], [1.0])
            self.fmu.setReal([self.input_max_pitch], [1.0])
            self.fmu.setReal([self.input_rho], [1025])
            self.fmu.setReal([self.input_lpp], [28.9])

            # input
            self.fmu.setReal([self.input_act_revs], [0.0])
            self.fmu.setReal([self.input_cg_surge_vel], [0.0])
            self.fmu.setReal([self.input_cg_sway_vel], [0.0])
            self.fmu.setReal([self.input_yaw_vel], [0.0])

            self.fmu.doStep(currentCommunicationPoint=self.globaltime, communicationStepSize=self.h)

            self.globaltime = self.globaltime + self.h

    def simulate(self, *args, **kwargs):

        self.fmusetparameters()

        self.Force()

        self.fmucooldown()