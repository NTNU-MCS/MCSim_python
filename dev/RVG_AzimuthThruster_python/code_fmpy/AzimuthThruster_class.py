import numpy as np

class AzimuthThruster():

    def __init__(self, h):

        self.h = h
        self.globaltime = 0.0
        self.ly = 2.7
        self.lx = 13.22

    def Force(self):

        # constant values
        self.fmu.setReal([self.input_x_rel_ap], [0.0])
        self.fmu.setReal([self.input_y_rel_cl], [self.ly])
        self.fmu.setReal([self.input_z_rel_bl], [0.55])
        self.fmu.setReal([self.input_cg_x_rel_ap], [self.lx])
        self.fmu.setReal([self.input_cg_y_rel_cl], [0.0])
        self.fmu.setReal([self.input_cg_z_rel_bl], [3.624001])
        self.fmu.setReal([self.input_prop_diam], [1.9])
        self.fmu.setReal([self.input_distancetohull], [1.5])
        self.fmu.setReal([self.input_bilgeradius], [3.0])
        self.fmu.setReal([self.input_rho], [1025])
        self.fmu.setReal([self.input_lpp], [28.9])

        # input
        self.fmu.setReal([self.input_act_revs], [self.revs])
        self.fmu.setReal([self.input_act_angle], [self.angle])
        self.fmu.setReal([self.input_cg_surge_vel], [self.u])
        self.fmu.setReal([self.input_cg_sway_vel], [self.v])
        self.fmu.setReal([self.input_yaw_vel], [self.rdeg])

        self.fmu.doStep(currentCommunicationPoint=self.globaltime, communicationStepSize=self.h)

        self.globaltime = self.globaltime + self.h

        self.Fx, self.Fy = self.fmu.getReal([self.output_force_surge, self.output_force_sway])

    def fmusetparameters(self):

        self.input_x_rel_ap = self.vrs['input_x_rel_ap']
        self.input_y_rel_cl = self.vrs['input_y_rel_cl']
        self.input_z_rel_bl = self.vrs['input_z_rel_bl']
        self.input_act_revs = self.vrs['input_act_revs']
        self.input_act_angle = self.vrs['input_act_angle']
        self.input_cg_x_rel_ap = self.vrs['input_cg_x_rel_ap']
        self.input_cg_y_rel_cl = self.vrs['input_cg_y_rel_cl']
        self.input_cg_z_rel_bl = self.vrs['input_cg_z_rel_bl']
        self.input_cg_surge_vel = self.vrs['input_cg_surge_vel']
        self.input_cg_sway_vel = self.vrs['input_cg_sway_vel']
        self.input_yaw_vel = self.vrs['input_yaw_vel']

        self.input_prop_diam = self.vrs['input_prop_diam']
        self.input_bilgeradius = self.vrs['input_bilgeradius']
        self.input_distancetohull = self.vrs['input_distancetohull']
        self.input_rho = self.vrs['input_rho']
        self.input_lpp = self.vrs['input_lpp']

        self.output_force_surge = self.vrs['output_force_surge']
        self.output_force_sway = self.vrs['output_force_sway']

    def simulate(self, fmu, vrs, u, v, rdeg, revs, angle):

        self.fmu = fmu
        self.vrs = vrs
        self.u = u
        self.v = v
        self.rdeg = rdeg
        self.angle = angle
        self.revs = revs

        self.fmusetparameters()

        self.Force()