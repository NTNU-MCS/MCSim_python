import numpy as np
import math

class vesselmodel():

    """
    Vessel model for non-elongated Gunnerus. Validated in the simval project.
    """

    def __init__(self, h):

        self.h = h

        """
        Parameters
        ------------
        m : float (kg)
            Ship mass
        x_g : float (m) 
            Position of gravity centre in the x direction of the body-fixed coordinate system.
        L_pp : float (m)
            Length between perpendiculars
        I_z : float (kgm2)
            The inertia moment about the vertical axis.
        dens : float (kg/m3)
            The density of water
        ly : float (m)
            absolute y position of the azimuth thrusters from the center line.
        lx : float (m)
            longitudinal distance from the azimuth thrusters to the center of gravity.
        bow_x_rel_ap : float (m)
            longitudinal distance from the bow thruster to A.P.
        """

        self.m = 0.41093181E+06
        self.x_g = 0.0
        self.L_pp = 28.9
        self.I_z = 0.21450896E+08
        self.dens = 1025
        self.ly = 2.7
        self.lx = 13.22
        self.bow_x_rel_ap = 31.5

        Xudot = -79320.5895
        Yvdot = -408192.6004
        Yrdot = 331671.6384
        Nvdot = 331671.6384
        Nrdot = -12245783.9277

        Xuu = -2100
        Xvv = 2880.5065
        Xrr = -29267.3371
        Xrv = -323707.1228
        Xuvv = -2371.1792
        Xrvu = 33926.4945
        Xurr = 83774.6548

        Yuv = -31000.2779
        Yur = 277765.7816
        Yuur = 11112.4159
        Yuuv = 777.3214
        Yvvv = -11198.6727
        Yrrr = 21117955.8908
        Yrrv = -12912371.0982
        Yvvr = 102679.0573

        Nuv = 309051.5592
        Nur = -2413124.664
        Nuur = -187538.9302
        Nuuv = -14041.6851
        Nvvv = -56669.589
        Nrrr = -179720233.7453
        Nrrv = -585678.1567
        Nvvr = -4278941.6707

        Xumodv = 619.643
        Ymodrv = 485260.3423
        Ymodvv = -21640.6686
        Ymodrr = -1020386.1325
        Ymodvr = 321944.5769
        Nmodrv = 1626540.037
        Nmodvv = 116643.7578
        Nmodrr = -9397835.7612
        Nmodvr = -2258197.6812

        self.M = np.array([[self.m-Xudot,                     0,                     0],
                           [           0,          self.m-Yvdot, self.m*self.x_g-Yrdot],
                           [           0, self.m*self.x_g-Nvdot,        self.I_z-Nrdot]])

        self.X_coeff = [Yvdot, 0.5*(Nvdot+Yrdot), Xuu, 0.0, Xrvu, Xvv, Xrv, Xuvv, Xrr, Xurr, Xumodv]
        self.Y_coeff = [Xudot, Yuv, Yur, Yuur, Yuuv, Yvvv, Yrrr, Yrrv, Yvvr, Ymodrv, Ymodvv, Ymodvr, Ymodrr]
        self.N_coeff = [(Yvdot-Xudot), 0.5*(Nvdot+Yrdot), Nuv, Nur, Nuur, Nuuv, Nvvv, Nrrr, Nrrv, Nvvr, Nmodvv, Nmodvr, Nmodrv, Nmodrr]

    def RBCC_Forces(self, u, v, r):

        CRB = np.array([[                0, -self.m * r, -self.m * self.x_g * r],
                        [       self.m * r,           0,                      0],
                        [self.m * self.x_g * r,       0,                      0]])

        F_RBCC = - CRB @ np.array([u, v, r])

        return - np.array([[F_RBCC[0], F_RBCC[1], F_RBCC[2]]]).T

    def HydrodynamicForces(self, u, v, r):

        """
        The equation of hydrodynamic coefficients
        refers to "Identification of nonlinear manoeuvring models for marine vessels using planar motion mechanism test (OMAE2015.41789).
        Note that Xuuu is not given by vesselFMU and it is set to zero below.
        """

        X_h = np.dot(self.X_coeff, np.array([v*r,
                                             r*r,
                                             u*u,
                                             u*u*u,
                                             r*v*u,
                                             v*v,
                                             r*v,
                                             u*v*v,
                                             r*r,
                                             u*r*r,
                                             u*abs(v)]))

        Y_h = np.dot(self.Y_coeff, np.array([u*r,
                                             u*v,
                                             u*r,
                                             u*u*r,
                                             u*u*v,
                                             v*v*v,
                                             r*r*r,
                                             r*r*v,
                                             v*v*r,
                                             v*abs(r),
                                             v*abs(v),
                                             r*abs(v),
                                             r*abs(r)]))

        N_h = np.dot(self.N_coeff, np.array([v*u,
                                             r*u,
                                             u*v,
                                             u*r,
                                             u*u*r,
                                             u*u*v,
                                             v*v*v,
                                             r*r*r,
                                             r*r*v,
                                             v*v*r,
                                             v*abs(v),
                                             r*abs(v),
                                             v*abs(r),
                                             r*abs(r)]))

        return np.array([[X_h, Y_h, N_h]]).T

    def WindForces(self, psi):

        A_L = 172 # ship side projection area (m2)
        A_F = 81 # ship front projection area (m2)

        u_wr = self.V_w * math.cos(self.beta_w + np.pi - psi)
        v_wr = self.V_w * math.sin(self.beta_w + np.pi - psi)

        Ua = (u_wr ** 2 + v_wr ** 2) ** 0.5
        epsilon = math.atan2(v_wr, u_wr)

        dir = np.arange(0.0, 181.0, 10.0) * np.pi
        CX = np.array([-0.53, -0.59, -0.65, -0.59, -0.51, -0.47, -0.4, -0.29, -0.2, -0.18, -0.14, -0.05, 0.12, 0.37, 0.61, 0.82, 0.86, 0.72, 0.62])
        CY = np.array([0, 0.22, 0.4, 0.66, 0.83, 0.9, 0.88, 0.87, 0.86, 0.85, 0.83, 0.82, 0.81, 0.73, 0.58, 0.46, 0.26, 0.09, 0])
        CN = np.array([0, 0.05, 0.1, 0.135, 0.149, 0.148, 0.114, 0.093, 0.075, 0.04, 0.02, -0.013, -0.035, -0.041, -0.045, -0.04, -0.029, -0.014, 0])

        X = np.interp(abs(epsilon), dir, CX) * 1.23 / 2 * Ua ** 2 * A_F
        Y = -np.copysign(1, epsilon) * np.interp(abs(epsilon), dir, CY) * 1.23 / 2 * Ua ** 2 * A_L
        N = -np.copysign(1, epsilon) * np.interp(abs(epsilon), dir, CN) * 1.23 / 2 * Ua ** 2 * A_L * self.L_pp

        return np.array([[X, Y, N]]).T

    def ForceAzimuth(self, u, v, r, revs, angle, ly):

        lx = 13.22
        l = np.sqrt(self.lx ** 2 + ly ** 2)
        sint = ly / l
        cost = self.lx / l

        # constant values
        self.fmu.setReal([self.input_x_rel_ap], [0.0])
        self.fmu.setReal([self.input_y_rel_cl], [ly])
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
        self.fmu.setReal([self.input_act_revs], [revs])
        self.fmu.setReal([self.input_act_angle], [angle])
        self.fmu.setReal([self.input_cg_surge_vel], [u])
        self.fmu.setReal([self.input_cg_sway_vel], [v])
        self.fmu.setReal([self.input_yaw_vel], [np.rad2deg(r)])

        self.fmu.doStep(currentCommunicationPoint=self.globaltime, communicationStepSize=self.h)

        self.globaltime = self.globaltime + self.h

        Fx, Fy = self.fmu.getReal([self.output_force_surge, self.output_force_sway])
        Nz = - l * (Fx * sint + Fy * cost)

        return np.array([[Fx, Fy, Nz]]).T

    def ForceTunnel(self):

        Fx = 0.0
        Fy = - (3.15982607e-04*self.tun_revs**3 + 1.23363612e-03*self.tun_revs**2 + 5.17642375e+00*self.tun_revs)
        Nz = Fy * (self.bow_x_rel_ap - self.lx) # Bow thruster is located in the bow. If Fy is positive, Nz should be positive.

        return np.array([[Fx, Fy, Nz]]).T

    def odefun(self):

        psi = self.x0[2]
        u = self.x0[3]
        v = self.x0[4]
        r = self.x0[5]

        res = np.zeros(6)

        res[0:3] = np.array([self.x0[3]*math.cos(self.x0[2])-self.x0[4]*math.sin(self.x0[2]),
                             self.x0[3]*math.sin(self.x0[2])+self.x0[4]*math.cos(self.x0[2]),
                             self.x0[5]]).T

        F_h = self.HydrodynamicForces(u=u, v=v, r=r)
        F_RBCC = self.RBCC_Forces(u=u, v=v, r=r)
        F_w = self.WindForces(psi=psi)
        F_AzimuthP = self.ForceAzimuth(u=u, v=v, r=r, revs=self.p_revs, angle=self.p_angle, ly=self.ly)
        F_AzimuthS = self.ForceAzimuth(u=u, v=v, r=r, revs=self.s_revs, angle=self.s_angle, ly=-self.ly)
        F_Tunnel = self.ForceTunnel()

        F_total = F_h + F_w + F_RBCC + F_AzimuthP + F_AzimuthS + F_Tunnel

        res[3:6] = (np.linalg.inv(self.M) @ F_total).flatten()

        return res

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

    def simulate(self,
                 x0,
                 globaltime,
                 fmu,
                 vrs,
                 p_angle,
                 p_revs,
                 s_angle,
                 s_revs,
                 tun_revs,
                 V_w,
                 beta_w):

        self.x0 = x0
        self.globaltime = globaltime
        self.fmu = fmu
        self.vrs = vrs
        self.p_angle = p_angle
        self.p_revs = p_revs
        self.s_angle = s_angle
        self.s_revs = s_revs
        self.tun_revs = tun_revs
        self.V_w = V_w
        self.beta_w = beta_w

        self.fmusetparameters()

        acc = self.odefun()
        x = self.x0 + self.h * acc

        return x, self.globaltime