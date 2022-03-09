import vesselmodel
import numpy as np
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from fmpy import *
from fmpy.fmi1 import FMU1Slave

def fmuinitialize():

    fmu_filename = 'fmus/PMAzimuth.fmu'

    model_description = read_model_description(fmu_filename)

    vrs = {}
    for variable in model_description.modelVariables:
        vrs[variable.name] = variable.valueReference

    unzipdir = extract(fmu_filename)

    fmu = FMU1Slave(guid=model_description.guid,
                         unzipDirectory=unzipdir,
                         modelIdentifier=model_description.coSimulation.modelIdentifier,
                         instanceName='instance1')

    fmu.instantiate()

    return fmu, vrs

if __name__ == '__main__':

    """
    Invoke a class "vesselmodel" to calculate vessel states in the future.
    
    Parameters
    ------------
    h : float (s) 
        Time step for numerical integration.
    N0: float (m)
        Initial north position.
    E0: float (m)
        Initial east position. 
    psi0: float (rad)
        Initial heading to north.
    u0: float (m/s)
        Initial surge velocity.
    v0: float (m/s)
        Initial sway velocity.
    r0: float (rad/s)
        Initial yaw velocity.
    nph: int (-)
        The number of time steps for numerical integration.
    p,s_revs: float array - length nph (rpm)
        P/S azimuth thruster revolution over the prediction horizon.
        Must be in the range of -203 (rpm) - 203 (rpm)
    p,s_angle: float array - length nph (deg)
        P/S Azimuth thruster angle over the prediction horizon.
    tun_revs: float array - length nph (rpm)
        Tunnel thruster revolution over the prediction horizon.
        Must be in the range of -203 (rpm) - 203 (rpm)
    V_w: float (m/s)
        Global wind speed. Assumed no change over the prediction horizon.
    beta_w: float (rad)
        Global wind direction to north. Assumed no change over the prediction horizon.
        
    Outputs
    ------------
    Log_states: float array - size (nph, 6). (m, m, rad, m/s, m/s, rad/s)
        Six states over the prediction horizon.
    """

    # Vico result: calculated by NTNU Ålesund with same conditions.
    np_load = np.load('Data_vico/turn_angle20_revs100.npz')
    N_vico = np_load['N']
    E_vico = np_load['E']
    psi_vico = np_load['heading']
    u_vico = np_load['surge_vel']
    v_vico = np_load['sway_vel']
    rdeg_vico = np_load['yaw_vel']

    h = 1.0
    nph = 30
    N0 = N_vico[0]
    E0 = E_vico[0]
    psi0 = np.deg2rad(psi_vico[0])
    u0 = u_vico[0]
    v0 = v_vico[0]
    r0 = np.deg2rad(rdeg_vico[0])
    revs0 = 100.0
    angle0 = 20.0
    p_revs = np.ones(nph) * revs0
    p_angle = np.ones(nph) * angle0
    s_revs = p_revs
    s_angle = p_angle
    tun_revs = np.zeros(nph)
    V_w = 0.0
    beta_w = 0.0
    Log_states = np.zeros((nph, 6))

    fmu, vrs = fmuinitialize()
    globaltime = 0.0

    # initial states
    x = np.array([N0,
                  E0,
                  psi0,
                  u0,
                  v0,
                  r0])

    model = vesselmodel.vesselmodel(h=h)

    for iph in range(nph):

        x, globaltime = model.simulate(fmu=fmu,
                                       vrs=vrs,
                                       p_revs=p_revs[iph],
                                       p_angle=p_angle[iph],
                                       s_revs=s_revs[iph],
                                       s_angle=s_angle[iph],
                                       tun_revs=tun_revs[iph],
                                       V_w=V_w,
                                       beta_w=beta_w,
                                       globaltime=globaltime,
                                       x0=x)

        Log_states[iph, :] = x

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(Log_states[:, 1], Log_states[:, 0], color='black', label='python')
    ax.plot(E_vico[1:nph+1], N_vico[1:nph+1], color='blue', label='vico')
    ax.scatter(Log_states[0, 1], Log_states[0, 0], color='red', label='Initial position')
    ax.set_xlabel('East(m)')
    ax.set_ylabel('North(m)')
    ax.set_aspect('equal')
    ax.set_title('Trajectory')
    plt.legend()
    plt.show()

    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(311)
    ax.plot(Log_states[:, 3], color='black', label='Python')
    ax.plot(u_vico[1:nph+1], color='blue', label='vico')
    ax.set_ylabel('u(m/s)')
    plt.legend()
    ax = fig.add_subplot(312)
    ax.plot(Log_states[:, 4], color='black', label='Python')
    ax.plot(v_vico[1:nph+1], color='blue', label='vico')
    ax.set_ylabel('v(m/s)')
    plt.legend()
    ax = fig.add_subplot(313)
    ax.plot(np.rad2deg(Log_states[:, 5]), color='black', label='Python')
    ax.plot(rdeg_vico[1:nph+1], color='blue', label='vico')
    ax.set_ylabel('r(deg/s)')
    ax.set_xlabel('Time(s)')
    plt.legend()
    plt.show()