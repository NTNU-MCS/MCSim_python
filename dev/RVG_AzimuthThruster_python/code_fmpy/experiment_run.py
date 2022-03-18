from AzimuthThruster_class import AzimuthThruster
import numpy as np
import matplotlib.pyplot as plt
import itertools
from fmpy import *
from fmpy.fmi1 import FMU1Slave

def fmuinitialize():

    fmu_filename = '../../../models/RVG_AzimuthThruster/PMAzimuth.fmu'

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

    fmu, vrs = fmuinitialize()
    globaltime = 0.0
    h = 1.0

    u_list = [5.0, 6.0]
    v_list = [0.0, 1.0, 2.0]
    rdeg_list = [0.0, 2.0, 4.0]
    angle_list = [0.0, 10.0, 20.0]
    revs_list = [165, 200]

    iter = itertools.product(u_list,
                             v_list,
                             rdeg_list,
                             angle_list,
                             revs_list)

    log_s = 'output.csv'
    flog = open(log_s, 'w')
    flog.write('u(m/s),v(m/s),r(deg/s),angle(deg),revs(RPM),Fx(N),Fy(N)\n')

    for u, v, rdeg, angle, revs in iter:

        model = AzimuthThruster(h=h)
        model.simulate(fmu=fmu,
                       vrs=vrs,
                       u=u,
                       v=v,
                       rdeg=rdeg,
                       revs=revs,
                       angle=angle)

        flog.write('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n' % (u,
                                                             v,
                                                             rdeg,
                                                             angle,
                                                             revs,
                                                             model.Fx,
                                                             model.Fy))

    flog.close()