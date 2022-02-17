import TunnelThruster_class
import numpy as np
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

from scipy import optimize

import fmpy
from fmpy import *
from fmpy.fmi1 import FMU1Slave
from fmpy.util import plot_result, download_test_file

def fmuinitialize():

    fmu_filename = '../fmus/TunnelThruster_modified.fmu'

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
    ncooldown = 10
    h = 1.0

    # the range of approximation
    revs_min = -200
    revs_max = 200

    revs_list = np.linspace(revs_min, revs_max, 1000)
    result_array = np.zeros((len(revs_list), 2))

    for irevs, revs in enumerate(revs_list):

        model = TunnelThruster_class.TunnelThruster(fmu = fmu,
                                                    vrs = vrs,
                                                    h=h,
                                                    v=0.0,
                                                    revs=revs,
                                                    globaltime=globaltime,
                                                    ncooldown=ncooldown)

        model.simulate()
        globaltime = model.globaltime
        result_array[irevs, 0] = revs
        result_array[irevs, 1] = model.Fy

    REV = result_array[:, 0]

    X = np.concatenate([REV[:, np.newaxis]**3,
                        REV[:, np.newaxis]**2,
                        REV[:, np.newaxis]],
                        axis=1)

    regressor = LinearRegression(fit_intercept=False)
    regressor.fit(X, result_array[:, 1])

    print('coefficients')
    print(regressor.coef_)

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111)
    ax.scatter(result_array[:, 0], result_array[:, 1])
    ax.plot(result_array[:, 0], regressor.predict(X), c='red')
    ax.set_xlabel('revs')
    ax.set_ylabel('force')
    plt.show()

