Identified tunnel thruster model:
Force = 3.15982607e-04  * rev**3 + 1.60941887e-03 * rev**2 + 5.17642375e+00 * rev

Note that this approximation assumes
- Thruster force is determined only be thruster revolution
- Ship velocities are assumed to be zero
- If one wants to re-identify the approximation with different assumption,
- one can re-run code_fmpy/experiment_run.py
- "experiment_run.py" runs "fmus/ThunnelThruster_modified.fmu" since the original FMU does not work
- due to having some problems in modelDescription.xml for fmpy.

Contact: motoyasu.kanazawa@ntnu.no