Contact: motoyasu.kanazawa@ntnu.no.

Verification has been conducted with two azimuth thrusters
by comparing a Python code result with a Vico result.
For verification, run main.py.

test_wo_wind.py compares a Python code with a Vico result in Data_vico with no wind.
test_with_wind.py compares a Python code with a Vico result in Data_vico with wind.

Note that
- Tunnel thruster verification has not been conducted.
- For wind forces, we still see some discrepancy in yaw motion.