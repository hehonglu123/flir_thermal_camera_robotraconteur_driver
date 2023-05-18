# Example demonstrating using camera parameters

from RobotRaconteur.Client import *
import numpy as np
import matplotlib.pyplot as plt

url='rr+tcp://127.0.0.1:60827/?service=camera'

c1=RRN.ConnectService(url)

param_names = ["object_emissivity", "object_distance", "reflected_temperature", "atmospheric_temperature", 
               "relative_humidity", "estimated_transmission", "ext_optics_temperature", "ext_optics_transmission",
               "focus_pos", "fps", "ir_format"]

for param_name in param_names:
    param_val = c1.getf_param(param_name)
    if param_val.datatype != "string":
        print(f"{param_name}: {param_val.data[0]}")
    else:
        print(f"{param_name}: {param_val.data}")

# Set to test parameters
c1.setf_param("object_emissivity", RR.VarValue(0.90,"double"))
c1.setf_param("object_distance", RR.VarValue(1.12,"double"))
c1.setf_param("reflected_temperature", RR.VarValue(291.15,"double"))
c1.setf_param("atmospheric_temperature", RR.VarValue(293.15,"double"))
c1.setf_param("relative_humidity", RR.VarValue(0.6,"double"))
c1.setf_param("estimated_transmission", RR.VarValue(0.99,"double"))
c1.setf_param("ext_optics_temperature", RR.VarValue(292.15,"double"))
c1.setf_param("ext_optics_transmission", RR.VarValue(0.97,"double"))
c1.setf_param("focus_pos", RR.VarValue(1500,"int32"))
c1.setf_param("fps", RR.VarValue(15,"double"))
c1.setf_param("ir_format", RR.VarValue("temperature_linear_100mK","string"))
# c1.setf_param("ir_format", RR.VarValue("radiometric","string"))

# Set the calibration to third option
#c1.setf_param("current_case", RR.VarValue(2,"int32"))