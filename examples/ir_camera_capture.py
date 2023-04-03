# Capture and display a single frame

from RobotRaconteur.Client import *
import numpy as np
import matplotlib.pyplot as plt

url='rr+tcp://127.0.0.1:60827/?service=camera'

c1=RRN.ConnectService(url)
image_consts = RRN.GetConstants('com.robotraconteur.image', c1)

rr_img = c1.capture_frame()
if rr_img.image_info.encoding == image_consts["ImageEncoding"]["mono8"]:
    # Simple uint8 image
    mat = rr_img.data.reshape([rr_img.image_info.height, rr_img.image_info.width], order='C')
elif rr_img.image_info.encoding == image_consts["ImageEncoding"]["mono16"]:
    data_u16 = np.array(rr_img.data.view(np.uint16))
    mat = data_u16.reshape([rr_img.image_info.height, rr_img.image_info.width], order='C')

fig = plt.figure(1)

ir_format = rr_img.image_info.extended["ir_format"].data

if ir_format == "temperature_linear_10mK":
    display_mat = (mat * 0.01) - 273.15    
elif ir_format == "temperature_linear_100mK":
    display_mat = (mat * 0.1) - 273.15    
else:
    display_mat = mat
plt.imshow(display_mat, cmap='inferno', aspect='auto')
plt.colorbar(format='%.2f')
plt.show()
