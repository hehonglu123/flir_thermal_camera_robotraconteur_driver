# FLIR Thermal Camera Robot Raconteur Driver

This repository contains a Robot Raconteur driver for FLIR thermal cameras using the Python Spinaker SDK. This
driver uses the standard Robot Raconteur `com.robotraconteur.imaging.Camera` interface. The driver should be compatible
with most FLIR cameras supported by the Spinaker SDK, but has only been tested on the FLIR ThermoVision A320 camera.
This camera is a 320x240 pixel camera with a 30 Hz frame rate. This is an older camera, but newer cameras should work
but may need some tweaking.

## Installation

Python needs to be installed. For the older version of the Spinaker SDK described below, Python version 3.8 is required.
It is recommended that Python 3.8 be installed to `C:\Python38` on Windows. The Python installation must be 64 bit.

The FLIR Spinaker Full SDK  and the Python Spinaker SDK must be installed. The ThermoVision A320 camera requires an 
older version of the Spinaker SDK. Version `2.7.0.128` is known to work. The SDK can be downloaded from the FLIR 
website after registration. Both the SDK and the Python wrapper must be installed. Install the Spinaker SDK first 
to the default location. Extract the Python SDK zip file to a directory of your choice.

Install the Python Spinaker SDK from the wheel file extracted from the zip:

```
c:\python38\python -m pip install --user <spinaker python directory>/spinnaker_python-2.7.0.128-cp38-cp38-win_amd64.whl
```

Now install the driver from GitHub using pip:

```
c:\python38\python -m pip install --user git+https://github.com/hehonglu123/flir_thermal_camera_robotraconteur_driver.git
```

The driver requires a configuration file to be specified at the command line. Download the A320 config file
using curl:

```
curl -L -o flir_thermovision_a320_default_config.yml https://raw.githubusercontent.com/robotraconteur-contrib/flir_thermal_camera_robotraconteur_driver/main/config/flir_thermovision_a320_default_config.yml
```

Other cameras can be defined by modifying the contents of the config file. The config file is a YAML file. See the
documentation for the camera standard type for more information.

## Running the driver

The driver can be run from the command line using the following command:

```
c:\python38\python -m flir_thermal_camera_robotraconteur_driver --config-file=flir_thermovision_a320_default_config.yml
```

By default the driver can be connected using the following url: `rr+tcp://127.0.0.1:60827/?service=camera`.

The standard Robot Raconteur command line configuration flags are supported. See 
https://github.com/robotraconteur/robotraconteur/wiki/Command-Line-Options

## Driver Clients

The driver implements a standard Robot Raconteur `com.robotraconteur.imaging.Camera` interface. The main difference
from a typical webcam is that the image format is by default `mono16`, and of course the output is a thermal image
rather than color or monochrome. The thermal camera also supplies different parameters compared to a normal
camera. These are configured using the `getf_param()` and `setf_param()` functions. See the Camera Parameters
for more information on the parameters.

## Examples

There are several examples in the `examples` directory. The simplest example is the single frame capture:

```python
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
```

## Camera Parameters

The configuration parameters for a thermal camera can be quite complex. Converting between the raw camera data and
a temperature is quite complicated, and requires knowing many parameters about the camera itself and the environment.
On newer cameras, all of the calibration parameters required to do this conversion are available to be read off 
the camera. A full explanation of the equations used by FLIR can be found here: 
https://flir.custhelp.com/app/answers/detail/a_id/3321/~/the-measurement-formula#:~:text=How%20does%20the%20camera%20measure,converted%20in%20to%20temperature%20values
 . An example of using calibration parameters read off the camera and converted to temperature can be found here:
 https://flir.custhelp.com/app/answers/detail/a_id/4186/~/using-spinnaker-sdk-to-connect-to-a-flir-a50%2Fa70-or-a400%2Fa500%2Fa700-image
  . Unfortunately the A320 does not provide these calibration parameters, and instead does the conversion from
raw data to temperature on the camera itself. This means that the camera must be configured with the correct
parameters for the environment it is being used in.

The Robot Raconteur FLIR driver currently supports the following parameters:

| Parameter | R/W | Data Type | Description |
| --- | --- | --- | --- |
| `object_emissivity` | R/W | `double` | The emissivity of the object being imaged. This is a value between 0 and 1. |
| `object_distance` | R/W | `double` | The distance to the object being imaged in meters. |
| `reflected_temperature` | R/W | `double` | The ambient reflected temperature of the reflected environment in Kelvin. |
| `atmospheric_temperature` | R/W | `double` | The ambient atmospheric temperature of the environment in Kelvin. |
| `relative_humidity` | R/W | `double` | The relative humidity of the environment in percent. |
| `estimated_transmission` | R/W | `double` | The estimated transmission of the atmosphere. This is a value between 0 and 1. |
| `ext_optics_temperature` | R/W | `double` | The temperature of the external optics in Kelvin. |
| `ext_optics_transmission` | R/W | `double` | The transmission of the external optics. This is a value between 0 and 1. |
| `focus_pos` | R/W | `int32` | The focus position of the camera in counts. |
| `scale_limit_low` | R/W | `double` | The lower limit of the temperature scale in Kelvin in the "current case". |
| `scale_limit_high` | R/W | `double` | The upper limit of the temperature scale in Kelvin in the "current case". |
| `current_case` | R/W | `int32` | The "current case" of the camera. This is used to select different calibration ranges of the camera. For the A320, it is between 0 and 3 |
| `ir_format` | R/W | `string` | The format of the IR data. This is `temperature_linear_10mK`, `temperature_linear_100mK`, or `radiometric` for the A320. |
| `fps` | R/W | `double` | The frame rate of the camera in frames per second. For the A320, valid values are 10, 15, 30, and 60 |

For the A320 camera, the `current_case=2` is a high temperature range between 200 C and 1200 C. The first two 
`current_case` are for human body temperature reading.

For the A320, either `temperature_linear_10mK` or `temperature_linear_100mK` should be used. Due to the lack
of thermal calibration parameters, it is not possible to convert radiometric data to temperature. Note that 
there may be a several second delay when changing the `ir_format` parameter before the camera will start
outputting data again.

It is recommended all parameters be configured before using the camera.

Note that `VarValue` must be used with the `setf_param()` function to set the parameters. For example:

```python
c1.setf_param("object_emissivity", RR.VarValue(0.95, "double"))
```

See the `ir_camera_parameters.py` example and the linked documentation for more information on how to use the parameters.

## License

Apache 2.0
