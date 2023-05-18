import cv2
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import argparse
import sys
import platform
import threading
import numpy as np
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
import struct
import socket
import re
import time
import weakref
from contextlib import suppress
import traceback

import PySpin

class _ImageEventHandler(PySpin.ImageEventHandler):
    def __init__(self, parent):
        super().__init__()

        self.parent = parent


    def OnImageEvent(self, image):
        self.parent._image_received(image)


class ThermalCameraImpl(object):
    
    def __init__(self, thermal_camera,camera_info):
        
        self._cam = thermal_camera
        self._nodemap = None
        self._fps = 0

        self._seqno = 0

        self._imaging_consts = RRN.GetConstants('com.robotraconteur.imaging')
        self._image_consts = RRN.GetConstants('com.robotraconteur.image')
        self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
        self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')
        self._compressed_image_type = RRN.GetStructureType('com.robotraconteur.image.CompressedImage')
        self._date_time_utc_type = RRN.GetPodDType('com.robotraconteur.datetime.DateTimeUTC')
        self._isoch_info = RRN.GetStructureType('com.robotraconteur.device.isoch.IsochInfo')
        self._capture_lock = threading.Lock()
        self._settings_lock = threading.Lock()
        self._streaming = False
        self._camera_info = camera_info
        self._date_time_util = DateTimeUtil(RRN)
        self._sensor_data_util = SensorDataUtil(RRN)
        self._image_event_handler = None
        self._current_image = None
        self._wires_init = False

    def RRServiceObjectInit(self, ctx, service_path):
        self._downsampler = RR.BroadcastDownsampler(ctx)
        self._downsampler.AddPipeBroadcaster(self.frame_stream)
        self._downsampler.AddPipeBroadcaster(self.frame_stream_compressed)
        self._downsampler.AddPipeBroadcaster(self.preview_stream)
        self._downsampler.AddWireBroadcaster(self.device_clock_now)
        self.frame_stream.MaxBacklog = 2
        self.frame_stream_compressed.MaxBacklog = 2
        self.preview_stream.MaxBacklog = 2
        
        # TODO: Broadcaster peek handler in Python
        self.device_clock_now.PeekInValueCallback = lambda ep: self._date_time_util.FillDeviceTime(self._camera_info.device_info,self._seqno)
        self._wires_init = True

    def _start(self):
        self._cam.Init()
        self._nodemap = self._cam.GetNodeMap()
        
        fps = _get_fps(self._nodemap)
        if fps is not None:
            self._fps = fps
        else:
            print('Unable to retrieve frame rate')

        ir_format_val = _gige_read_node_value(self._nodemap, "IRFormat")
        self._current_irformat = _ir_format_params_rev[ir_format_val]

        self._image_event_handler = _ImageEventHandler(self)
        self._cam.RegisterEventHandler(self._image_event_handler)

        self._cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
        self._cam.BeginAcquisition()

    @property
    def device_info(self):
        return self._camera_info.device_info

    @property
    def camera_info(self):
        return self._camera_info

    def _cv_mat_to_image(self, mat):

        image_info = self._image_info_type()
        image_info.width =mat.shape[1]
        image_info.height = mat.shape[0]
        
        image_info.step = mat.shape[1]
        image_info.encoding = self._image_consts["ImageEncoding"]["mono16"]
       
        image_info.data_header = self._sensor_data_util.FillSensorDataHeader(self._camera_info.device_info,self._seqno)
        image_info.extended = {
            "ir_format": RR.VarValue(self._current_irformat, "string")
        }
        

        image = self._image_type()
        image.image_info = image_info
        image.data=mat.reshape(mat.size, order='C').tobytes()
        return image

    def _cv_mat_to_compressed_image(self, mat, quality = 100):

        image_info = self._image_info_type()
        image_info.width =mat.shape[1]
        image_info.height = mat.shape[0]
        
        image_info.step = 0
        image_info.encoding = self._image_consts["ImageEncoding"]["compressed"]
        image_info.data_header = self._sensor_data_util.FillSensorDataHeader(self._camera_info.device_info,self._seqno)
        image_info.extended = {
            "ir_format": RR.VarValue(self._current_irformat, "string")
        }
        
        image = self._compressed_image_type()
        image.image_info = image_info
        # jpg can't handle 16 bit images, use png instead
        res, encimg = cv2.imencode(".png",mat)
        assert res, "Could not compress frame!"
        image.data=encimg
        return image

    def capture_frame(self):
        with self._capture_lock:
            mat = self._current_image
            if mat is None:
                raise RR.OperationFailedException("Could not read from camera")
        return self._cv_mat_to_image(mat)

    def capture_frame_compressed(self):
        with self._capture_lock:
            mat = self._current_image
            if mat is None:
                raise RRN.OperationFailedException("Could not read from camera")
        return self._cv_mat_to_compressed_image(mat)

    def trigger(self):
        raise RR.NotImplementedException("Not available on this device")

    def start_streaming(self):
        with self._settings_lock:
            if (self._streaming):
                raise RR.InvalidOperationException("Already streaming")
            self._streaming=True
            # t=threading.Thread(target=self.frame_threadfunc)
            # t.daemon=True
            # t.start()
            

    def stop_streaming(self):
        with self._settings_lock:
            if (not self._streaming):
                raise RR.InvalidOperationException("Not streaming")
            self._streaming=False
            
    @property
    def isoch_downsample(self):
        return self._downsampler.GetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint())

    @isoch_downsample.setter
    def isoch_downsample(self, value):
        return self._downsampler.SetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint(),value)

    @property
    def isoch_info(self):
        ret = self._isoch_info()
        ret.update_rate = self._fps
        ret.max_downsample = 100
        ret.isoch_epoch = np.zeros((1,),dtype=self._date_time_utc_type)

    @property
    def capabilities(self):
        return 0x1 | 0x2 | 0x4

    def _close(self):

        self._cam.EndAcquisition()

        if self._streaming:
            self._streaming = False
        

        if self._image_event_handler is not None:
            self._cam.UnregisterEventHandler(self._image_event_handler)
        if self._cam is not None:
            self._cam.DeInit()
        del self._cam

    def _close_rr(self):
        self._wires_init = False

    def _image_received(self, image):
        try:
            self._seqno+=1
            
            device_now = self._date_time_util.FillDeviceTime(self._camera_info.device_info,self._seqno)
            if self._wires_init:
                self.device_clock_now.OutValue = device_now

            if image.IsIncomplete():
                # Deal with trailing buffer bug on ThermoVision A320
                if image.GetImageStatus() != 5:
                    print('Image incomplete with image status %d...' % image.GetImageStatus())
                    return

            image2 = image.Convert(PySpin.PixelFormat_Mono16)
            mat = image2.GetNDArray()
            self._current_image = mat

            if self._streaming and self._wires_init:
                self.frame_stream.AsyncSendPacket(self._cv_mat_to_image(mat),lambda: None)
                self.frame_stream_compressed.AsyncSendPacket(self._cv_mat_to_compressed_image(mat),lambda: None)
                self.preview_stream.AsyncSendPacket(self._cv_mat_to_compressed_image(mat,70),lambda: None)
        except Exception as e:
            traceback.print_exc()

    def getf_param(self, param_name):

        _normal_param = _normal_params.get(param_name)
        if _normal_param is not None:
            return RR.VarValue(_gige_read_node_value(self._nodemap, _normal_param[0]), _normal_param[1])

        if param_name == "fps":
            return RR.VarValue(_get_fps(self._nodemap), "double")

        if param_name == "ir_format":
            ir_format_val = _gige_read_node_value(self._nodemap, "IRFormat")
            return RR.VarValue(_ir_format_params_rev[ir_format_val], "string")
        
        raise RR.InvalidArgumentException("Invalid parameter")

    def setf_param(self, param_name, value):
        _normal_param = _normal_params.get(param_name)
        if _normal_param is not None:
            _gige_set_node_value(self._nodemap, _normal_param[0], value.data[0])
            return
        if param_name == "fps":
            available_fps = dict(_get_available_fps(self._nodemap))
            assert float(value.data) in available_fps, f"Invalid fps specified: {value.data}"
            _gige_set_node_value(self._nodemap, "IRFrameRate", available_fps[float(value.data)])
            return          
        
        if param_name == "ir_format":
            ir_format_e = _ir_format_params.get(value.data, None)
            if ir_format_e is not None:
                _gige_set_node_value(self._nodemap, "IRFormat", ir_format_e)
                self._current_irformat = value.data
                return
            else:
                raise RR.InvalidArgumentException(f"Invalid ir_format specified: {value.data}")
    

        raise RR.InvalidArgumentException("Invalid parameter")

_ir_format_params = {
    "temperature_linear_10mK": "TemperatureLinear10mK",
    "temperature_linear_100mK": "TemperatureLinear100mK",
    "radiometric": "Radiometric"
}

_ir_format_params_rev = {
    "TemperatureLinear10mK": "temperature_linear_10mK",
    "TemperatureLinear100mK": "temperature_linear_100mK",
    "Radiometric": "radiometric"
}


_normal_params = {
    "object_emissivity": ("ObjectEmissivity", "double"),
    "object_distance": ("ObjectDistance", "double"),
    "reflected_temperature": ("ReflectedTemperature", "double"),
    "atmospheric_temperature": ("AtmosphericTemperature", "double"),
    "relative_humidity": ("RelativeHumidity", "double"),
    "estimated_transmission": ("EstimatedTransmission", "double"),
    "ext_optics_temperature": ("ExtOpticsTemperature", "double"),
    "ext_optics_transmission": ("ExtOpticsTransmission", "double"),
    "focus_pos": ("FocusPos", "int32"),
    "scale_limit_low": ("ScaleLimitLow", "double"),
    "scale_limit_upper": ("ScaleLimitUpper", "double"),
    "current_case": ("CurrentCase", "int32")
}

class PySpinSystem:

    def __init__(self):
        self._system = None

    def start(self):
        self._system = PySpin.System.GetInstance()

    def open_thermal_camera(self, serial_number = None, ip_address = None, mac_address = None):

        ip_integer = _ip_str_to_gige_integer(ip_address)
        mac_integer = _mac_str_to_gige_integer(mac_address)

        cam_list = self._system.GetCameras()
        try:
            
            num_cameras = cam_list.GetSize()

            assert num_cameras > 0, "Could not find any cameras!"

            for i, cam in enumerate(cam_list):
                if serial_number is None and ip_integer is None and mac_integer is None:
                    return cam
                nodemap_tldevice = cam.GetTLDeviceNodeMap()
               
                if serial_number is not None:
                    device_serial_number = _gige_read_node_value(nodemap_tldevice, 'DeviceSerialNumber')
                    if device_serial_number is not None:                    
                        if device_serial_number.strip() == serial_number.strip():
                            return cam
                if ip_integer is not None:
                    device_ip_integer = _gige_read_node_value(nodemap_tldevice, "GevDeviceIPAddress")
                    if device_ip_integer == ip_integer:
                        return cam
                if mac_integer is not None:
                    device_mac_integer = _gige_read_node_value(nodemap_tldevice, 'GevDeviceMACAddress')
                    if device_mac_integer == mac_integer:
                        return cam
                
            assert False, "Could not find requested camera!"

        finally:
            cam_list.Clear()
    
    def print_detected_cameras(self):
        cam_list = self._system.GetCameras()
        try:
            
            num_cameras = cam_list.GetSize()

            assert num_cameras > 0, "Could not find any cameras!"

            for i, cam in enumerate(cam_list):
                nodemap_tldevice = cam.GetTLDeviceNodeMap()
                
                device_vendor = _gige_read_node_value(nodemap_tldevice, 'DeviceVendorName')
                device_model = _gige_read_node_value(nodemap_tldevice, 'DeviceModelName')
                device_serial_number = _gige_read_node_value(nodemap_tldevice, 'DeviceSerialNumber')
                device_id = _gige_read_node_value(nodemap_tldevice, 'DeviceID')
                device_ip_address = _gige_integer_to_ip_str(_gige_read_node_value(\
                    nodemap_tldevice, 'GevDeviceIPAddress'))
                device_mac_address = _gige_integer_to_mac_str(_gige_read_node_value(\
                    nodemap_tldevice, 'GevDeviceMACAddress'))

                print(f"{device_vendor}, {device_model}, {device_serial_number}, {device_id}, {device_ip_address}, {device_mac_address}")
        finally:
            cam_list.Clear()

    def close(self):
        if self._system is not None:
            self._system.ReleaseInstance()

def _gige_integer_to_ip_str(ip_integer):
    if ip_integer is None:
        return None
    # https://stackoverflow.com/questions/9590965/convert-an-ip-string-to-a-number-and-vice-versa
    return socket.inet_ntoa(struct.pack("!L", ip_integer))
def _ip_str_to_gige_integer(ip_str):
    if ip_str is None:
        return None
    # https://stackoverflow.com/questions/9590965/convert-an-ip-string-to-a-number-and-vice-versa
    packedIP = socket.inet_aton(ip_str)
    return struct.unpack("!L", packedIP)[0]

def _gige_integer_to_mac_str(mac_integer):
    if mac_integer is None:
        return None
    # https://stackoverflow.com/questions/36857521/how-to-convert-mac-address-to-decimal-in-python
    mac_hex = "{:012x}".format(mac_integer)
    mac_str = ":".join(mac_hex[i:i+2] for i in range(0, len(mac_hex), 2))
    return mac_str

def _mac_str_to_gige_integer(mac_str):
    if mac_str is None:
        return None
    # https://stackoverflow.com/questions/36857521/how-to-convert-mac-address-to-decimal-in-python
    mac_int = int(mac_str.translate(str.maketrans('','', ":.- ")), 16)
    return mac_int

def _gige_read_node_value(nodemap_tldevice, nodename):
    node = nodemap_tldevice.GetNode(nodename)
    if node is None:
        return None
    node_type_code = node.GetPrincipalInterfaceType()
    if node_type_code == PySpin.intfIString:
        node_val = PySpin.CStringPtr(node)
    elif node_type_code == PySpin.intfIInteger:
        node_val = PySpin.CIntegerPtr(node)
    elif node_type_code == PySpin.intfIFloat:
        node_val = PySpin.CFloatPtr(node)
    elif node_type_code == PySpin.intfIEnumeration:
        node_val = PySpin.CEnumerationPtr(node)
    if not (PySpin.IsAvailable(node_val) and PySpin.IsReadable(node_val)):
        return None
    if hasattr(node_val, "GetIntValue"):
        return node_val.GetEntry(node_val.GetIntValue()).GetDisplayName()
    return node_val.GetValue()

def _gige_set_node_value(nodemap_tldevice, nodename, value):
    node = nodemap_tldevice.GetNode(nodename)
    assert node is not None, "Invalid flir attribute node"
    node_type_code = node.GetPrincipalInterfaceType()
    if node_type_code == PySpin.intfIString:
        node_val = PySpin.CStringPtr(node)
        node_val.SetValue(value)
    elif node_type_code == PySpin.intfIInteger:
        node_val = PySpin.CIntegerPtr(node)
        node_val.SetValue(int(value))
    elif node_type_code == PySpin.intfIFloat:
        node_val = PySpin.CFloatPtr(node)
        node_val.SetValue(float(value))
    elif node_type_code == PySpin.intfIEnumeration:
        node_val = PySpin.CEnumerationPtr(node)
        node_entry = node_val.GetEntryByName(value)
        assert node_entry is not None
        node_entry_int = PySpin.CEnumEntryPtr(node_entry).GetValue()
        node_val.SetIntValue(node_entry_int)
    else:
        assert False, "Unsupported node type"

def _get_fps(nodemap):
    acquisition_framerate = _gige_read_node_value(nodemap, 'AcquisitionFrameRate')
    if acquisition_framerate is not None:
        return acquisition_framerate

    ir_framerate = _gige_read_node_value(nodemap, 'IRFrameRate')
    if ir_framerate is not None:
        re_match = re.match(r".*Rate(\d+)Hz$", ir_framerate)
        if not re_match:
            print("Could not parse IRFrameRate enum")
        return float(re_match.group(1))

    return None

def _get_available_fps(nodemap):
    fps_enum_node = PySpin.CEnumerationPtr(nodemap.GetNode("IRFrameRate"))
    assert PySpin.IsAvailable(fps_enum_node) or not PySpin.IsReadable(fps_enum_node), \
        "Could not find IRFrameRate gige node"
    fps_enum_entries = fps_enum_node.GetEntries()
    ret = []
    for fps_enum_entry in fps_enum_entries:
        node_entry = PySpin.CEnumEntryPtr(fps_enum_entry)
        if not PySpin.IsAvailable(node_entry) or not PySpin.IsReadable(node_entry):
            continue
        re_match = re.match(r".*Rate(\d+)Hz$", node_entry.GetDisplayName())
        if not re_match:
            print("Could not parse IRFrameRate enum")
        ret.append((float(re_match.group(1)), str(node_entry.GetDisplayName())))
    return ret

    
def main():
    parser = argparse.ArgumentParser(description="Flir thermal camera driver service for Robot Raconteur")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--camera-info-file", type=argparse.FileType('r'),default=None,help="Camera info file (required)")
    group.add_argument("--list-cameras", action='store_true',default=False,help="List available cameras and exit")
    group2 = parser.add_mutually_exclusive_group()
    group2.add_argument("--camera-serial-number", type=str, default=None, help="Serial number of desired camera")
    group2.add_argument("--camera-ip-address", type=str, default=None, help="IP address of desired camera")
    group2.add_argument("--camera-mac-address", type=str, default=None, help="MAC address of desired camera")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    if (args.list_cameras):
        cam_sys = PySpinSystem()
        cam_sys.start()
        cam_sys.print_detected_cameras()
        return

    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv

    #RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.imaging'],True)
    RRC.RegisterStdRobDefServiceTypes(RRN)

    with args.camera_info_file:
        camera_info_text = args.camera_info_file.read()

    info_loader = InfoFileLoader(RRN)
    camera_info, camera_ident_fd = info_loader.LoadInfoFileFromString(camera_info_text, "com.robotraconteur.imaging.camerainfo.CameraInfo", "camera")

    attributes_util = AttributesUtil(RRN)
    camera_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(camera_info.device_info)
    
    cam_sys = PySpinSystem()
    cam_sys.start()
    cam = cam_sys.open_thermal_camera(args.camera_serial_number, args.camera_ip_address, args.camera_mac_address)

    # Use weakref.proxy to avoid creating dangling references to camera
    weak_cam_proxy = weakref.proxy(cam)
    camera = ThermalCameraImpl(weak_cam_proxy, camera_info)
    try:
        camera._start()

        
        with RR.ServerNodeSetup("experimental.flir_thermal_camera",60827,argv=rr_args):

            service_ctx = RRN.RegisterService("camera","com.robotraconteur.imaging.Camera",camera)
            service_ctx.SetServiceAttributes(camera_attributes)
            time.sleep(1)
            camera.start_streaming()

            if args.wait_signal:  
                #Wait for shutdown signal if running in service mode          
                print("Press Ctrl-C to quit...")
                import signal
                signal.sigwait([signal.SIGTERM,signal.SIGINT])
            else:            
                input("Server started, press enter to quit...")
            
            camera._close_rr()
            time.sleep(0.1)
    finally:
        camera._close_rr()
        with suppress(Exception):
            camera._close()
        del camera

        del cam

        with suppress(Exception):
            cam_sys.close()
        del cam_sys