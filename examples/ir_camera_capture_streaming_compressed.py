# Capture and display streaming compressed frames

from RobotRaconteur.Client import *
import numpy as np
import matplotlib.pyplot as plt
import cv2

def packet_received(self, pipe):
    pass

image_consts = None

def main():

    url='rr+tcp://127.0.0.1:60827/?service=camera'

    c1=RRN.ConnectService(url)

    global image_consts
    image_consts = RRN.GetConstants('com.robotraconteur.image', c1)

    p=c1.frame_stream_compressed.Connect(-1)

    #Set the callback for when a new pipe packet is received to the
    #new_frame function
    p.PacketReceivedEvent+=new_frame
    try:
        c1.start_streaming()
    except: pass


    fig = plt.figure(1)

    try:
        while True:
            if current_mat is not None:
                plt.imshow(current_mat, cmap='inferno', aspect='auto')
                plt.colorbar(format='%.2f')
            plt.pause(0.001)
            plt.clf()
    
    finally:
        try:
            p.Close()
        except: pass

        try:
            c1.stop_streaming()
        except: pass


current_mat = None

def new_frame(pipe_ep):
    global current_mat

    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        rr_img=pipe_ep.ReceivePacket()
               
        #Convert the packet to an image and set the global variable
        mat = cv2.imdecode(rr_img.data,cv2.IMREAD_UNCHANGED)
        ir_format = rr_img.image_info.extended["ir_format"].data

        if ir_format == "temperature_linear_10mK":
            display_mat = (mat * 0.01) - 273.15    
        elif ir_format == "temperature_linear_100mK":
            display_mat = (mat * 0.1) - 273.15    
        else:
            display_mat = mat

        current_mat = display_mat

if __name__ == "__main__":
    main()