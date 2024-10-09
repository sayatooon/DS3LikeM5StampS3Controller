import serial
from serial.tools import list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
from matplotlib import patches
from matplotlib.widgets import Button, Slider
from cobs import cobs
import struct
import threading
import time

def main():
    #ser = select_port()

    serialUse =SerialUse(115200, None)
        
    if serialUse.open_port() == False:
        return
    ser = serialUse.get_port()
    serialUse.start()
    
    fig = plt.figure(figsize=(11,6.5))
    fig.subplots_adjust(right=0.95, left=0.1, top=0.9)

    # -- button visualizer --
    ax_img = plt.subplot(221)
    img = plt.imread('controller.png')
    ax_img.imshow(img, alpha=1, extent=(0, 500, -330, 0), zorder=2)
        
    # SLIDERS
    fig.text(0.3, 0.52, 'MOTORS', ha='center', weight='semibold')
    ax_motor_r = fig.add_axes([0.33, 0.2, 0.02, 0.27])
    ax_motor_l = fig.add_axes([0.25, 0.2, 0.02, 0.27])
    motor_r_slider = Slider(
        ax=ax_motor_r,
        label="RIGHT",
        valmin=-255,
        valmax=255,
        valinit=0,
        orientation="vertical"
    )
    motor_l_slider = Slider(
        ax=ax_motor_l,
        label="LEFT",
        valmin=-255,
        valmax=255,
        valinit=0,
        orientation="vertical"
    )

    def send_value(val):
        send_data = [int(motor_r_slider.val), int(motor_l_slider.val), 0]
        encode_data = cobs.encode(struct.pack("iii",*send_data))
        #print(encode_data)
        ser.write(encode_data + b'\x00')
        #ser.write(bytes(send_data, encoding='ascii') )

    motor_r_slider.on_changed(send_value)
    motor_l_slider.on_changed(send_value)    

    # RESET button
    ax_reset = fig.add_axes([0.25, 0.1, 0.1, 0.04])
    button_m = Button(ax_reset, 'RESET', hovercolor='0.975')

    def reset(event):
        motor_r_slider.reset()
        motor_l_slider.reset()

    button_m.on_clicked(reset)
    
    # IMU init button
    ax_init = fig.add_axes([0.57, 0.93, 0.1, 0.04])
    button_init = Button(ax_init, 'Init IMU', hovercolor='0.975')

    def send_init(event):
        send_data = [int(motor_r_slider.val), int(motor_l_slider.val), 1]
        encode_data = cobs.encode(struct.pack("iii",*send_data))
        ser.write(encode_data + b'\x00')

    button_init.on_clicked(send_init)

    # CONTROLLER BUTTON DRAWING
    p_ps = patches.Circle(xy=(250, -180), radius=16, fc='r', ec=None, alpha =0, zorder=1) # PS button
    ax_img.add_patch(p_ps)
    start = [(284.3,-127.4), (309.5, -135), (284.3, -142.6)] # START button
    p_start = patches.Polygon(start, fc='r', ec=None, alpha =0, zorder=1)
    ax_img.add_patch(p_start)
    p_sel = patches.Rectangle(xy=(190, -141), width=25, height=15, fc='r', ec=None, alpha =0, zorder=1) # SELECT button
    ax_img.add_patch(p_sel)

    p_l2 = patches.Rectangle(xy=(80, -24), width=50, height=22, fc='r', ec=None, alpha =0, zorder=1) # L2 button
    ax_img.add_patch(p_l2)
    p_l1 = patches.Rectangle(xy=(80, -45), width=50, height=10, fc='r', ec=None, alpha =0, zorder=1) # L1 button
    ax_img.add_patch(p_l1)
    up = [(90,-95.8), (114, -95.8), (114, -116), (102, -126), (90, -116)] # UP button
    p_up = patches.Polygon(up, fc='r', ec=None, alpha =0, zorder=1)
    ax_img.add_patch(p_up)
    left = [(93.5,-135), (83.4, -123), (63.3, -123), (63.3, -147), (83.4, -147)] # LEFT button
    p_left = patches.Polygon(left, fc='r', ec=None, alpha =0, zorder=1)
    ax_img.add_patch(p_left)
    down = [(102,-143), (114, -153), (114, -173), (90, -173), (90, -153)] # DOWN button
    p_down = patches.Polygon(down, fc='r', ec=None, alpha =0, zorder=1)
    ax_img.add_patch(p_down)
    right = [(110.5,-135), (121, -123), (141, -123), (141, -147), (121, -147)] # RIGHT button
    p_right = patches.Polygon(right, fc='r', ec=None, alpha =0, zorder=1)
    ax_img.add_patch(p_right)

    p_r1 = patches.Rectangle(xy=(370, -45), width=50, height=10, fc='r', ec=None, alpha =0,zorder=1) # R1 button
    ax_img.add_patch(p_r1)
    p_r2 = patches.Rectangle(xy=(370, -24), width=50, height=22, fc='r', ec=None, alpha =0, zorder=1) # R2 button
    ax_img.add_patch(p_r2)
    p_tri = patches.Circle(xy=(398, -97), radius=16, fc='r', ec=None, alpha =0, zorder=1) # TRI button
    ax_img.add_patch(p_tri)
    p_cir = patches.Circle(xy=(436, -135), radius=16, fc='r', ec=None, alpha =0, zorder=1) # CIR button
    ax_img.add_patch(p_cir)
    p_crs = patches.Circle(xy=(398, -173), radius=16, fc='r', ec=None, alpha =0, zorder=1) # CRS button
    ax_img.add_patch(p_crs)    
    p_sqr = patches.Circle(xy=(360, -135), radius=16, fc='r', ec=None, alpha =0, zorder=1) # SQR button
    ax_img.add_patch(p_sqr)
    p_r3 = patches.Circle(xy=(320, -205), radius=41, fc='r', ec=None, alpha =0, zorder=1) # R3 button
    ax_img.add_patch(p_r3)
    p_l3 = patches.Circle(xy=(180, -205), radius=41, fc='r', ec=None, alpha =0, zorder=1) # L3 button
    ax_img.add_patch(p_l3)    
    
    p_buttons = [p_ps, p_start, p_sel, p_l2, p_l1, p_up, p_left, p_down, p_right, p_r1, p_r2, p_tri, p_cir, p_crs, p_sqr, p_r3, p_l3]
    ax_img.set_axis_off()

    p_jr = patches.Circle(xy=(320, -205), radius=16, fc='darkgray', ec='dimgray',lw =2, alpha =1, zorder=3) # PS button
    ax_img.add_patch(p_jr)
    p_jl = patches.Circle(xy=(180, -205), radius=16, fc='darkgray', ec='dimgray', lw =2, alpha =1, zorder=3) # PS button
    ax_img.add_patch(p_jl)    

    p_joy = [p_jr, p_jl]

    # -- serial plotter --
    ax_acc = plt.subplot(322)
    ax_acc.set_ylim(-4, 4)
    ax_gyro = plt.subplot(324)
    ax_gyro.set_ylim(-500, 500)
    ax_mag = plt.subplot(326)
    ax_mag.set_ylim(-600, 600)    
    ax_mag.set_yticks(np.arange(-600, 601, 300))
    ax_acc.set_ylabel('acc [g]')
    ax_gyro.set_ylabel('gyro [deg/s]')
    ax_mag.set_ylabel('mag [uT]')
    
    ax_acc.set_xticks([])
    ax_gyro.set_xticks([])
    ax_mag.set_xticks([])
    
    color_list = ['r', 'g', 'b']
    label_list = ['x', 'y', 'z']

    
    data_acc = [deque([0.0]*100, 100) for i in range(3)]
    line_acc = [ax_acc.plot(range(100), data_acc[i], color=color_list[i], label=label_list[i])[0] for i in range(3)]
    ax_acc.legend( loc='lower center', bbox_to_anchor=(0.75, 1.1), ncol=3)
    data_gyro = [deque([0.0]*100, 100) for i in range(3)]
    line_gyro = [ax_gyro.plot(range(100), data_gyro[i], color=color_list[i], label=label_list[i])[0] for i in range(3)]
    data_mag = [deque([0.0]*100, 100) for i in range(3)]
    line_mag = [ax_mag.plot(range(100), data_mag[i], color=color_list[i], label=label_list[i])[0] for i in range(3)]
    
    data_IMU = [data_acc, data_gyro, data_mag]
    line_IMU = [line_acc, line_gyro, line_mag]



    ani = animation.FuncAnimation(fig=fig,
                                  func=update,
                                  fargs =(serialUse, data_IMU, line_IMU, p_buttons, p_joy),
                                  frames=200,
                                  interval=50)
    
    plt.show()
    
class SerialUse():
    def __init__(self, baudrate, timeout):
        self.__ser = serial.Serial()
        self.__ser.baudrate = baudrate
        self.__ser.timeout = timeout
        self.__decoded_data =[0] * 30  
        self.__thread  = threading.Thread(target=self.update, daemon = True)     
        
    def open_port(self):
        ports = list_ports.comports()
        devices = [info.device for info in ports]
        
        if len(devices) == 0:
            print("Error: No port found.")
            return None
        elif len(devices) == 1:
            print(f"{devices[0]} found.")
            self.__ser.port = devices[0]
        else:
            for i in range(len(devices)):
                print(f"Input {i:d} to open {devices[i]}.")
            num = int(input("Enter No.:"))
            self.__ser.port = devices[num]
        
        try:
            self.__ser.open()
            return True
        except:
            print("Error: Couldn't open COM port.")
            return False
    
    def get_port(self):
        return self.__ser
    
    def get_data(self):
        return self.__decoded_data
    
    def start(self):        
        self.__thread.start()
           
    def update(self):
        while self.__ser.is_open:
            self.__ser.reset_input_buffer()
            read_data = self.__ser.read_until(b'\x00')
            n = len(read_data)
            
            if n == 122:
                self.__decoded_data = list(struct.unpack('iiiiiiiiiiiiiiiiiiiiifffffffff',cobs.decode(read_data[0:n-1])))
                #print(self.__decoded_data)
            
            time.sleep(0.010)

"""    
def select_port():
    ser = serial.Serial()
    ser.baudrate = 115200    
    ser.timeout = None

    ports = list_ports.comports()
    devices = [info.device for info in ports]

    if len(devices) == 0:
        print("Error: No port found.")
        return None
    elif len(devices) == 1:
        print(f"{devices[0]} found.")
        ser.port = devices[0]
    else:
        for i in range(len(devices)):
            print(f"Input {i:d} to open {devices[i]}.")
        num = int(input("Enter No.:"))
        ser.port = devices[num]
    
    try:
        ser.open()
        return ser
    except:
        print("Error: Couldn't open COM port.")
        return None

def serial_update(ser, decoded_data):    
    while ser.is_open:
    
        ser.reset_input_buffer()
        read_data = ser.read_until(b'\x00')
        n = len(read_data)
        
        if n == 122:
            decoded_data = list(struct.unpack('iiiiiiiiiiiiiiiiiiiiifffffffff',cobs.decode(read_data[0:n-1])))
            #print(decoded_data)
"""

def update(frame, serialUse, imu_data, line, p, pj):   

    decoded_data = serialUse.get_data()
    for i in range(17): # for buttons
        if decoded_data[i] == 4 or decoded_data[i] == 5:
            p[i].set_alpha(1)
        else:
            p[i].set_alpha(0)

    for i in range(4): # for joy sticks
        if abs(decoded_data[i+17]) < 2: decoded_data[i+17] = 0 # threshold for drawing
    pj[0].set_center([320+decoded_data[17]*2,-205+decoded_data[18]*2])
    pj[1].set_center([180+decoded_data[19]*2,-205+decoded_data[20]*2])
    for i in range(3): # for IMU
        for j in range(3):
            imu_data[i][j].append(decoded_data[i*3+j+21])
            line[i][j].set_ydata(imu_data[i][j])

if __name__ == "__main__":
    main()