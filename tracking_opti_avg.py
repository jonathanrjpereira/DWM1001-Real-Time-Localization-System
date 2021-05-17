import serial
import time
import threading

import math
from sklearn.metrics import euclidean_distances
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random

import matplotlib.image as mpimg

# Anchor node locations
# anchor = #4 #3 #2
anchor_x = [0,3280,3280]
anchor_y = [3070,3070,0]
anchor = list(zip(anchor_x, anchor_y))

def trilaterate(anchor_x, anchor_y, anchor1, anchor2, anchor3):
    """
    @brief: Trilaterate Tag location
    @param: anchor_x - List of anchor coordinates along the X-axis
            anchor_y - List of anchor coordinates along the Y-axis
            anchor1 - Distance to the 1st Anchor
            anchor2 - Distance to the 2nd Anchor
            anchor3 - Distance to the 3rd Anchor
    @ret:   tag_coordinates - Tag Coordinates in a numpy array.
    """
    r1_sq = pow(anchor1,2)
    r2_sq = pow(anchor2,2)
    r3_sq = pow(anchor3,2)

    # Solve a linear matrix equation where x,y is the Tag coordinate:
    # Ax + By = C
    # Dx + Ey = F
    A = (-2*anchor_x[0]) + (2*anchor_x[1])
    B = (-2*anchor_y[0]) + (2*anchor_y[1])
    C = r1_sq - r2_sq - pow(anchor_x[0],2) + pow(anchor_x[1],2) - pow(anchor_y[0],2) + pow(anchor_y[1],2) 
    D = (-2*anchor_x[1]) + (2*anchor_x[2])
    E = (-2*anchor_y[1]) + (2*anchor_y[2])
    F = r2_sq - r3_sq - pow(anchor_x[1],2) + pow(anchor_x[2],2) - pow(anchor_y[1],2) + pow(anchor_y[2],2) 

    a = np.array([[A, B], [D, E]])
    b = np.array([C, F])
    tag_coordinates = np.linalg.solve(a, b)
    # print("Tag Coordinate:", tag_coordinates)
    return tag_coordinates

# Configure the Serial Port
ser = serial.Serial(
    port='COM4',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=None)
print("connected to: " + ser.portstr)

# Set the figure dimensions
fig = plt.figure()
ax = plt.axes(xlim=(-500,4000),ylim=(-500,4000))

# Plot an image containing the room layout
img = mpimg.imread('C:/Users/Jonathan/Documents/GitHub/UWB/Basic_Tracking/half10.png')
imgplot = ax.imshow(img, extent=(-500, 4301-500, -500, 3987-500))

# Plot the anchors and initialize the tag locations.
anchor = np.array(anchor)
ax.scatter(anchor[:,0], anchor[:,1], c='blue')
scatter = ax.scatter(0, 0, c='red')

# Variables used to calculate the average location. Reduce effect of noisy measurements.
previous_value = [(0,0)]
value = 0
samples_to_count = 3
count = 0
total = np.array((0,0))

# Multithreaded serial read. 
def in_background():
    while(1):
        global value
        value = ser.readline().decode('utf-8', errors='replace')

thread = threading.Thread(target = in_background)
thread.start() 

# Filter out corrupted UART data and perform trilateration 
def update(particles, previous_value):
    global value
    print("Raw Data: ",value)
    if(("0x4818: =" in value) and (" | 0x528d: =" in value) and (" | 0x84b9: =" in value) and ("\r\n" in value)):
    # if(("4818" in value) and ("528d" in value) and ("84b9" in value) and ("\r\n" in value)):
        data = value.split(" | ")
        # print("Data: ", data)
        if(len(data) == 4): # check for 3 nodes and \r\n
            # print("data[0]: ", data[0])
            # print("data[1]: ", data[1])
            # print("data[2]: ", data[2])
            node1 = data[0].split("=")
            node2 = data[1].split("=")
            node3 = data[2].split("=")
            if (((len(node1) == 2) and (len(node2) == 2) and (len(node3) == 2)) and 
            ((node1[1].isnumeric() == 1) and (node2[1].isnumeric() == 1) and (node3[1].isnumeric() == 1))):
                node1 = int(node1[1])
                node2 = int(node2[1])
                node3 = int(node3[1])
                # print("node1: ", node1)
                # print("node2: ", node2)
                # print("node3: ", node3)
                tag = trilaterate(anchor_x, anchor_y, node1, node2, node3)
                # print("Tag Coordinate:", tag)
                previous_value = tag

                global count
                global samples_to_count
                global total
                if count < samples_to_count:
                        total = total + tag
                        count += 1
                if count == samples_to_count:
                    total /= samples_to_count
                    scatter.set_offsets(total)
                    # print("Tag: ", total)
                    total = np.array((0,0))
                    count = 0
            else:
                print("Node Length / Int Error")
                tag = previous_value
        else:
            print("3 node Error")
            tag = previous_value
    else:
        print("Readline Error")
        tag = previous_value

    return scatter,

anim = FuncAnimation(fig, update, interval = 0.0000001, fargs= previous_value, blit = True)

plt.show()
