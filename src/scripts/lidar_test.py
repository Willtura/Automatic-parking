import numpy as np
import matplotlib.pyplot as plt
from math import floor
from adafruit_rplidar import RPLidar
import matplotlib
from sklearn.cluster import DBSCAN, KMeans
import cv2


PORT_NAME = 'COM4'
lidar = RPLidar(None, PORT_NAME, timeout=3)
max_distance = 0

def find_obstacle_distance(data, anglemin, anglemax):
    point = 15000
    for i in range(anglemin, anglemax):
        if point > data[i] > 500:
            point = data[i]
    return point

def detect_walls(data):
    """a function used to detect the walls around the car
    to the right of the car, there will be a straight line with a hole in it, detect this line"""
    size = 0
    max_i = 0
    initial_angle = 0
    for i in range(100, 260):
        if data[i] < 6000:
            distance = data[i]
            initial_angle = i
            current = 0
            jpos = data[i]
            max_i = 0
            for j in range(i, 260):
                if jpos - 1000 < data[j] < jpos + 1000:
                    jpos = data[j]
                    current += 1
                    max_i = j
            if current > size:
                size = current
    left_distance = data[initial_angle]
    right_distance = data[max_i]
    average_distance = (left_distance + right_distance) / 2

    return [(initial_angle + max_i) // 2, average_distance, initial_angle, max_i, left_distance, right_distance]


def display_minimum(data):
    #find the point on 0:40 and on 320:360
    point = 15000
    if point < pointleft:
        point = pointleft
    if point > 500:
        plt.plot(point,0, 'ro')  # Plot het eerste punt in het rood
    leftpoint = min(data[45:120])
    rightpoint = min(data[225:320])
    if leftpoint > 500:
        plt.plot(0, leftpoint, 'ro')
    if rightpoint > 500:
        plt.plot(0, -rightpoint, 'ro')
def process_data(data):
    # how to make the plt.plot in a different window instead of the editor
    # Gegevens omzetten naar polaire coördinaten
    angles = np.linspace(0, 2*np.pi, len(data), endpoint=False)
    distances = np.array(data)
    newdistances = distances

    for i in range(len(distances)):
        if distances[i] < 500:
            newdistances[i] = 15000
        #if distances[i] - distances[i-1] > 1500 or distances[i] - distances[i-1] < -1500:
            #newdistances[i] = np.nan
            #newdistances[i]= np.nan

    x = newdistances * np.cos(angles)
    y = newdistances * np.sin(angles)
    #detect_lines(newdistances)
    plt.clf()  # Clear het huidige plot
    plt.scatter(x, y, s=5)  # s is de grootte van de punten, je kunt deze aanpassen aan je voorkeur
    plt.title('2D Lidar')
    plt.xlabel('X')
    plt.ylabel('Y')
    display_minimum(newdistances)
    #print(wall)
    plt.xlim(-1000, 10000)
    plt.ylim(-2000,2000)
    plt.grid(True)
    plt.pause(0.01)  # Pauze om de plot te laten zien (voor realtime effect)

try:
    scan_data = [0]*360
    for scan in lidar.iter_scans():
        scan_data = [0]*360
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)  # dit is de data
except KeyboardInterrupt:
    print('Stopping.')
lidar.stop()
lidar.disconnect()