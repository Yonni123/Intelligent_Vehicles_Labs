import math
import matplotlib.pyplot as plt
import numpy as np

# This is just a function to plot the data the same way MATLAB does
def plot_map(Longitude, Latitude, Title):
    plt.plot(Longitude, Latitude, 'b-')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(Title)
    plt.ticklabel_format(useOffset=False)
    plt.show()

DATA = np.loadtxt('gps_ex1_window.txt')
Longitude = DATA[:,3]
Latitude = DATA[:,2]
plot_map(Longitude, Latitude, 'Position in NMEA-0183 format')

# 3.1 Write a function that transform your longitude and latitude angles from NMEA-0183 into meters

# 1. longitude and latitude angles from NMEA-0183 into degrees
def NMEA2deg(NMEA):
    # floor(Longitude / 100) + (Longitude - floor(Longitude / 100) * 100) / 60;
    if type(NMEA) == float:
        deg = int(NMEA/100) # floor(Longitude / 100)
    elif type(NMEA) == np.ndarray:
        deg = np.floor(NMEA/100)
    else:
        print('NMEA must be float or numpy.ndarray')
        return

    min = NMEA - deg*100 # (Longitude - floor(Longitude / 100) * 100)
    return deg + min/60

LongDeg = NMEA2deg(Longitude)
LatDeg = NMEA2deg(Latitude)
plot_map(LongDeg, LatDeg, 'Position in degrees')

# 2. longitude and latitude angles from degrees into meters
def deg2m(lon, lat):
    # This formula is based on the equation in file "Longitude and Latitude Conversion Table.pdf"
    a = 6378137.0   # semi-major axis of WGS-84 ellipsoid
    b = 6356752.3142    # semi-minor axis of WGS-84 ellipsoid
    h = 0.0  # height above ellipsoid'
    for i in range(len(lat)):
        c = a**2 * math.cos(lat[i])**2 + b**2 * math.sin(lat[i])**2 # what's under the square root (for convenience)
        Flon = (math.pi/180) * (a**2 / np.sqrt(c) + h) * np.cos(lat)

    c = a**2 * np.power(np.cos(lat), 2) + b**2 * np.power(np.sin(lat), 2)
    Flon = (math.pi/180) * (a**2 / np.sqrt(c) + h) * np.cos(lat)
    Flat = (math.pi/180) * (a**2 * b**2 / np.power(c, 3/2) + h)
    x = lon * Flon
    y = lat * Flat
    return x, y

LongM, LatM = deg2m(LatDeg, LongDeg)
LongM = LongDeg * 62393.0
LatM = LatDeg * 111342.0
plot_map(LongM, LatM, 'Position in meters')

# 3.2 Estimate the mean and variance of the position (in x and y) Matlab function mean() and var()
# NO because MATLAB is cringe lol
# -> Your code here

# 3.3 Plot, with respect to time, the errors and the auto-correlation in x and y separately.
# -> Your code here