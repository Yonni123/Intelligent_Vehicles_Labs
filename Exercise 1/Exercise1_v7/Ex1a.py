import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse


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
    h = 0.0  # height above ellipsoid's surface

    angles = np.radians(np.floor(lat))    # convert to radians for trig functions
    c = a**2 * np.power(np.cos(angles), 2) + b**2 * np.power(np.sin(angles), 2) # What's under the square root
    Flon = (math.pi/180) * (a**2 / np.sqrt(c) + h) * np.cos(angles)
    Flat = (math.pi/180) * (a**2 * b**2 / np.power(c, 3/2) + h)
    Flon = np.round(Flon, 0)
    Flat = np.round(Flat, 0)
    x = np.multiply(Flon, lon)
    y = np.multiply(Flat, lat)
    return x, y

LongM, LatM = deg2m(LongDeg, LatDeg)    # This gives different results depending on precision of trig functions
plot_map(LongM, LatM, 'Position in meters')

# 3.2 Estimate the mean and variance of the position (in x and y) Matlab function mean() and var()
# NO because MATLAB is cringe lol
# -> Your code here
x_error = LongM - np.mean(LongM)
y_error = LatM - np.mean(LatM)
plot_map(x_error, y_error, 'Position error in meters')

def plot_uncertainty(x_error, y_error, Title='Uncertainty'):
    # Calculate the covariance matrix
    data = np.array([x_error, y_error]).T
    cov_matrix = np.cov(data.T)

    # eigenvalue and eigenvectors from the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

    # mean of the data
    mean = np.mean(data, axis=0)

    # Plotting the ellipse
    angle = np.degrees(np.arctan2(*eigenvectors[:, 0][::-1]))
    width, height = 2 * np.sqrt(eigenvalues)
    ell = Ellipse(xy=mean, width=width, height=height, angle=angle, color='red', zorder=2)
    ell.set_facecolor('none')
    plt.gca().add_artist(ell)

    # Plotting the data points
    plt.plot(data[:, 0], data[:, 1], 'b-', zorder=1)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(Title)

    plt.show()

# Calculate the covariance matrix
cov = np.cov(x_error, y_error)
print('Covariance matrix:\n', cov)
plot_uncertainty(x_error, y_error, 'Uncertainty')

# 3.3 Plot, with respect to time, the errors and the auto-correlation in x and y separately.
# -> Your code here