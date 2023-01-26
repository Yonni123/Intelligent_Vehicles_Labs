import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
from scipy.stats import norm


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
    ell = Ellipse(xy=mean, width=width, height=height, angle=angle, color='red', lw=2, zorder=2)
    ell.set_facecolor('none')
    plt.gca().add_artist(ell)

    # Plotting the data points
    plt.plot(data[:, 0], data[:, 1], 'b-', zorder=1)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(Title)

    # Scatter plot the first 10 of the data points and mark them as .
    plt.scatter(data[:10, 0], data[:10, 1], marker='.', color='red', zorder=3)
    plt.scatter(data[101:110, 0], data[101:110, 1], marker='.', color='red', zorder=3)

    # Find the max point and mark it as non-filled circle
    max_point = np.argmax(np.sqrt(np.power(data[:, 0], 2) + np.power(data[:, 1], 2)))
    plt.scatter(data[max_point, 0], data[max_point, 1], marker='o', facecolors='none', edgecolors='red', zorder=4)

    # Generate 10 random points with the same variance as X and Y and plot them as x
    random_points = np.random.multivariate_normal(mean, cov_matrix, 10)
    plt.scatter(random_points[:, 0], random_points[:, 1], marker='x', color='red', zorder=5)

    # Print the value of the max point
    print('Max point: ', data[max_point, :])
    print('value:' , np.sqrt(np.power(data[max_point, 0], 2) + np.power(data[max_point, 1], 2)))

    plt.show()

# Calculate the covariance matrix
cov = np.cov(x_error, y_error)
print('Covariance matrix:\n', cov)
plot_uncertainty(x_error, y_error, 'Uncertainty')

# Plot histograms of the x and y errors
# Plot a normal distribution with the same mean and variance as the x and y errors
#plt.hist(x_error, bins=30, edgecolor='black')
plt.title('Histogram of x error')
plt.xlabel('x error')
plt.ylabel('Frequency')

# Plot the histogram.
(n, _, _) = plt.hist(x_error, bins=25, edgecolor='black')
binmax = np.max(n)

# Plot the PDF.
mu, std = np.mean(x_error), np.std(x_error)
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mu, std)
multiplier = binmax / np.max(p)
plt.plot(x, multiplier*p, 'k', linewidth=2)
plt.show()

# Plot histograms of the x and y errors
plt.title('Histogram of y error')
plt.xlabel('y error')
plt.ylabel('Frequency')

# Plot the histogram.
(n, _, _) = plt.hist(y_error, bins=25, edgecolor='black')
binmax = np.max(n)

# Plot the PDF.
mu, std = np.mean(x_error), np.std(x_error)
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mu, std)
multiplier = binmax / np.max(p)
plt.plot(x, multiplier*p, 'k', linewidth=2)
plt.show()

# 3.3 Plot, with respect to time, the errors and the auto-correlation in x and y separately.
# -> Your code here

# Plot the errors in x and y with respect to time
# X and Y in separate sub-plots
Time = np.arange(0, len(x_error), 1)
plt.subplot(2, 1, 1)
plt.plot(Time, x_error)
plt.title('X and Y error with respect to time')
plt.xlabel('')
plt.ylabel('X error')

plt.subplot(2, 1, 2)
plt.plot(Time, y_error)
plt.title('')
plt.xlabel('Time')
plt.ylabel('Y error')
plt.show()

# Calculate the auto-correlation in x and y
# Plot the auto-correlation in x and y with respect to time

# Generate a random signal as long as the x and y error
x_rand = np.random.normal(0, 1, len(x_error))
y_rand = np.random.normal(0, 1, len(y_error))

# Plot the random signal over time
plt.subplot(2, 1, 1)
plt.plot(Time, x_rand, color='red')
plt.title('Random X and Y error with respect to time')
plt.xlabel('')
plt.ylabel('X error')

plt.subplot(2, 1, 2)
plt.plot(Time, y_rand, color='red')
plt.title('')
plt.xlabel('Time')
plt.ylabel('Y error')
plt.show()

# Calculate the auto-correlation in x and y
x_auto_corr = np.correlate(x_error, x_error, mode='full')
y_auto_corr = np.correlate(y_error, y_error, mode='full')
x_rand_auto_corr = np.correlate(x_rand, x_rand, mode='full')
y_rand_auto_corr = np.correlate(y_rand, y_rand, mode='full')

# Normalize the auto-correlation
x_auto_corr = x_auto_corr / np.max(x_auto_corr)
y_auto_corr = y_auto_corr / np.max(y_auto_corr)
x_rand_auto_corr = x_rand_auto_corr / np.max(x_rand_auto_corr)
y_rand_auto_corr = y_rand_auto_corr / np.max(y_rand_auto_corr)

# Plot the auto-correlation in x and y with respect to same time as previous plot
Time = np.arange(-len(x_error) + 1, len(x_error), 1)
plt.subplot(2, 1, 1)
plt.plot(Time, x_auto_corr, zorder=2)
plt.plot(Time, x_rand_auto_corr, color='red', zorder=1)
plt.title('X and Y auto-correlation with respect to time')
plt.xlabel('')
plt.ylabel('X auto-correlation')
# Hide x-axis numberings on this subplot
plt.xticks([])

plt.subplot(2, 1, 2)
plt.plot(Time, y_auto_corr, zorder=2)
plt.plot(Time, y_rand_auto_corr, color='red', zorder=1)
plt.title('')
plt.xlabel('Time')
plt.ylabel('Y auto-correlation')
plt.show()
