import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import image

# This is just a function to plot the data the same way MATLAB does
def plot_map(Longitude, Latitude, Title):
    plt.plot(Longitude, Latitude, 'b-', zorder=2)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(Title)
    plt.ticklabel_format(useOffset=False)
    plt.show()

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

def m2dist(mx, my):
    distances = []
    for i in range(len(mx)-1):
        distances.append(math.sqrt((mx[i] - mx[i+1])**2 + (my[i]-my[i+1])**2))
    return distances

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


## Plot the position data (x and y) in the same plot, which should give you the path taken by the car 
# (this path is the same path I showed you in the lectures). 

# Extent adjusts the position of the image in the plot. Attempt to find a better solution if possible
extent=[802800, 804400, 6308900, 6310400]

map = image.imread('halmstad_drive_area.gif')
fig, ax = plt.subplots()
# TODO Fix extent (adapt automatically if possible)
ax.imshow(map, extent=extent)


# Read the NMEA-0183 data
DATA = np.loadtxt('gps_ex1_morningdrive.txt')
Longitude = DATA[:,3]
Latitude = DATA[:,2]

# Convert to degrees
LongDeg = NMEA2deg(Longitude)
LatDeg = NMEA2deg(Latitude)

# Convert to meters and plot
LongM, LatM = deg2m(LongDeg, LatDeg)
ax.plot(LongM, LatM, 'b-')
#plot_map(LongM, LatM, 'Position in meters')
plt.show()

## Plot the speed data
# In m/s
# Assuming the time between measurements is 1 second. In this case the distance in meters is equal to
# the velocity in m/s. 
ms_speed = m2dist(LongM, LatM)
plt.plot(ms_speed)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title("Speed in m/s")
plt.show()

# In km/h
kmh_speeds = [x*3.6 for x in ms_speed]
plt.plot(kmh_speeds)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (km/h)')
plt.title("Speed in km/h")
plt.show()

## Q9: Calculate the maximum speed taken by the driver.
max_speed = max(kmh_speeds)
print("Maximum Speed is: ", max_speed, "km/h")

## Q10: Did I ever break the speed limits, i.e. 70 km/h? 

# Yes, here are the positions plotted on the map
above_limit_x = []
above_limit_y = []
for i in range(len(kmh_speeds)-1):
    if kmh_speeds[i] >= 70:
        above_limit_x.append(LongM[i])
        above_limit_y.append(LatM[i])
        
fig, ax = plt.subplots()
ax.imshow(map, extent=extent)
ax.plot(LongM, LatM, 'b-')
ax.plot(above_limit_x, above_limit_y, 'ro')
plt.show()

## Q11: How come the estimate in speed is so accurate while the estimate in position is not? 
# This is very important! Recall what you find out about the GPS error in the first part of the exercise. 
# TODO

## Plot the headings
# Calculate the headings (and plot them with respect to time) along the path 
# (see compendium http://adamchukpa.mcgill.ca/web_ssm/web_GPS.html). 
# Note that the the matlab function atan2d() can be used here instead of the "if statments"  
# in the compendium. 
heading = []
for i in range(len(LongM)-1):
    heading.append((math.atan2(LatM[i+1] - LatM[i], LongM[i+1] - LongM[i])*180)/math.pi)

plt.plot(heading)
plt.show()

## Q12: Can you calculate the error in headings? Tips! If you only consider the headings along the
#  straight-line path (along Laholmsvägen or Vrangelsleden) – can you then estimate the variance in
#  the heading? 
x_error = LongM - np.mean(LongM)
y_error = LatM - np.mean(LatM)

plot_map(x_error, y_error, 'Position error in meters')

## Mark the heading values used for calculating the heading variance in the heading plot and the plot at the position plot above. Tips use the data points between 240:315.
