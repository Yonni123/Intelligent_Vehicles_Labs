import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import image
import statistics
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
        deg = int(NMEA/100)  # floor(Longitude / 100)
    elif type(NMEA) == np.ndarray:
        deg = np.floor(NMEA/100)
    else:
        print('NMEA must be float or numpy.ndarray')
        return

    min = NMEA - deg*100  # (Longitude - floor(Longitude / 100) * 100)
    return deg + min/60


def deg2m(lon, lat):
    # This formula is based on the equation in file "Longitude and Latitude
    # Conversion Table.pdf"
    a = 6378137.0   # semi-major axis of WGS-84 ellipsoid
    b = 6356752.3142    # semi-minor axis of WGS-84 ellipsoid
    h = 0.0  # height above ellipsoid's surface

    # convert to radians for trig functions
    angles = np.radians(np.floor(lat))
    # What's under the square root
    c = a**2 * np.power(np.cos(angles), 2) + b**2 * np.power(np.sin(angles), 2)
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


# Plot the position data (x and y) in the same plot, which should give you the
# path taken by the car (this is the same path I showed you in the lectures)

# Extent adjusts the position of the image in the plot.
# Attempt to find a better solution if possible

extent = [802750, 804300, 6308850, 6310340]
map = image.imread('halmstad_drive_area.gif')
fig, ax = plt.subplots()
# TODO Fix extent (adapt automatically if possible)
ax.imshow(map, extent=extent)


# Read the NMEA-0183 data
DATA = np.loadtxt('gps_ex1_morningdrive.txt')
Longitude = DATA[:, 3]
Latitude = DATA[:, 2]

# Convert to degrees
LongDeg = NMEA2deg(Longitude)
LatDeg = NMEA2deg(Latitude)

# Convert to meters and plot
LongM, LatM = deg2m(LongDeg, LatDeg)
ax.plot(LongM, LatM, 'b-')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('The route taken by the car')
fig.autofmt_xdate()
plt.show()

# Plot the speed data
# In m/s
# Assuming the time between measurements is 1 second. In this case the distance
# in meters is equal to the velocity in m/s.
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

# Mark the maximum speed with a red circle
max_speed = max(kmh_speeds)
max_speed_index = kmh_speeds.index(max_speed)
plt.plot(max_speed_index, max_speed, 'ro', fillstyle='none')
plt.show()

# Q9: Calculate the maximum speed taken by the driver.
print("Maximum Speed is: ", max_speed, "km/h")

# Q10: Did I ever break the speed limits, i.e. 70 km/h?

# Yes, here are the positions plotted on the map
above_limit_x = []
above_limit_y = []
for i in range(len(kmh_speeds)-1):
    if kmh_speeds[i] >= 70:
        above_limit_x.append(LongM[i])
        above_limit_y.append(LatM[i])

fig, ax = plt.subplots()
#ax.imshow(map, extent=extent)
ax.plot(LongM, LatM, 'b-')
ax.plot(above_limit_x, above_limit_y, 'ro')
plt.show()

# Q11: How come the estimate in speed is so accurate while the estimate in
# position is not? This is very important! Recall what you find out about the
# GPS error in the first part of the exercise.

# Plot the headings
# Calculate the headings (and plot them with respect to time) along the path
# (see compendium http://adamchukpa.mcgill.ca/web_ssm/web_GPS.html).
# Note that the the matlab function atan2d() can be used here instead of the
# "if statments" in the compendium.
heading = []
for i in range(len(LongM)-1):
    heading.append(
        (math.atan2(LatM[i+1] - LatM[i], LongM[i+1] - LongM[i])*180)/math.pi)

straight_heading = heading[240:315]

# Plot the headings between Laholm and Vrang in red, and the rest in blue
plt.plot(heading, 'b-')
plt.plot(range(240, 315), straight_heading, 'r-')
plt.xlabel('Time (s)')
plt.ylabel('Heading (degrees)')
plt.title("Heading over time")
plt.show()

# Q12: Can you calculate the error in headings? Tips! If you only consider the
# headings along the straight-line path (along Laholmsvägen or Vrangelsleden)
# can you then estimate the variance in the heading?
plt.plot(straight_heading, 'r-')
plt.xlabel('Time (s)')
plt.ylabel('Heading (degrees)')
plt.title("Heading over time, between Laholmsvägen or Vrangelsleden \n (straight-line path)")
plt.show()

# Calculate the variance in the heading along the straight-line path
variance = np.var(straight_heading)
print("Variance in heading is: ", variance)

# Calculate the error in heading based on the straight-line path
mean = np.mean(straight_heading)
errors = straight_heading - mean

# Plot the error in heading
plt.plot(errors, 'r-')
plt.xlabel('Time (s)')
plt.ylabel('Error in heading (degrees)')
plt.title("Error in heading over time, between Laholmsvägen or Vrangelsleden \n (straight-line path)")
plt.show()

# Mark the heading values used for calculating the heading variance in the
# heading plot and the plot at the position plot above.
# Tips use the data points between 240:315.
plt.plot(LongM, LatM, 'b-')
plt.plot(LongM[240:315], LatM[240:315], 'r-')
plt.title("Position plot with heading variance marked")
plt.show()