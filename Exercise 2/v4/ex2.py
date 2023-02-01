import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from sympy import symbols, Matrix


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
    ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
                  color='red', lw=2, zorder=2)
    ell.set_facecolor('none')
    plt.gca().add_artist(ell)

    # Plotting the data points
    plt.plot(data[:, 0], data[:, 1], 'b-', zorder=1)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(Title)

    # Scatter plot the first 10 of the data points and mark them as .
    plt.scatter(data[:10, 0], data[:10, 1], marker='.', color='red', zorder=3)
    plt.scatter(data[101:110, 0], data[101:110, 1], marker='.', color='red',
                zorder=3)

    # Find the max point and mark it as non-filled circle
    max_point = np.argmax(np.sqrt(np.power(data[:, 0], 2)+np.power(data[:, 1],
                                                                   2)))
    plt.scatter(data[max_point, 0], data[max_point, 1], marker='o',
                facecolors='none', edgecolors='red', zorder=4)

    # Generate 10 random points with the same variance as X and Y and plot
    # them as x
    random_points = np.random.multivariate_normal(mean, cov_matrix, 10)
    plt.scatter(random_points[:, 0], random_points[:, 1], marker='x',
                color='red', zorder=5)

    # Print the value of the max point
    print('Max point: ', data[max_point, :])
    print('value:', np.sqrt(np.power(data[max_point, 0], 2) +
                            np.power(data[max_point, 1], 2)))

    plt.show()


# Example usage of calculating partial derivatives with sympy (jacobian)
x, y, z = symbols('x y z', real=True)
X = Matrix([x*y*z, y**2, x + z])
Y = Matrix([x, y, z])

print(X.jacobian(Y))
