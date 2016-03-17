"""
Copyright (c) 2015, The Regents of the University of California (Regents).
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Please contact the author(s) of this library if you have any questions.
Author: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
"""

###########################################################################
#
# Surface fitting library. Use Gaussian Process regression to fit a surface
# to a set of points in 3D.
#
###########################################################################

import matplotlib.pyplot as plt
import numpy as np
from point import Point3D, Point2D

class EstimateSurface:
    """
    Generate covariance matrices and compute conditional expectation
    and covariance for signed distance function.
    """
    def __init__(self, training_points, training_dists, gamma, noise_sd):
        self.K11_ = self.Covariance(training_points, gamma, noise_sd)
        self.K11_inv_ = np.linalg.inv(self.K11_)
        self.mu_training_ = np.asarray(training_dists)
        self.mu_training_ = np.asmatrix(self.mu_training_).T

    def RBF(self, p1, p2, gamma):
        """
        Compute the RBF covariance kernel for two points.
        i.e. exp(- gamma * ||p1 - p2||_2^2)
        """
        return np.exp(-gamma * p1.SquaredDistanceTo(p2))

    def Covariance(self, points, gamma, noise_sd=0.0):
        """
        Compute covariance matrix for a list of points, using the RBF kernel.
        """
        K = np.zeros((len(points), len(points)))
        for ii, p1 in enumerate(points):
            for jj, p2 in enumerate(points):
                K[ii, jj] = self.RBF(p1, p2, gamma)

        return np.asmatrix(K + noise_sd*noise_sd*np.eye(len(points)))

    def CrossCovariance(self, points1, points2, gamma):
        """
        Compute cross covariance matrix for two lists of points.
        """
        K = np.zeros((len(points1), len(points2)))
        for ii, p1 in enumerate(points1):
            for jj, p2 in enumerate(points2):
                K[ii, jj] = self.RBF(p1, p2, gamma)

        return np.asmatrix(K)


    def SignedDistance(self, query, gamma, noise_sd):
        K22 = self.Covariance([query], gamma, noise_sd)
        K12 = self.CrossCovariance(training_points, [query], gamma)
        K21 = K12.T

        # Gaussian conditioning.
        mu_query = K21 * self.K11_inv_ * self.mu_training_
        K_query = K22 - K21 * self.K11_inv_ * K12

        return (mu_query, K_query)

    def CovarianceToUncertainty(self, K):
        """
        Return uncertainty -- essentially just the diagonal of the covariance matrix.
        """
        uncertainty = []
        for ii in range(K.shape[0]):
            uncertainty.append(K[ii, ii])

        return uncertainty

# --------------------------------- MAIN FUNCTION ------------------------------ #

if __name__ == "__main__":
    N = 20  # number of points
    R = 1.0   # radius of sphere
    BOX = 2.0 # bounding box
    RES = 0.1 # plot resolution
    SD = 0.1  # noise standard deviation
    GAMMA = 10.0 # RBF parameter

    # Generate a bunch of points on a sphere of radius R.
    training_points = []
    training_dists = []
    for ii in range(N):
        theta = np.random.uniform(0.0, 2.0*np.pi)
        x = R * np.cos(theta)
        y = R * np.sin(theta)

        # Compute a point inside and a point outside the sphere.
        p_inside = Point2D(0.9*x, 0.9*y)
        training_points.append(p_inside)
        training_dists.append(-0.1*R)

        p_outside = Point2D(1.1*x, 1.1*y)
        training_points.append(p_outside)
        training_dists.append(0.1*R)

    # Create a new EstimateSurface object.
    gp = EstimateSurface(training_points, training_dists, GAMMA, SD)

    # Generate a meshgrid and plot the isosurface.
    x, y = np.meshgrid(np.arange(-BOX, BOX, RES), np.arange(-BOX, BOX, RES))
    distances = np.zeros(x.shape)
    errors = np.zeros(x.shape)
    for ii in range(x.shape[0]):
        for jj in range(x.shape[1]):
            query = Point2D(x[ii, jj], y[ii, jj])
            dist, err = gp.SignedDistance(query, GAMMA, SD)
            distances[ii, jj] = dist
            errors[ii, jj] = err

    plt.figure()
    cs = plt.contour(x, y, distances, [-0.1, 0, 0.1])
    plt.colorbar()

    # Plot the training points.
    for p in training_points:
        plt.plot(p.x_, p.y_, 'ro')

    plt.show()
