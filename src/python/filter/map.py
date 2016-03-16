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
# Map class to test the filtering approach to mapping.
#
###########################################################################

import numpy as np
from numpy import matlib
import matplotlib.pyplot as plt

from landmark import Landmark

class Map:

    # Constructor.
    def __init__(self):
        self.size_ = 0
        self.registry_ = {}

    # Add a landmark.
    def AddLandmark(self, p):
        if p.GetID() in self.registry_:
            print "This landmark is already in the map. Did not add."
            return

        # Add to state vector and assign identiy covariance.
        position = p.GetLocation()
        if self.size_ == 0:
            self.point_size_ = len(position)
            self.state_ = position
            self.covariance_ = np.matlib.eye(self.point_size_)
        elif len(position) != self.point_size_:
            print "Point size does not match. Did not add."
            return
        else:
            self.state_ = np.vstack([self.state_, position])
            old_covariance = self.covariance_
            self.covariance_ = np.matlib.eye(old_covariance.shape[0] +
                                             self.point_size_)
            self.covariance_[:-self.point_size_,
                             :-self.point_size_] = old_covariance

        # Update the registry.
        self.registry_[p.GetID()] = self.size_
        self.size_ += 1

    # Update a landmark. This is a pure Kalman update.
    def UpdateLandmark(self, p):
        if p.GetID() not in self.registry_:
            print "This landmark is not in the registry. Did not update."
            return

        # Extract index and position.
        index = self.registry_[p.GetID()]
        position = p.GetLocation()

        # Generate observation vector z.
        z = np.matlib.zeros(self.state_.shape)
        z[index*self.point_size_:(index + 1)*self.point_size_] = position

        # Generate measurement matrix H.
        H = np.matlib.zeros(self.covariance_.shape)
        H[index*self.point_size_:(index + 1)*self.point_size_,
          index*self.point_size_:(index + 1)*self.point_size_] = \
                                            np.matlib.eye(self.point_size_)

        # Generate measurement covariance R.
        R = np.matlib.zeros(self.covariance_.shape)
        np.fill_diagonal(R, float("inf"))
        R[index*self.point_size_:(index + 1)*self.point_size_,
          index*self.point_size_:(index + 1)*self.point_size_] = \
                                            np.matlib.eye(self.point_size_)

        # Calculate innovation residual y and covariance S.
        y = z - H * self.state_
        S = H * self.covariance_ * H.T + R

        # Calculate Kalman gain and posteriors.
        K = self.covariance_ * H.T * np.linalg.inv(S)
        self.state_ = self.state_ + K * y
        self.covariance_ = (np.matlib.eye(len(z)) - K * H) * self.covariance_

    # Visualize as a scatterplot.
    def Visualize2D(self):
        if self.point_size_ != 2:
            print "Points must be in 2D."
            return

        x_coordinates = np.zeros(len(self.state_) / 2)
        x_coordinates[:] = self.state_[0:len(self.state_):2].flatten()
        y_coordinates = np.zeros(len(self.state_) / 2)
        y_coordinates[:] = self.state_[1:len(self.state_):2].flatten()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(x_coordinates, y_coordinates, color="green")
        return fig

    # Visualize as a scatterplot.
    def VisualizeLines2D(self, true_positions):
        if self.point_size_ != 2:
            print "Points must be in 2D."
            return

        if len(true_positions) != self.size_:
            print "Incorrect number of true positions."
            return

        # Extract estimated coordinates.
        x_coordinates = np.zeros(len(self.state_) / 2)
        x_coordinates[:] = self.state_[0:len(self.state_):2].flatten()
        y_coordinates = np.zeros(len(self.state_) / 2)
        y_coordinates[:] = self.state_[1:len(self.state_):2].flatten()

        # Extract true coordinates.
        true_x = np.zeros(len(self.state_) / 2)
        true_y = np.zeros(len(self.state_) / 2)
        for ii, position in enumerate(true_positions):
            true_x[ii] = position[0]
            true_y[ii] = position[1]

        # Plot.
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(x_coordinates, y_coordinates, color="green")
        ax.scatter(true_x, true_y, color="red")
        for ii in range(self.size_):
            ax.plot([true_x[ii], x_coordinates[ii]],
                    [true_y[ii], y_coordinates[ii]], 'b-', lw=2)
        return fig

    # Getters.
    def Size(self):
        return self.size_

    def PointSize(self):
        return self.point_size_

    def State(self):
        return self.state_

    def Covariance(self):
        return self.covariance_
