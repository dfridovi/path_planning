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

def RBF(p1, p2, gamma):
    """
    Compute the RBF covariance kernel for two points.
    i.e. exp(- gamma * ||p1 - p2||_2^2)
    """
    return np.exp(-gamma * p1.SquaredDistanceTo(p2))

def Covariance(points, gamma):
    """
    Compute covariance matrix for a list of points, using the RBF kernel.
    """
    K = np.zeros((len(points), len(points)))
    for ii, p1 in enumerate(points):
        for jj, p2 in enumerate(points):
            K[ii, jj] = RBF(p1, p2, gamma)

    return np.asmatrix(K)

def CrossCovariance(points1, points2):
    """
    Compute cross covariance matrix for two lists of points.
    """
    K = np.zeros((len(points1), len(points2)))
    for ii, p1 in enumerate(points1):
        for jj, p2 in enumerate(points2):
            K[ii, jj] = RBF(p1, p2, gamma)

    return np.asmatrix(K)

def EstimateDistance(training_points, training_dists, query_points):
    """
    Generate covariance matrices and compute conditional expectation
    and covariance for signed distance function.
    """


