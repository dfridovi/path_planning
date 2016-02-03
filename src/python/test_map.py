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
# Test script. Create a bunch of random 2D points, for each one generate
# noisy observations and update, then visualize.
#
###########################################################################

import numpy as np
from landmark import Landmark
from map import Map

import matplotlib.pyplot as plt

NUM_POINTS = 10
NUM_OBS = 10

# Initialize the map and an empty list to keep track of landmarks.
m = Map()
landmarks = []
positions = []

# Generate random landmarks.
for ii in range(NUM_POINTS):
    position = np.matrix([np.random.randn(),
                          np.random.randn()]).T
    jittered = position + \
               2.0 * np.matrix([np.random.randn(),
                                np.random.randn()]).T
    landmark = Landmark(jittered)
    m.AddLandmark(landmark)
    landmarks.append(landmark)
    positions.append(position)

# Visualize the map.
print "Map includes %d points." % m.Size()
fig1 = m.Visualize2D()

# Generate random noise at each landmark.
for landmark, position in zip(landmarks, positions):
    for ii in range(NUM_OBS):
        jittered = position + \
                   1.0 * np.matrix([np.random.randn(),
                                    np.random.randn()]).T
        landmark.SetLocation(jittered)
        m.UpdateLandmark(landmark)

# Visualize again.
fig2 = m.Visualize2D()
plt.show()
