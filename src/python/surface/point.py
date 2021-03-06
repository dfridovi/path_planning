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
# 2D/3D point classes.
#
###########################################################################

import numpy as np

class Point3D:
    def __init__(self):
        self.x_ = 0
        self.y_ = 0
        self.z_ = 0

    def __init__(self, x, y, z):
        self.x_ = x
        self.y_ = y
        self.z_ = z

    def SquaredDistanceTo(self, other):
        dx = self.x_ - other.x_
        dy = self.y_ - other.y_
        dz = self.z_ - other.z_
        return dx*dx + dy*dy + dz*dz

class Point2D:
    def __init__(self):
        self.x_ = 0
        self.y_ = 0

    def __init__(self, x, y):
        self.x_ = x
        self.y_ = y

    def SquaredDistanceTo(self, other):
        dx = self.x_ - other.x_
        dy = self.y_ - other.y_
        return dx*dx + dy*dy
