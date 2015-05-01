/*
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, University of California, Berkeley
# All rights reserved.
# Authors: Cameron Lee (cameronlee@berkeley.edu) and Dmitry Berenson (
        berenson@eecs.berkeley.edu)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of California, Berkeley nor the names
of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * This file contains several utility functions used by the CollisionChecker
 * class.
 */

#ifndef COLLISION_UTILS_H_
#define COLLISION_UTILS_H_

#include <ros/ros.h>
#include <math.h>
#include <vector>

/**
 * Utility for getting a point a distance of dist from p1 along the line between
 * p1 and p2.
 *
 * @param p1 Vector of first point.
 * @param p2 Vector for second point; should be same length as p1.
 * @param dist Distance from p1 to generate the returned point.
 * @returns A vector of same size as p1 that is on the line between p1 and p2.
 */
std::vector<double> getPointBetween(const std::vector<double> &p1,
                                    const std::vector<double> &p2, double dist);

/**
 * Takes path and interpolates points between consecutive points such that the
 * returned path has points at intervals of step_size.
 *
 * @param path Path to be discretized.
 * @param step_size Maximum step between resultant points.
 * @returns A path with points separated by at most step_size and passes through
 *   the points of path.
 */
std::vector<std::vector<double> > rediscretizePath(
    const std::vector<std::vector<double> > &path, double step_size);

/**
 * Takes to angles and returns the absolute value of the difference such that it
 * is between 0 and pi.
 *
 * @param a An angle, in radians.
 * @param b Another angle, in radians.
 * @returns The absolute value of the difference between a and b, bound to [0, PI).
 */
double getAngleBetween(double a, double b);

/**
 * Returns the distance between two N-Dimensional points.
 *
 * @param p1 A point reprsented by a vector.
 * @param p2 Another point, with the same size as p1.
 * @returns The distance between p1 and p2.
 */
double getLineDistance(const std::vector<double> &p1,
                       const std::vector<double> &p2);

/**
 * Goes with getAngleBetween(); essentially, it determines the sign of the angle
 * that is returned by getAngleBetween().
 *
 * @param a An angle, in radians.
 * @param b Another angle, in radians.
 * @returns The sign of (b - a) when scaled to -PI to +PI.
 */
int getDirectionMultiplier(double a, double b);

/**
 * Interpolates between two points with a distance between each resultant point
 * of step_size.
 *
 * @param p1 First point to interpolate from.
 * @param p2 Vector of same length as p1.
 * @param step_size distance between points in generated interpolation.
 * @returns a vector of points representing the line between the two points.
 */
std::vector<std::vector<double> > interpolate(const std::vector<double> &p1,
                                              const std::vector<double> &p2,
                                              double step_size);

#endif
