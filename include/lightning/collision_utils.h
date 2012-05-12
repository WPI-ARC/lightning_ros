#ifndef COLLISION_UTILS_H_
#define COLLISION_UTILS_H_

#include <ros/ros.h>
#include <math.h>
#include <vector>

std::vector<double> getPointBetween(const std::vector<double> &p1, const std::vector<double> &p2, double dist);
std::vector< std::vector<double> > rediscretizePath(const std::vector< std::vector<double> > &path, double step_size);
double getAngleBetween(double a, double b);
double getLineDistance(const std::vector<double> &p1, const std::vector<double> &p2);
int getDirectionMultiplier(double a, double b);
std::vector< std::vector<double> > interpolate(const std::vector<double> &p1, const std::vector<double> &p2, double step_size);

#endif
