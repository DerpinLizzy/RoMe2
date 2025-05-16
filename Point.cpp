/*
 * Point.cpp
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#include <cmath>
#include "Point.h"

using namespace std;

/**
 * Creates a Point object.
 */
Point::Point() {
    
    x = 0.0f;
    y = 0.0f;
}

/**
 * Creates a Point object from given Cartesian coordinates.
 * @param x the X coordinate of the point.
 * @param y the Y coordinate of the point.
 */
Point::Point(float x, float y) {
    
    this->x = x;
    this->y = y;
}

/**
 * Deletes this object.
 */
Point::~Point() {}

/**
 * Calculates the distance of this point from the origin.
 */
float Point::distance() {
    
    return sqrt(x*x+y*y);
}

/**
 * Calculates the distance between this point and a given point.
 * @param point another point to calculate the distance to.
 */
float Point::distance(Point& point) {
    
    return sqrt((x-point.x)*(x-point.x)+(y-point.y)*(y-point.y));
}

/**
 * Calculates the manhattan distance of this point from the origin.
 * This calculation is only an approximation, but a lot faster to compute.
 */
float Point::manhattanDistance() {
    
    return fabs(x)+fabs(y);
}

/**
 * Calculates the manhattan distance between this point and a given point.
 * This calculation is only an approximation, but a lot faster to compute.
 * @param point another point to calculate the distance to.
 */
float Point::manhattanDistance(Point& point) {
    
    return fabs(x-point.x)+fabs(y-point.y);
}
