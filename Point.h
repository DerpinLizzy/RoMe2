/*
 * Point.h
 * Copyright (c) 2025, ZHAW
 * All rights reserved.
 */

#ifndef POINT_H_
#define POINT_H_

#include <cstdlib>

/**
 * This class stores the coordinates of a point in two dimensions.
 */
class Point {
    
    public:
        
        float           x;
        float           y;
        
                        Point();
                        Point(float x, float y);
        virtual         ~Point();
        float           distance();
        float           distance(Point& point);
        float           manhattanDistance();
        float           manhattanDistance(Point& point);
};

#endif /* POINT_H_ */
