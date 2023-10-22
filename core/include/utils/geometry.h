#pragma once
#include <cmath>
#include "../core/include/utils/math_util.h"

/**
 * Data structure representing an X,Y coordinate
 */
typedef struct point_s
{
    double x; ///< the x position in space
    double y; ///< the y position in space

    /**
     * dist calculates the euclidian distance between this point and another point using the pythagorean theorem
     * @param other the point to measure the distance from
     * @return the euclidian distance between this and other
     */
    double dist(const point_s other) const;

    /**
     * Vector2D addition operation on points
     * @param other the point to add on to this
     * @return this + other (this.x + other.x, this.y + other.y)
     */
    point_s operator+(const point_s &other) const;

    /**
     * Vector2D subtraction operation on points
     * @param other the point_t to subtract from this
     * @return this - other (this.x - other.x, this.y - other.y)
     */
    point_s operator-(const point_s &other) const;
    

    point_s operator*(double s) const;
    
    point_s operator/(double s) const;

    point_s operator-() const;

    point_s operator+() const;

    bool operator==(const point_s &rhs);
} point_t;

typedef struct point3_s
{
    double x, y, z;

    bool operator==(const point3_s &rhs);
    
    // TODO Math functions, if needed in the future

} point3_t;

/**
 *  Describes a single position and rotation
 */
struct pose_t
{
    double x;   ///< x position in the world
    double y;   ///< y position in the world
    double rot; ///< rotation in the world

    point_t get_point();

} ;

struct Rect
{
    point_t min;
    point_t max;
    static Rect from_min_and_size(point_t min, point_t size);

    point_t dimensions() const;
    point_t center() const;
    double width() const;
    double height() const;
    bool contains(point_t p) const;

};

struct Mat2
{
    double X11, X12;
    double X21, X22;
    point_t operator*(const point_t p) const;

    static Mat2 FromRotationDegrees(double degrees);
};

/**
 * 3 dimensional matrix type, useful for transformations.
*/
struct Mat3
{
    double X11, X12, X13;
    double X21, X22, X23;
    double X31, X32, X33;

    /**
     * Multipily by a 3x1 matrix (aka point3)
     */
    point3_t operator*(const point3_t rhs) const;

    /**
     * Multiply by another 3X3 matrix
    */
    Mat3 operator*(const Mat3 rhs) const;

};

/**
 * Returns a rotation matrix that can then be multiplied by a point3_t. For example:
 * 
 * \code{.cpp}
 * point3_t orient = {1, 2, 3};
 * point3_t reorient = get_rotation_matrix(xaxis, 90) * orient;
 * \endcode
 * 
 * Source: \link https://austinmorlan.com/posts/rotation_matrices/ \endlink
 * 
 * @param axis axis to rotate about
 * @param degrees angle to rotate; Follow right hand rule for pos / neg
 * @return rotation matrix for multiplication
*/
Mat3 get_rotation_matrix(vex::axisType axis, double degrees);

Mat3 get_swapaxis_matrix(vex::axisType a1, vex::axisType a2);

inline constexpr Mat3 notransform_matrix = {
    .X11 = 1, .X12 = 0, .X13 = 0,
    .X21 = 0, .X22 = 1, .X23 = 0,
    .X31 = 0, .X32 = 0, .X33 = 1
};