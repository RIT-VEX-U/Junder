#include "../core/include/utils/geometry.h"

double point_s::dist(const point_s other) const
{
    return std::sqrt(std::pow(this->x - other.x, 2) + pow(this->y - other.y, 2));
}

point_s point_s::operator+(const point_s &other) const
{
    point_s p{
        .x = this->x + other.x,
        .y = this->y + other.y};
    return p;
}

point_s point_s::operator-(const point_s &other) const
{
    point_s p{
        .x = this->x - other.x,
        .y = this->y - other.y};
    return p;
}

point_s point_s::operator*(double s) const
{
    return {x * s, y * s};
}

point_s point_s::operator/(double s) const
{
    return {x / s, y / s};
}

point_s point_s::operator-() const
{
    return {-x, -y};
}

point_s point_s::operator+() const
{
    return {x, y};
}

bool point_s::operator==(const point_s &rhs)
{
    return x == rhs.x && y == rhs.y;
}

bool point3_s::operator==(const point3_s &rhs)
{
    return x==rhs.x && y==rhs.y && z==rhs.z;
}

point_t pose_t::get_point()
{
    return point_t{.x = x, .y = y};
}

Rect Rect::from_min_and_size(point_t min, point_t size)
{
    return {min, min+size};
}

point_t Rect::dimensions() const
{
    return max - min;
}

point_t Rect::center() const
{
    return (min + max)/2;
}

double Rect::width() const
{
    return max.x - min.x;
}

double Rect::height() const
{
    return max.y - min.y;
}

bool Rect::contains(point_t p) const
{
    bool xin = p.x > min.x && p.x < max.x;
    bool yin = p.y > min.y && p.y < max.y;
    return xin && yin;
}

point_t Mat2::operator*(const point_t p) const
{
    double outx = p.x * X11 + p.y * X12;
    double outy = p.x * X21 + p.y * X22;
    return {outx, outy};
}

Mat2 Mat2::FromRotationDegrees(double degrees)
{
    double rad = degrees * (M_PI / 180.0);
    double c = cos(rad);
    double s = sin(rad);
    return {c, -s, s, c};
}

/**
 * Multipily by a 3x1 matrix (aka point3)
 */
point3_t Mat3::operator*(const point3_t rhs) const
{
    double x=rhs.x, y=rhs.y, z=rhs.z;
    return point3_t {
        .x = (X11 * x) + (X12 * y) + (X13 * z),
        .y = (X21 * x) + (X22 * y) + (X23 * z),
        .z = (X31 * x) + (X32 * y) + (X33 * z)
    };
}

/**
 * Multiply by another 3X3 matrix
*/
Mat3 Mat3::operator*(const Mat3 rhs) const
{
    return Mat3
    {
        // Row 1
        .X11 = (X11 * rhs.X11) + (X12 * rhs.X21) + (X13 * rhs.X31), 
        .X12 = (X11 * rhs.X12) + (X12 * rhs.X22) + (X13 * rhs.X32),
        .X13 = (X11 * rhs.X13) + (X12 * rhs.X23) + (X13 * rhs.X33),

        // Row 2
        .X21 = (X21 * rhs.X11) + (X22 * rhs.X21) + (X23 * rhs.X31),
        .X22 = (X21 * rhs.X12) + (X22 * rhs.X22) + (X23 * rhs.X32),
        .X23 = (X21 * rhs.X13) + (X22 * rhs.X23) + (X23 * rhs.X33),

        // Row 2
        .X31 = (X31 * rhs.X11) + (X32 * rhs.X21) + (X33 * rhs.X31),
        .X32 = (X31 * rhs.X12) + (X32 * rhs.X22) + (X33 * rhs.X32),
        .X33 = (X31 * rhs.X13) + (X32 * rhs.X23) + (X33 * rhs.X33),
    };
}


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
Mat3 get_rotation_matrix(vex::axisType axis, double degrees)
{
    switch(axis)
    {
        case vex::axisType::xaxis:
            return Mat3{
                .X11 = 1, .X12 = 0,                     .X13 = 0,
                .X21 = 0, .X22 = cos(deg2rad(degrees)), .X23 = -sin(deg2rad(degrees)),
                .X31 = 0, .X32 = sin(deg2rad(degrees)), .X33 = cos(deg2rad(degrees))
            };
        case vex::axisType::yaxis:
            return Mat3{
                .X11 = cos(deg2rad(degrees)),  .X12 = 0, .X13 = sin(deg2rad(degrees)),
                .X21 = 0,                      .X22 = 1, .X23 = 0,
                .X31 = -sin(deg2rad(degrees)), .X32 = 0, .X33 = cos(deg2rad(degrees))
            };
        case vex::axisType::zaxis:
            return Mat3{
                .X11 = cos(deg2rad(degrees)), .X12 = -sin(deg2rad(degrees)), .X13 = 0,
                .X21 = sin(deg2rad(degrees)), .X22 = cos(deg2rad(degrees)),  .X23 = 0,
                .X31 = 0,                     .X32 = 0,                      .X33 = 1
            };        
    }
}

/**
 * Get a 3 dimensional matrix, that when multiplied by a X, Y and Z
 * orientation, swaps the axis in question.
*/
Mat3 get_swapaxis_matrix(vex::axisType a1, vex::axisType a2)
{
    // Swap X and Y axis
    if ((a1 == vex::axisType::xaxis && a2 == vex::axisType::yaxis) || 
        (a1 == vex::axisType::yaxis && a2 == vex::axisType::xaxis))
    {
        return Mat3 {
            .X11=0, .X12=1, .X13=0,
            .X21=1, .X22=0, .X23=0,
            .X31=0, .X32=0, .X33=1
        };
    }

    // Swap X and Z axis
    if ((a1 == vex::axisType::xaxis && a2 == vex::axisType::zaxis) || 
        (a1 == vex::axisType::zaxis && a2 == vex::axisType::xaxis))
    {
        return Mat3 {
            .X11=0, .X12=0, .X13=1,
            .X21=0, .X22=1, .X23=0,
            .X31=1, .X32=0, .X33=0
        };
    }

    // Swap Y and Z axis
    if ((a1 == vex::axisType::yaxis && a2 == vex::axisType::zaxis) || 
        (a1 == vex::axisType::zaxis && a2 == vex::axisType::yaxis))
    {
        return Mat3 {
            .X11=1, .X12=0, .X13=0,
            .X21=0, .X22=0, .X23=1,
            .X31=0, .X32=1, .X33=0
        };
    }

    // If the same axis is input twice, don't transform
    return notransform_matrix;
}