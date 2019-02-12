#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <vector>
#include "hw2_types.h"
#include "hw2_math_ops.h"
#include "hw2_file_ops.h"
#include <iostream>
#include <algorithm>

#include <cassert>

// Globals
Color **image;

Camera cameras[100];
int numberOfCameras = 0;

Model models[1000];
int numberOfModels = 0;

Color colors[100000];
int numberOfColors = 0;

Translation translations[1000];
int numberOfTranslations = 0;

Rotation rotations[1000];
int numberOfRotations = 0;

Scaling scalings[1000];
int numberOfScalings = 0;

Vec3 vertices[100000];
int numberOfVertices = 0;

Color backgroundColor;

// backface culling setting, default disabled
int backfaceCullingSetting = 0;


// Have to keep new vertex coordinates after transformations.
Vec3 triangleVertices[100000][3];

// After his r = m.
void equalizeMatrices(double r[4][4], double m[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            r[i][j] = m[i][j];
        }
    }
}

// After this r is a zero matrix.
void makeZeroMatrix(double r[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            r[i][j] = 0;
        }
    }
}

/**
 * @brief Namespace for modeling transformations.
 */
namespace model_tr {

/**
 * @brief Translate the 4D matrix r by the
 *   translation t and store the result in r
 *
 * @param r[4][4]
 * @param t
 */
void getTranslationMatrix(double r[4][4], Translation t) {
    makeIdentityMatrix(r);
    r[0][3] = t.tx;
    r[1][3] = t.ty;
    r[2][3] = t.tz;
}



/**
 * @brief Scale the 4D matrix r by the scaling factors in
 * s.
 *
 * @param r[4][4]
 * @param s
 */
void getScalingMatrix(double r[4][4], Scaling s) {
    makeIdentityMatrix(r);
    r[0][0] = s.sx;
    r[1][1] = s.sy;
    r[2][2] = s.sz;
}

/**
 * @brief Helper for getRotationMatrix.
 */
void getRotationXMatrix(double r[4][4], Rotation rot, bool sign) {
    double b = rot.uy;
    double c = rot.uz;
    double d = std::sqrt(b * b + c * c);

    if (d == 0) {
        makeIdentityMatrix(r);
        return;
    }

    r[1][1] = c / d;
    r[1][2] = (-b) / d;

    r[2][1] = b / d;
    r[2][2] = c / d;

    if (!sign) {
        r[1][2] = -r[1][2];
        r[2][1] = -r[2][1];
    }

}

/**
 * @brief Helper for getRotationMatrix.
 */
void getRotationYMatrix(double r[4][4], Rotation rot, bool sign) {
    double b = rot.uy;
    double c = rot.uz;
    double d = std::sqrt(b * b + c * c);

    r[0][0] = d;
    r[0][2] = (rot.ux);
    r[2][0] = -rot.ux;
    r[2][2] = d;

    if (!sign) {
        r[0][2] = -r[0][2];
        r[2][0] = -r[2][0];
    }
}

/**
 * @brief Helper for getRotationMatrix.
 */
void getRotationZMatrix(double r[4][4], Rotation rot) {

    double theta = (rot.angle * M_PI) / 180;
    r[0][0] = std::cos(theta);
    r[0][1] = -std::sin(theta);
    r[1][0] = std::sin(theta);
    r[1][1] = std::cos(theta);
}

/**
 * @brief Get the rotation matrix according to the
 * rotation rot.
 *
 * @param r[4][4]
 * @param rot
 */
void getRotationMatrix(double r[4][4], Rotation rot) {
    makeIdentityMatrix(r);
    double rotx[4][4], roty[4][4], rotz[4][4];
    double i1[4][4], i2[4][4], i3[4][4];

    makeIdentityMatrix(rotx);
    makeIdentityMatrix(roty);
    makeIdentityMatrix(rotz);

    getRotationXMatrix(rotx, rot, false);
    getRotationYMatrix(roty, rot, true);
    getRotationZMatrix(rotz, rot);

    multiplyMatrixWithMatrix(i1, rotx, roty);
    multiplyMatrixWithMatrix(i2, i1, rotz);

    getRotationXMatrix(rotx, rot, true);
    getRotationYMatrix(roty, rot, false);

    multiplyMatrixWithMatrix(i3, i2, roty);
    multiplyMatrixWithMatrix(r, i3, rotx);

}

}; // namespace model_tr

/**
 * @brief Namespace for rasterization functions.
 */
namespace rasterize {

/**
 * @brief Namespace for drawing pixels, lines and triangles.
 */
namespace draw {

/**
 * @brief Draw a pixel on the screen
 *
 * @param x x-coordinate of the image plane
 * @param y y-coordinate of the image plane
 * @param c The color to write
 */
void draw_pixel(int x, int y, Color c) {
    image[x][y].r = c.r > 255 ? 255 : c.r;
    image[x][y].g = c.g > 255 ? 255 : c.g;
    image[x][y].b = c.b > 255 ? 255 : c.b;
}

/**
 * @brief Draw a line with [0,1) slope.
 *
 * @param p A Vec3
 * @param q Another Vec3
 */
void draw_line_v1(Vec3 p, Vec3 q) {

    int x1 = static_cast<int>(q.x);
    int y1 = static_cast<int>(q.y);
    int x0 = static_cast<int>(p.x);
    int y0 = static_cast<int>(p.y);
    int y = y0;

    if (x0 > x1) {
        draw_line_v1(q, p);
        return;
    }

    double d = (y0 - y1) + 0.5 * (x1 - x0);

    Color c0, c1;
    c0.r = colors[p.colorId].r;
    c0.g = colors[p.colorId].g;
    c0.b = colors[p.colorId].b;

    c1.r = colors[q.colorId].r;
    c1.g = colors[q.colorId].g;
    c1.b = colors[q.colorId].b;

    Color c = c0;

    // Directly compute color increment
    Color dc;
    dc.r = (c1.r - c0.r) / (x1 - x0);
    dc.g = (c1.g - c0.g) / (x1 - x0);
    dc.b = (c1.b - c0.b) / (x1 - x0);

    for (int x = x0; x < x1; x++) {

        draw_pixel(x, y, {
                std::round(c.r),
                std::round(c.g),
                std::round(c.b)
                });

        if (d < 0) {
            y = y + 1;
            d += (y0 - y1) + (x1 - x0);
        } else {
            d += (y0 - y1);
        }
        // Update color
        c.r += dc.r;
        c.g += dc.g;
        c.b += dc.b;
    }
} 

/**
 * @brief Draw a line with [1,+inf) slope.
 *
 * @param p A Vec3
 * @param q Another Vec3
 */
void draw_line_v2(Vec3 p, Vec3 q) {
    int x1 = static_cast<int>(q.x);
    int y1 = static_cast<int>(q.y);
    int x0 = static_cast<int>(p.x);
    int y0 = static_cast<int>(p.y);
    int x = x0;
    
    if (x0 > x1) {
        draw_line_v2(q,p);
        return;
    }

    double d = (x0 - x1) + 0.5 * (y1 - y0);

    Color c0, c1;
    c0.r = colors[p.colorId].r;
    c0.g = colors[p.colorId].g;
    c0.b = colors[p.colorId].b;

    c1.r = colors[q.colorId].r;
    c1.g = colors[q.colorId].g;
    c1.b = colors[q.colorId].b;

    Color c = c0;

    // Directly compute color increment
    Color dc;
    dc.r = (c1.r - c0.r) / (y1 - y0);
    dc.g = (c1.g - c0.g) / (y1 - y0);
    dc.b = (c1.b - c0.b) / (y1 - y0);

    for (int y = y0; y < y1; y++) {

        draw_pixel(x, y, {
                std::round(c.r),
                std::round(c.g),
                std::round(c.b)
                });

        if (d < 0) {
            x = x + 1;
            d += (x0 - x1) + (y1 - y0);
        } else {
            d += (x0 - x1);
        }
        // Update color
        c.r += dc.r;
        c.g += dc.g;
        c.b += dc.b;
    }
} 

/**
 * @brief Draw a line with [-1, 0) slope.
 *
 * @param p A Vec3
 * @param q Another Vec3
 */
void draw_line_v3(Vec3 p, Vec3 q) {

    int x1 = static_cast<int>(q.x);
    int y1 = static_cast<int>(q.y);
    int x0 = static_cast<int>(p.x);
    int y0 = static_cast<int>(p.y);
    int y = y0;

    if (x0 > x1) {
        draw_line_v3(q,p);
        return;
    }

    double d = (y1 - y0) + 0.5 * (x1 - x0);

    Color c0, c1;
    c0.r = colors[p.colorId].r;
    c0.g = colors[p.colorId].g;
    c0.b = colors[p.colorId].b;

    c1.r = colors[q.colorId].r;
    c1.g = colors[q.colorId].g;
    c1.b = colors[q.colorId].b;

    Color c = c0;

    // Directly compute color increment
    Color dc;
    dc.r = (c1.r - c0.r) / (x1 - x0);
    dc.g = (c1.g - c0.g) / (x1 - x0);
    dc.b = (c1.b - c0.b) / (x1 - x0);

    for (int x = x0; x < x1; x++) {

        draw_pixel(x, y, {
                std::round(c.r),
                std::round(c.g),
                std::round(c.b)
                });

        if (d < 0) {
            y = y - 1;
            d += (y1 - y0) + (x1 - x0);
        } else {
            d += (y1 - y0);
        }
        // Update color
        c.r += dc.r;
        c.g += dc.g;
        c.b += dc.b;
    }
} 

/**
 * @brief Draw a line with (-inf, -1) slope.
 *
 * @param p A Vec3
 * @param q Another Vec3
 */
void draw_line_v4(Vec3 p, Vec3 q) {

    int x1 = static_cast<int>(q.x);
    int y1 = static_cast<int>(q.y);
    int x0 = static_cast<int>(p.x);
    int y0 = static_cast<int>(p.y);
    int x = x0;

    if (x0 > x1) {
        draw_line_v4(q,p);
        return;
    }

    double d = (x0 - x1) + 0.5 * (y0 - y1);

    Color c0, c1;
    c0.r = colors[p.colorId].r;
    c0.g = colors[p.colorId].g;
    c0.b = colors[p.colorId].b;

    c1.r = colors[q.colorId].r;
    c1.g = colors[q.colorId].g;
    c1.b = colors[q.colorId].b;

    Color c = c0;

    // Directly compute color increment
    Color dc;
    dc.r = -(c1.r - c0.r) / (y1 - y0);
    dc.g = -(c1.g - c0.g) / (y1 - y0);
    dc.b = -(c1.b - c0.b) / (y1 - y0);

    for (int y = y0; y > y1; y--) {

        draw_pixel(x, y, {
                std::round(c.r),
                std::round(c.g),
                std::round(c.b)
                });

        if (d < 0) {
            x = x + 1;
            d += (x0 - x1) + (y0 - y1);
        } else {
            d += (x0 - x1);
        }
        // Update color
        c.r += dc.r;
        c.g += dc.g;
        c.b += dc.b;
    }
}

/**
 * @brief Draw a line with -inf or +inf  slope.
 *
 * @param p A Vec3
 * @param q Another Vec3
 */
void draw_line_v5(Vec3 p, Vec3 q) {

    int y1 = static_cast<int>(q.y);
    int y0 = static_cast<int>(p.y);
    int x = static_cast<int>(p.x);

    if (y0 > y1) {
        draw_line_v5(q, p);
        return;
    }

    Color c0, c1;
    c0.r = colors[p.colorId].r;
    c0.g = colors[p.colorId].g;
    c0.b = colors[p.colorId].b;

    c1.r = colors[q.colorId].r;
    c1.g = colors[q.colorId].g;
    c1.b = colors[q.colorId].b;

    Color c = c0;

    // Directly compute color increment
    // Change x1-x0 to y1-y0
    
    Color dc;
    dc.r = (c1.r - c0.r) / (y1 - y0);
    dc.g = (c1.g - c0.g) / (y1 - y0);
    dc.b = (c1.b - c0.b) / (y1 - y0);


    for (int y = y0; y != y1; y++) {

        draw_pixel(x, y, {
                std::round(c.r),
                std::round(c.g),
                std::round(c.b)
                });

        // Update color
        c.r += dc.r;
        c.g += dc.g;
        c.b += dc.b;
    }
}

/**
 * @brief Draw a line with 0 slope.
 *
 * @param p A Vec3
 * @param q Another Vec3
 */
void draw_line_v6(Vec3 p, Vec3 q) {

    int x1 = static_cast<int>(q.x);
    int x0 = static_cast<int>(p.x);
    int y = static_cast<int>(p.y);

    if (x0 > x1) {
        draw_line_v6(q, p);
        return;
    }

    Color c0, c1;
    c0.r = colors[p.colorId].r;
    c0.g = colors[p.colorId].g;
    c0.b = colors[p.colorId].b;

    c1.r = colors[q.colorId].r;
    c1.g = colors[q.colorId].g;
    c1.b = colors[q.colorId].b;

    Color c = c0;

    // Directly compute color increment
    
    Color dc;
    dc.r = (c1.r - c0.r) / (x1 - x0);
    dc.g = (c1.g - c0.g) / (x1 - x0);
    dc.b = (c1.b - c0.b) / (x1 - x0);


    for (int x = x0; x != x1; x++) {

        draw_pixel(x, y, {
                std::round(c.r),
                std::round(c.g),
                std::round(c.b)
                });

        // Update color
        c.r += dc.r;
        c.g += dc.g;
        c.b += dc.b;
    }
}

/**
 * @brief Draws the line using the appropriate helper
 *  draw function.
 *
 * @param p A Vec3
 * @param q Another Vec3
 */
void draw_line(Vec3 p, Vec3 q) {
    int x1 = static_cast<int>(q.x);
    int y1 = static_cast<int>(q.y);
    int x0 = static_cast<int>(p.x);
    int y0 = static_cast<int>(p.y);

    if (x0 == x1) {
        // Infinite slope
        draw_line_v5(p, q);
    } else if (y0 == y1) {
        // 0 slope
        draw_line_v6(p, q);
    } else {
        double m = static_cast<double>((y1 - y0)) / (x1 - x0);

        if (m < -1) {
            draw_line_v4(p, q);
        } else if (m < 0) {
            draw_line_v3(p, q);
        } else if (m < 1) {
            draw_line_v1(p, q);
        } else {
            draw_line_v2(p, q);
        }
    }
}

/**
 * @brief Calculates f_01
 *
 * @param xp0 first x-subscript
 * @param yp0 first y-subscript
 * @param xp1 second x-subscript
 * @param yp1 second y-subscript
 * @param x  target x-coordinate
 * @param y  target y-coordinate
 *
 * @return f_01(x,y)
 */
double f_01(int xp0, int yp0, int xp1, int yp1, int x, int y) {
    return (yp0 - yp1) * x + (xp1 - xp0) * y 
        + (xp0 * yp1) - (xp1 * yp0);
}

/**
 * @brief Calculates f_12
 *
 * @param xp0 first x-subscript
 * @param yp0 first y-subscript
 * @param xp1 second x-subscript
 * @param yp1 second y-subscript
 * @param x  target x-coordinate
 * @param y  target y-coordinate
 *
 * @return f_12(x,y)
 */
double f_12(int xp1, int yp1, int xp2, int yp2, int x, int y) {
    return (yp1 - yp2) * x + (xp2 - xp1) * y 
        + (xp1 * yp2) - (xp2 * yp1);
}

/**
 * @brief Calculates f_01
 *
 * @param xp0 first x-subscript
 * @param yp0 first y-subscript
 * @param xp1 second x-subscript
 * @param yp1 second y-subscript
 * @param x  target x-coordinate
 * @param y  target y-coordinate
 *
 * @return f_20(x,y)
 */
double f_20(int xp2, int yp2, int xp0, int yp0, int x, int y) {
    return (yp2 - yp0) * x + (xp0 - xp2) * y 
        + (xp2 * yp0) - (xp0 * yp2);
}

/**
 * @brief Draw a triangle with screen space coordinates
 * p0, p1 and p2.
 *
 * @param p0 A Vec3
 * @param p1 Another Vec3
 * @param p2 Another Vec3
 */
void draw_triangle(Vec3 p0, Vec3 p1, Vec3 p2) {

    int x0 = static_cast<int>(p0.x);
    int x1 = static_cast<int>(p1.x);
    int x2 = static_cast<int>(p2.x);

    int y0 = static_cast<int>(p0.y);
    int y1 = static_cast<int>(p1.y);
    int y2 = static_cast<int>(p2.y);

    // Get bounding boxes
    int x_min = std::floor(std::min(std::min(x0, x1), x2));
    int x_max = std::ceil(std::max(std::max(x0, x1), x2));

    int y_min = std::floor(std::min(std::min(y0, y1), y2));
    int y_max = std::ceil(std::max(std::max(y0, y1), y2));

    double alpha, beta, gamma;
    double f_alpha = draw::f_12(x1, y1, x2, y2, x0, y0);
    double f_beta = draw::f_20(x2, y2, x0, y0, x1, y1);
    double f_gamma = draw::f_01(x0, y0, x1, y1, x2, y2);

    Color c = {0,0,0};

    for (int y = y_min; y < y_max; y++) {
        for (int x = x_min; x < x_max; x++) {
            // Calculate alpha
            double f_12_xy = draw::f_12(x1, y1, x2, y2, x, y);
            alpha = f_12_xy / f_alpha;

            // Calculate beta
            double f_20_xy = draw::f_20(x2, y2, x0, y0, x, y);
            beta = f_20_xy / f_beta;

            // Calculate gamma
            double f_01_xy = draw::f_01(x0, y0, x1, y1, x, y);
            gamma = f_01_xy / f_gamma;

            // Check if inside
            if (alpha >= 0 && beta >= 0 && gamma >= 0) {
                Color c0 = colors[p0.colorId];
                Color c1 = colors[p1.colorId];
                Color c2 = colors[p2.colorId];
                // Interpolate the colors
                c.r = static_cast<int>(
                        std::round(
                        alpha * c0.r +
                        beta * c1.r +
                        gamma * c2.r)
                        );
                c.g = static_cast<int>(
                        std::round(
                        alpha * c0.g +
                        beta * c1.g +
                        gamma * c2.g)
                        );
                c.b = static_cast<int>(
                        std::round(
                        alpha * c0.b +
                        beta * c1.b +
                        gamma * c2.b)
                        );
                // Draw it!
                draw::draw_pixel(x, y, c);
            }
        }
    }

}


}; // namespace draw
    
void rasterize(Model model, double viewTransMatrix[4][4], Camera cam) {
    for (int i = 0; i < model.numberOfTriangles; i++) {
        
        Vec3 a = triangleVertices[i][0];
        Vec3 b = triangleVertices[i][1];
        Vec3 c = triangleVertices[i][2];
        
        // Culling.
        if (backfaceCullingSetting) {
                
            Vec3 normal = normalizeVec3(crossProductVec3(subtractVec3(b, a), subtractVec3(c, a)));
            Vec3 viewVector = subtractVec3(a, cam.pos);
                
            // No calculations.
            if (dotProductVec3(normal, viewVector) >= 0) {
                continue;
            }
                
            // Rasterize.
            else {
                
                double point[4], tmpVec[4];
                
                point[0] = a.x;
                point[1] = a.y;
                point[2] = a.z;
                point[3] = 1;
                
                multiplyMatrixWithVec4d(tmpVec, viewTransMatrix, point);
                
                a.x = tmpVec[0] / tmpVec[3];
                a.y = tmpVec[1] / tmpVec[3];
                a.z = tmpVec[2] / tmpVec[3];
                
                point[0] = b.x;
                point[1] = b.y;
                point[2] = b.z;
                point[3] = 1;
                
                multiplyMatrixWithVec4d(tmpVec, viewTransMatrix, point);
                
                b.x = tmpVec[0] / tmpVec[3];
                b.y = tmpVec[1] / tmpVec[3];
                b.z = tmpVec[2] / tmpVec[3];
                
                point[0] = c.x;
                point[1] = c.y;
                point[2] = c.z;
                point[3] = 1;
                
                multiplyMatrixWithVec4d(tmpVec, viewTransMatrix, point);
                
                c.x = tmpVec[0] / tmpVec[3];
                c.y = tmpVec[1] / tmpVec[3];
                c.z = tmpVec[2] / tmpVec[3];
                
                std::cout << a.x << ' ' << b.z << ' ' << c.z << std::endl << std::endl;
                
                // If model type = Solid then do triangle rasterization.
                if (model.type) {
                    draw::draw_triangle(a, b, c);
                }
                    
                // Else model type = wireframe and do line rasterization.
                else {
                    draw::draw_line(a,b);
                    draw::draw_line(b,c);
                    draw::draw_line(c,a);
                }
            }
        }
            
        // backfaceCullingSetting = 0, no culling is done.
        else {
            
            double point[4], tmpVec[4];
            
            point[0] = a.x;
            point[1] = a.y;
            point[2] = a.z;
            point[3] = 1;
            
            multiplyMatrixWithVec4d(tmpVec, viewTransMatrix, point);
            
            a.x = tmpVec[0] / tmpVec[3];
            a.y = tmpVec[1] / tmpVec[3];
            a.z = tmpVec[2] / tmpVec[3];
            
            point[0] = b.x;
            point[1] = b.y;
            point[2] = b.z;
            point[3] = 1;
            
            multiplyMatrixWithVec4d(tmpVec, viewTransMatrix, point);
            
            b.x = tmpVec[0] / tmpVec[3];
            b.y = tmpVec[1] / tmpVec[3];
            b.z = tmpVec[2] / tmpVec[3];
            
            point[0] = c.x;
            point[1] = c.y;
            point[2] = c.z;
            point[3] = 1;
            
            multiplyMatrixWithVec4d(tmpVec, viewTransMatrix, point);
            
            c.x = tmpVec[0] / tmpVec[3];
            c.y = tmpVec[1] / tmpVec[3];
            c.z = tmpVec[2] / tmpVec[3];
            
            // If model type = Solid then do triangle rasterization.
            if (model.type) {
                draw::draw_triangle(a, b, c);
            }
            
            // Else model type = wireframe and do line rasterization.
            else {
                draw::draw_line(a,b);
                draw::draw_line(b,c);
                draw::draw_line(c,a);
            }
        }
    }
}

    
}; // namespace rasterize

/**
 * @brief Namespace for various test suites.
 */
namespace tests {

/**
 * @brief Checks equality depending on a small interval eps.
 *
 * @param val1 First value
 * @param val2 Second value 
 * @param eps  Equality threshold
 */
bool near_equal(double val1, double val2, double eps) {
    return std::abs(val1 - val2) < eps;
}


/**
 * @brief Runs a small set of modeling
 *   transform tests which fail
 *   if they don't satisfy the assertions.
 */
void test_modeling_transforms() {
    std::size_t num_tests = 0;

    // Test 1
    double v[4] = {0,1,0, 1}; 
    double r1[4][4];
    makeIdentityMatrix(r1);

    double val = 1 / std::sqrt(3);
    Rotation rot = {
        240,
        val,
        val,
        val
    };

    model_tr::getRotationMatrix(r1, rot);

    double result[4];
    multiplyMatrixWithVec4d(result, r1, v);
    double eps = 1e-4;

    assert(
        near_equal(result[0], 1, eps) &&
        near_equal(result[1], 0, eps) &&
        near_equal(result[2], 0, eps)
    );
    num_tests++;

    // Test 2
    v[0] = 2;
    v[1] = 0;
    v[2] = 0;

    rot.angle = 90;
    rot.ux = rot.uz = 0;
    rot.uy = 1;

    makeIdentityMatrix(r1);
    model_tr::getRotationMatrix(r1, rot);
    multiplyMatrixWithVec4d(result, r1, v);

    assert(
        near_equal(result[0], 0, eps) &&
        near_equal(result[1], 0, eps) &&
        near_equal(result[2], -2, eps)
    );
    num_tests++;

    // Test 3: Translation
    
    v[0] = 1;
    v[1] = -2;
    v[2] = 3;

    Translation t = {
        2,
        0,
        -10
    };

    makeIdentityMatrix(r1);
    model_tr::getTranslationMatrix(r1, t);
    multiplyMatrixWithVec4d(result, r1, v);

    assert(
        near_equal(result[0], 3, eps) &&
        near_equal(result[1], -2, eps) &&
        near_equal(result[2], -7, eps)
    );
    num_tests++;

    Scaling s = {
        0.5, -5, 0
    };

    makeIdentityMatrix(r1);
    model_tr::getScalingMatrix(r1, s);
    multiplyMatrixWithVec4d(result, r1, result);

    assert(
        near_equal(result[0], 1.5, eps) &&
        near_equal(result[1], 10, eps) &&
        near_equal(result[2], 0, eps)
    );
    num_tests++;

    std::cout << "[Tests] passed " << num_tests << " tests." << std::endl;
}

void test_drawing_functions() {
    colors[0] = {244,89,66};
    colors[1] = {66,122,244};
    colors[2] = {66,89,244}; // dark blue

    colors[3] = {0,0,0};
    colors[4] = {122,66,244};

    Vec3 p1 = {10,10,10,0};

    Vec3 p2 = {200,600,500,1};

    Vec3 p3 = {100,200, 300, 2};

    Vec3 p4 = {400, 200, 200, 1};
    Vec3 p5 = {300, 500, 300, 1};

    // Fill in the first triangle
    rasterize::draw::draw_triangle(p1, p3, p2);

    // Draw first triangle
    rasterize::draw::draw_line(p1, p2);
    rasterize::draw::draw_line(p2, p3);
    rasterize::draw::draw_line(p3, p1);

    // Fill in the second triangle
    rasterize::draw::draw_triangle(p1, p4, p5);

    // Draw second triangle
    rasterize::draw::draw_line(p1, p4);
    rasterize::draw::draw_line(p4, p5);
    rasterize::draw::draw_line(p5, p1);



    // Draw a star
    Vec3 s1 = {200,300,100, 1};
    Vec3 s2 = {600,300,100, 2};
    Vec3 s3 = {400,500,100, 3};
    Vec3 s4 = {300,150,100, 4};
    Vec3 s5 = {500,150,100, 5};

    rasterize::draw::draw_line(s1,s2);
    rasterize::draw::draw_line(s1,s5);
    rasterize::draw::draw_line(s2,s4);
    rasterize::draw::draw_line(s3,s4);
    rasterize::draw::draw_line(s5,s3);


    // Draw a square

    Vec3 sq1 = {500, 500, 400, 1};
    Vec3 sq2 = {500, 600, 400, 2};
    Vec3 sq3 = {600, 500, 400, 3};
    Vec3 sq4 = {600, 600, 400, 4};

    rasterize::draw::draw_line(sq1,sq2);
    rasterize::draw::draw_line(sq1,sq3);
    rasterize::draw::draw_line(sq3,sq4);
    rasterize::draw::draw_line(sq2,sq4);

}

}; // namespace tests


/*
	Initializes image with background color
*/
void viewportTransformation(double viewportMatrix[4][4], Camera cam);
void projectionTransformation(double projectionMatrix[4][4], Camera cam);
void cameraTransformation(double cameraMatrix[4][4], Camera cam);

void initializeImage(Camera cam) {
    int i, j;

    for (i = 0; i < cam.sizeX; i++)
        for (j = 0; j < cam.sizeY; j++) {
            image[i][j].r = backgroundColor.r;
            image[i][j].g = backgroundColor.g;
            image[i][j].b = backgroundColor.b;

        }
}

/*
	Transformations, culling, rasterization are done here.
	You can define helper functions inside this file (rasterizer.cpp) only.
	Using types in "hw2_types.h" and functions in "hw2_math_ops.cpp" will speed you up while working.
*/
void modelingTransformation(double modelTransMatrix[4][4], Model model) {
    // transformMatrix is the result of all the transform operations.
    // operationMatrix is a tmp matrix to hold new transform operation.
    double transformMatrix[4][4], operationMatrix[4][4], tmpMatrix[4][4];
    makeIdentityMatrix(transformMatrix);
    makeIdentityMatrix(operationMatrix);
    
    // Form the transform matrix which is going to be applied to all vertices.
    for (int i = 0; i < model.numberOfTransformations; i++) {
        switch (model.transformationTypes[i]) {
            case 'r':
                model_tr::getRotationMatrix(operationMatrix, rotations[model.transformationIDs[i]]);
                break;
            case 't':
                model_tr::getTranslationMatrix(operationMatrix, translations[model.transformationIDs[i]]);
                break;
            case 's':
                model_tr::getScalingMatrix(operationMatrix, scalings[model.transformationIDs[i]]);
                break;
        }
        
        multiplyMatrixWithMatrix(tmpMatrix, operationMatrix, transformMatrix);
        equalizeMatrices(transformMatrix, tmpMatrix);
        
        // Prints out the current operationMatrix and last version of transformMatrix.
//
//        std::cout << "Operation Matrix" << std::endl;
//        std::cout << operationMatrix[1][1] << ' ';
//        std::cout << operationMatrix[0][3] << ' ';
//        std::cout << operationMatrix[1][3] << ' ';
//        std::cout << operationMatrix[2][3] << std::endl;
//        std::cout << "Transform Matrix" << std::endl;
//        std::cout << transformMatrix[1][1] << ' ';
//        std::cout << transformMatrix[0][3] << ' ';
//        std::cout << transformMatrix[1][3] << ' ';
//        std::cout << transformMatrix[2][3] << std::endl;

    }
    
    equalizeMatrices(modelTransMatrix, transformMatrix);
}

void viewTransformation(double viewTransMatrix[4][4], Camera cam) {
    double viewportMatrix[4][4], projectionMatrix[4][4], cameraMatrix[4][4];
    double tmpMatrix[4][4];
    
    viewportTransformation(viewportMatrix, cam);
    projectionTransformation(projectionMatrix, cam);
    cameraTransformation(cameraMatrix, cam);
    
    multiplyMatrixWithMatrix(tmpMatrix, projectionMatrix, cameraMatrix);
    multiplyMatrixWithMatrix(viewTransMatrix, viewportMatrix, tmpMatrix);
}

void viewportTransformation(double viewportMatrix[4][4], Camera cam) {
    makeIdentityMatrix(viewportMatrix);
    viewportMatrix[0][0] = cam.sizeX / 2.0;
    viewportMatrix[1][1] = cam.sizeY / 2.0;
    viewportMatrix[0][3] = (cam.sizeX - 1) / 2.0;
    viewportMatrix[1][3] = (cam.sizeY - 1) / 2.0;
}

void projectionTransformation(double projectionMatrix[4][4], Camera cam) {
    makeIdentityMatrix(projectionMatrix);
    projectionMatrix[3][3] = 0;
    projectionMatrix[0][0] = (2 * cam.n) / (cam.r - cam.l);
    projectionMatrix[0][2] = - (cam.l + cam.r) / (cam.l - cam.r);
    projectionMatrix[1][1] = (2 * cam.n) / (cam.t - cam.b);
    projectionMatrix[1][2] = - (cam.b + cam.t) / (cam.b - cam.t);
    projectionMatrix[2][2] = (cam.f + cam.n) / (cam.n - cam.f);
    projectionMatrix[2][3] = - (2 * cam.f * cam.n) / (cam.f - cam.n);
    projectionMatrix[3][2] = -1;
}

void cameraTransformation(double cameraMatrix[4][4], Camera cam) {
    double tmpRot[4][4], tmpTrans[4][4];
    makeIdentityMatrix(tmpRot);
    makeIdentityMatrix(tmpTrans);
    
    tmpRot[0][0] = cam.u.x;
    tmpRot[1][0] = cam.v.x;
    tmpRot[2][0] = cam.w.x;
    tmpRot[0][1] = cam.u.y;
    tmpRot[1][1] = cam.v.y;
    tmpRot[2][1] = cam.w.y;
    tmpRot[0][2] = cam.u.z;
    tmpRot[1][2] = cam.v.z;
    tmpRot[2][2] = cam.w.z;
    
    tmpTrans[0][3] = -cam.pos.x;
    tmpTrans[1][3] = -cam.pos.y;
    tmpTrans[2][3] = -cam.pos.z;
    
    multiplyMatrixWithMatrix(cameraMatrix, tmpRot, tmpTrans);
}

void applyTransformations(Model model, double modelTransMatrix[4][4]) {
    
    // Copy current vertices coordiates of the model.
    for (int i = 0; i < model.numberOfTriangles; i++) {
        triangleVertices[i][0] = vertices[model.triangles[i].vertexIds[0]];
        triangleVertices[i][1] = vertices[model.triangles[i].vertexIds[1]];
        triangleVertices[i][2] = vertices[model.triangles[i].vertexIds[2]];
    }
    
    double point[4], tmpVec[4];
    for (int i = 0; i < model.numberOfTriangles; i++) {
        for (int j = 0; j < 3; j++) {
            point[0] = triangleVertices[i][j].x;
            point[1] = triangleVertices[i][j].y;
            point[2] = triangleVertices[i][j].z;
            point[3] = 1;
            
            multiplyMatrixWithVec4d(tmpVec, modelTransMatrix, point);
            
            triangleVertices[i][j].x = tmpVec[0];
            triangleVertices[i][j].y = tmpVec[1];
            triangleVertices[i][j].z = tmpVec[2];
        }
    }
}

void printModelVertices(Model model) {
    // Print out transformed vertecies of each triangle of the model.
    for (int i = 0; i < model.numberOfTriangles; i++) {
        std::cout << "Triangle: " << i << std::endl;
        for (int j = 0; j < 3; j++) {
            std::cout << "Vertex " << j << ": ";
            std::cout << triangleVertices[i][j].x << ' ';
            std::cout << triangleVertices[i][j].y << ' ';
            std::cout << triangleVertices[i][j].z << std::endl;
        }
    }
}

void forwardRenderingPipeline(Camera cam) {
    // TODO: IMPLEMENT HERE
    //tests::test_modeling_transforms();
    
    
    double viewTransMatrix[4][4];
    double modelTransMatrix[4][4];
    
    viewTransformation(viewTransMatrix, cam);
    
    for (int i = 0; i < numberOfModels; i++) {
        modelingTransformation(modelTransMatrix, models[i]);
        applyTransformations(models[i], modelTransMatrix);
        rasterize::rasterize(models[i], viewTransMatrix, cam);
    }
}


int main(int argc, char **argv) {
    int i, j;

    if (argc < 2) {
        std::cout << "Usage: ./rasterizer <scene file> <camera file>" << std::endl;
        return 1;
    }

    // read camera and scene files
    readSceneFile(argv[1]);
    readCameraFile(argv[2]);

    image = 0;

    // TODO:
    // remove this later:
//    numberOfCameras = 1;
    for (i = 0; i < numberOfCameras; i++) {

        // allocate memory for image
        if (image) {
			for (j = 0; j < cameras[i].sizeX; j++) {
		        delete image[j];
		    }

			delete[] image;
		}

        image = new Color*[cameras[i].sizeX];

        if (image == NULL) {
            std::cout << "ERROR: Cannot allocate memory for image." << std::endl;
            exit(1);
        }

        for (j = 0; j < cameras[i].sizeX; j++) {
            image[j] = new Color[cameras[i].sizeY];
            if (image[j] == NULL) {
                std::cout << "ERROR: Cannot allocate memory for image." << std::endl;
                exit(1);
            }
        }

        // initialize image with basic values
        initializeImage(cameras[i]);


        // Test the drawing functions
//        tests::test_drawing_functions();

        // do forward rendering pipeline operations
        forwardRenderingPipeline(cameras[i]);

        // generate PPM file
        writeImageToPPMFile(cameras[i]);

        // Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
        // Notice that os_type is not given as 1 (Ubuntu) or 2 (Windows), below call doesn't do conversion.
        // Change os_type to 1 or 2, after being sure that you have ImageMagick installed.
        convertPPMToPNG(cameras[i].outputFileName, 99);
    }

    return 0;

}
