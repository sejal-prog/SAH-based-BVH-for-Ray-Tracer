#include <iostream>
#include <vector>
#include <cmath>
#include "/home/sejal/Documents/Computer Graphics Lab /Simple_ray_tracer/CImg-3.5.4/CImg.h"
using namespace cimg_library;



struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float f) const { return Vec3(x * f, y * f, z * f); }

    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

    Vec3 cross(const Vec3& v) const {
        return Vec3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    Vec3 normalize() const {
        float len = std::sqrt(x * x + y * y + z * z);
        return Vec3(x / len, y / len, z / len);
    }
};

struct Color {
    int r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(int rr, int gg, int bb) : r(rr), g(gg), b(bb) {}
};

bool hitTriangle(const Vec3& rayOrigin, const Vec3& rayDir, const Vec3& A, const Vec3& B, const Vec3& C) {

    Vec3 N = (B - A).cross(C - A).normalize();
    float denom = N.dot(rayDir);
    if (std::abs(denom) < 1e-6) return false;  

    float t = N.dot(A - rayOrigin) / denom;
    if (t < 0) return false;

    Vec3 P = rayOrigin + rayDir * t;

    Vec3 v0 = B - A, v1 = C - A, v2 = P - A;
    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom_bary = d00 * d11 - d01 * d01;

    float v = (d11 * d20 - d01 * d21) / denom_bary;
    float w = (d00 * d21 - d01 * d20) / denom_bary;
    float u = 1.0f - v - w;

    return (u >= 0 && v >= 0 && w >= 0 && u <= 1 && v <= 1 && w <= 1);
}

Color traceRay(const Vec3& origin, const Vec3& dir) {
   
    Vec3 A(-0.3, -0.2, -1);
    Vec3 B(0.3, -0.2, -1);
    Vec3 C(0.0, 0.3, -1);

    if (hitTriangle(origin, dir, A, B, C)) {
        return Color(255, 255, 255); 
    }
    return Color(0, 0, 0); 
}

int main() {
    const int width = 400;
    const int height = 400;
    
    CImg<unsigned char> image(width, height, 1, 3, 0);
    
    Vec3 camera(0, 0, 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float u = float(x) / (width - 1);
            float v = float(height - y - 1) / (height - 1);
            Vec3 dir(u * 2 - 1, v * 2 - 1, -1);  
            Color pixel = traceRay(camera, dir.normalize());
            
            image(x, y, 0, 0) = pixel.r;  
            image(x, y, 0, 1) = pixel.g;  
            image(x, y, 0, 2) = pixel.b; 
        }
    }

    image.save("output.png");

    std::cout << "Image written to output.png\n";
    return 0;
}

// As we are using CImg library, make sure to link against the CImg library when compiling.
// For example, if you are using g++, you can compile with:
// g++ ray_tracer.cpp -o raytracer -O2 -std=c++11 -lX11
// and Then run the program with:
// ./raytracer