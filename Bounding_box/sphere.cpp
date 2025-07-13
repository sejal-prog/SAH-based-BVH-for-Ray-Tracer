

#define cimg_display 0  // Disable X11 display
#define TINYOBJLOADER_IMPLEMENTATION  // Include tinyobj implementation

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include "tiny_obj_loader.h"
#include "/home/sejal/Documents/Computer Graphics Lab /Simple_ray_tracer/CImg-3.5.4/CImg.h"  // Ensure CImg.h is in the current directory or adjust the path
using namespace cimg_library;

using namespace std;

// Vector3 helper
struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float a, float b, float c) : x(a), y(b), z(c) {}

    Vec3 operator+(const Vec3& b) const { return Vec3(x + b.x, y + b.y, z + b.z); }
    Vec3 operator-(const Vec3& b) const { return Vec3(x - b.x, y - b.y, z - b.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator*(const Vec3& b) const { return Vec3(x * b.x, y * b.y, z * b.z); }

    float dot(const Vec3& b) const { return x * b.x + y * b.y + z * b.z; }
    Vec3 cross(const Vec3& b) const {
        return Vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalize() const {
        float len = length();
        return (len > 0) ? Vec3(x / len, y / len, z / len) : Vec3();
    }
    Vec3 reflect(const Vec3& N) const {
        return *this - N * (2 * this->dot(N));
    }
};

// Axis-Aligned Bounding Box (AABB) structure
struct AABB {
    Vec3 min;
    Vec3 max;
    
    AABB() : min(INFINITY, INFINITY, INFINITY), max(-INFINITY, -INFINITY, -INFINITY) {}
    
    AABB(const Vec3& v1, const Vec3& v2) {
        min = Vec3(std::min(v1.x, v2.x), std::min(v1.y, v2.y), std::min(v1.z, v2.z));
        max = Vec3(std::max(v1.x, v2.x), std::max(v1.y, v2.y), std::max(v1.z, v2.z));
    }
    
    // Expand the bounding box to include a point
    void expand(const Vec3& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }
    
    // Expand to include another AABB
    void expand(const AABB& other) {
        min.x = std::min(min.x, other.min.x);
        min.y = std::min(min.y, other.min.y);
        min.z = std::min(min.z, other.min.z);
        max.x = std::max(max.x, other.max.x);
        max.y = std::max(max.y, other.max.y);
        max.z = std::max(max.z, other.max.z);
    }
};

// Ray-AABB intersection test (slab method)
bool rayAABBIntersect(const Vec3& rayOrigin, const Vec3& rayDir, const AABB& box, 
                      float& tMin, float& tMax) {
    tMin = -INFINITY;
    tMax = INFINITY;
    
    for (int i = 0; i < 3; i++) {
        float origin = (i == 0) ? rayOrigin.x : ((i == 1) ? rayOrigin.y : rayOrigin.z);
        float direction = (i == 0) ? rayDir.x : ((i == 1) ? rayDir.y : rayDir.z);
        float boxMin = (i == 0) ? box.min.x : ((i == 1) ? box.min.y : box.min.z);
        float boxMax = (i == 0) ? box.max.x : ((i == 1) ? box.max.y : box.max.z);
        
        if (std::abs(direction) < 1e-6) {
            // Ray is parallel to slab, check if ray origin is within slab
            if (origin < boxMin || origin > boxMax) {
                return false;
            }
        } else {
            // Calculate intersection t values of the ray with the near and far planes
            float invDir = 1.0f / direction;
            float t1 = (boxMin - origin) * invDir;
            float t2 = (boxMax - origin) * invDir;
            
            // Ensure t1 <= t2
            if (t1 > t2) {
                std::swap(t1, t2);
            }
            
            // Update tMin and tMax
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            
            if (tMin > tMax) {
                return false;
            }
        }
    }
    
    return true;
}

// Triangle structure with bounding box
struct Triangle {
    Vec3 vertices[3];
    Vec3 normals[3];
    AABB bbox;  // Bounding box for the triangle
    
    Triangle() {
        // Default constructor
    }
    
    // Initialize bounding box
    void computeBBox() {
        bbox = AABB();
        bbox.expand(vertices[0]);
        bbox.expand(vertices[1]);
        bbox.expand(vertices[2]);
    }
};

// Triangle mesh with hierarchical bounding box
struct TriangleMesh {
    std::vector<Triangle> triangles;
    AABB bbox;  // Bounding box for the entire mesh
    
    void computeBBox() {
        bbox = AABB();
        for (Triangle& tri : triangles) {
            tri.computeBBox();
            bbox.expand(tri.bbox);
        }
    }
};

// Sphere structure
struct Sphere {
    Vec3 center;
    float radius;
};

// Phong shading parameters
struct Light {
    Vec3 position = Vec3(0, 0, 5);  // CORRECTED: Light at center-front
    Vec3 intensity = Vec3(1, 1, 1);
};

struct Material {
    Vec3 ambient = Vec3(0.1, 0.1, 0.1);
    Vec3 diffuse = Vec3(0.7, 0.2, 0.2);  // Red diffuse color
    Vec3 specular = Vec3(1.0, 1.0, 1.0); // White specular highlights
    float shininess = 32.0f;
};

// Generate UV sphere mesh - CORRECTED for smooth normals
std::vector<Triangle> generateUVSphere(float radius, int stacks, int slices) {
    std::vector<Triangle> triangles;
    
    // Generate vertices with proper sphere normals
    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    
    // Add vertices for the sphere
    for (int i = 0; i <= stacks; i++) {
        float phi = M_PI * i / stacks;  // From 0 to PI
        float y = radius * cos(phi);
        float r = radius * sin(phi);
        
        for (int j = 0; j <= slices; j++) {
            float theta = 2.0f * M_PI * j / slices;  // From 0 to 2*PI
            float x = r * cos(theta);
            float z = r * sin(theta);
            
            Vec3 vertex(x, y, z);
            vertices.push_back(vertex);
            normals.push_back(vertex.normalize());  // For sphere, normal = normalized position
        }
    }
    
    // Create triangles
    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < slices; j++) {
            int current = i * (slices + 1) + j;
            int next = current + slices + 1;
            
            // First triangle
            Triangle t1;
            t1.vertices[0] = vertices[current];
            t1.vertices[1] = vertices[next];
            t1.vertices[2] = vertices[current + 1];
            
            t1.normals[0] = normals[current];
            t1.normals[1] = normals[next];
            t1.normals[2] = normals[current + 1];
            
            triangles.push_back(t1);
            
            // Second triangle
            Triangle t2;
            t2.vertices[0] = vertices[current + 1];
            t2.vertices[1] = vertices[next];
            t2.vertices[2] = vertices[next + 1];
            
            t2.normals[0] = normals[current + 1];
            t2.normals[1] = normals[next];
            t2.normals[2] = normals[next + 1];
            
            triangles.push_back(t2);
        }
    }
    
    return triangles;
}

// Phong shading model - CORRECTED reflection calculation
Vec3 phong_shading(const Vec3& normal, const Vec3& position, const Vec3& view_pos,
                   const Light& light, const Material& material,
                   bool use_diffuse = true, bool use_specular = true) {
    Vec3 N = normal.normalize();
    Vec3 L = (light.position - position).normalize();
    Vec3 V = (view_pos - position).normalize();
    Vec3 R = (L * -1).reflect(N);  // CORRECTED: Reflect incident ray direction

    Vec3 ambient = material.ambient;
    Vec3 diffuse = Vec3(0, 0, 0);
    Vec3 specular = Vec3(0, 0, 0);
    
    // Only calculate lighting if surface faces the light
    float NdotL = N.dot(L);
    if (NdotL > 0) {
        if (use_diffuse) {
            diffuse = material.diffuse * light.intensity * NdotL;
        }

        if (use_specular) {
            float RdotV = std::max(R.dot(V), 0.0f);
            float spec = std::pow(RdotV, material.shininess);
            specular = material.specular * light.intensity * spec;
        }
    }

    return ambient + diffuse + specular;
}

// Ray-triangle intersection using barycentric coordinates
bool rayTriangleIntersect(const Vec3& rayOrigin, const Vec3& rayDir, const Triangle& triangle, 
    float& t, float& u, float& v) {
const Vec3& A = triangle.vertices[0];
const Vec3& B = triangle.vertices[1];
const Vec3& C = triangle.vertices[2];

// Step 1: Compute triangle normal
Vec3 AB = B - A;
Vec3 AC = C - A;
Vec3 normal = AB.cross(AC);

// Step 2: Check if ray is parallel to triangle
float NdotRay = normal.dot(rayDir);
if (std::abs(NdotRay) < 1e-6f) {
return false; // Ray is parallel to triangle
}

// Step 3: Find intersection with triangle plane
float d = normal.dot(A);
t = (d - normal.dot(rayOrigin)) / NdotRay;

if (t < 1e-6f) {
return false; // Intersection is behind ray origin
}

// Step 4: Compute intersection point
Vec3 P = rayOrigin + rayDir * t;

// Step 5: Compute barycentric coordinates
Vec3 AP = P - A;

// Project onto triangle plane to get 2D coordinates
Vec3 n = normal.normalize();

// Find two orthogonal vectors in the triangle plane
Vec3 u_axis = AB.normalize();
Vec3 v_axis = n.cross(u_axis);

// Project vectors onto 2D coordinate system
float AP_u = AP.dot(u_axis);
float AP_v = AP.dot(v_axis);
float AB_u = AB.dot(u_axis);
float AB_v = AB.dot(v_axis);
float AC_u = AC.dot(u_axis);
float AC_v = AC.dot(v_axis);

// Solve 2D linear system: AP = u*AB + v*AC
float det = AB_u * AC_v - AB_v * AC_u;
if (std::abs(det) < 1e-6f) {
return false; // Degenerate triangle
}

u = (AP_u * AC_v - AP_v * AC_u) / det;
v = (AB_u * AP_v - AB_v * AP_u) / det;
float w = 1.0f - u - v;

// Step 6: Check if point is inside triangle
return (u >= 0.0f && v >= 0.0f && w >= 0.0f);
}
// Ray-sphere intersection
bool raySphereIntersect(const Vec3& rayOrigin, const Vec3& rayDir, const Sphere& sphere,
                        float& t1, float& t2) {
    Vec3 oc = rayOrigin - sphere.center;
    float a = rayDir.dot(rayDir);
    float b = 2.0f * oc.dot(rayDir);
    float c = oc.dot(oc) - sphere.radius * sphere.radius;
    float discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return false;
    } else {
        float sqrtd = sqrt(discriminant);
        t1 = (-b - sqrtd) / (2.0f * a);
        t2 = (-b + sqrtd) / (2.0f * a);
        return true;
    }
}

// Direct sphere rendering for comparison with mesh
bool renderAnalyticSphere(const Vec3& rayOrigin, const Vec3& rayDir, const Sphere& sphere,
                         float& closest_t, Vec3& hit_normal) {
    float t1, t2;
    if (!raySphereIntersect(rayOrigin, rayDir, sphere, t1, t2)) {
        return false;
    }
    
    closest_t = (t1 > 0) ? t1 : t2;
    if (closest_t <= 0) return false;
    
    Vec3 hit_point = rayOrigin + rayDir * closest_t;
    hit_normal = (hit_point - sphere.center).normalize();
    return true;
}

int main(int argc, char** argv) {
    // Parse command line arguments for customization
    float diffuse_r = 0.7f, diffuse_g = 0.2f, diffuse_b = 0.2f;
    float specular_r = 1.0f, specular_g = 1.0f, specular_b = 1.0f;
    float shininess = 32.0f;
    bool use_diffuse = true, use_specular = true;
    bool use_analytic = false;  
    bool use_obj = false;        // CORRECTED: Default to generated sphere for smoothness
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--diffuse" && i + 3 < argc) {
            diffuse_r = std::stof(argv[i+1]);
            diffuse_g = std::stof(argv[i+2]);
            diffuse_b = std::stof(argv[i+3]);
            i += 3;
        } else if (arg == "--specular" && i + 3 < argc) {
            specular_r = std::stof(argv[i+1]);
            specular_g = std::stof(argv[i+2]);
            specular_b = std::stof(argv[i+3]);
            i += 3;
        } else if (arg == "--shininess" && i + 1 < argc) {
            shininess = std::stof(argv[i+1]);
            i += 1;
        } else if (arg == "--no-diffuse") {
            use_diffuse = false;
        } else if (arg == "--no-specular") {
            use_specular = false;
        } else if (arg == "--analytic") {
            use_analytic = true;
        } else if (arg == "--obj") {  // CORRECTED: Use --obj flag to load OBJ
            use_obj = true;
        }
    }

    std::vector<Triangle> triangles;
    std::string inputfile = "sphere.obj";  // Remove the subdirectory path  // Input OBJ file
    
    if (use_obj) {
        // Load OBJ file using tinyobjloader
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputfile.c_str());

        if (!warn.empty()) std::cout << "WARN: " << warn << std::endl;
        if (!err.empty()) std::cerr << "ERR: " << err << std::endl;
        if (!ret) {
            std::cerr << "Failed to load OBJ. Will generate a sphere instead." << std::endl;
            use_obj = false;
        } else {
            std::cout << "OBJ file loaded successfully!" << std::endl;
            std::cout << "Number of shapes: " << shapes.size() << std::endl;
            
            // Process the loaded mesh into triangles
            for (const auto& shape : shapes) {
                size_t index_offset = 0;
                for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
                    int fv = shape.mesh.num_face_vertices[f];
                    
                    if (fv == 3) {  // We only process triangles
                        Triangle triangle;
                        
                        // Loop over vertices in the face
                        for (int v = 0; v < fv; v++) {
                            tinyobj::index_t idx = shape.mesh.indices[index_offset + v];
                            
                            // Access vertex position
                            triangle.vertices[v] = Vec3(
                                attrib.vertices[3 * idx.vertex_index + 0],
                                attrib.vertices[3 * idx.vertex_index + 1],
                                attrib.vertices[3 * idx.vertex_index + 2]
                            );
                            
                            // CORRECTED: Always generate smooth normals for sphere
                            Vec3 sphereNormal = triangle.vertices[v].normalize();
                            triangle.normals[v] = sphereNormal;
                        }
                        
                        triangles.push_back(triangle);
                    }
                    
                    index_offset += fv;
                }
            }
        }
    }
    
    // Generate smooth sphere mesh if not using OBJ
    if (!use_obj || triangles.empty()) {
        std::cout << "Generating UV sphere mesh..." << std::endl;
        triangles = generateUVSphere(1.0f, 32, 32); // CORRECTED: More reasonable resolution
        std::cout << "Generated " << triangles.size() << " triangles" << std::endl;
    }

    // Set up the mesh with bounding boxes
    TriangleMesh mesh;
    mesh.triangles = triangles;
    mesh.computeBBox();

    // Set up image dimensions for ray tracing
    int width = 800, height = 800;
    CImg<unsigned char> image(width, height, 1, 3, 0);  // Create an image

    Light light;
    Material material;
    material.diffuse = Vec3(diffuse_r, diffuse_g, diffuse_b);
    material.specular = Vec3(specular_r, specular_g, specular_b);
    material.shininess = shininess;
    
    Vec3 camera_pos(0, 0, 3);  // CORRECTED: Camera at center-front
    Vec3 camera_target(0, 0, 0);  // Looking at the origin
    float camera_fov = 60.0f * M_PI / 180.0f;  // 60 degrees field of view
    
    // Define analytic sphere for comparison
    Sphere sphere;
    sphere.center = Vec3(0, 0, 0);  // CORRECTED: Sphere at center
    sphere.radius = 1.0f;
    
    std::cout << "Rendering image..." << std::endl;
    std::cout << "Camera position: (" << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << ")" << std::endl;
    std::cout << "Light position: (" << light.position.x << ", " << light.position.y << ", " << light.position.z << ")" << std::endl;

    // Ray tracing loop for each pixel in the image
    #pragma omp parallel for
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Calculate ray direction properly for perspective projection
            float u = (float(x) + 0.5f) / float(width);
            float v = (float(y) + 0.5f) / float(height);
            
            // Convert to screen space coordinates [-1, 1]
            float ndc_x = 2.0f * u - 1.0f;
            float ndc_y = 1.0f - 2.0f * v;  // Flip Y to match image coordinates
            
            float aspect_ratio = float(width) / float(height);
            
            // Calculate ray direction with proper perspective projection
            Vec3 ray_dir(
                ndc_x * aspect_ratio * tan(camera_fov / 2.0f),
                ndc_y * tan(camera_fov / 2.0f),
                -1.0f
            );
            ray_dir = ray_dir.normalize();
            
            // Ray intersection variables
            float closest_t = INFINITY;
            Vec3 hit_normal;
            Vec3 hit_color(0.05f, 0.05f, 0.1f);  // CORRECTED: Dark blue background
            bool hit_found = false;
            
            if (use_analytic) {
                // Use analytic sphere for perfect results
                hit_found = renderAnalyticSphere(camera_pos, ray_dir, sphere, closest_t, hit_normal);
            } else {
                // Check intersection with all triangles in the mesh using bounding box acceleration
                float meshTMin, meshTMax;
                if (rayAABBIntersect(camera_pos, ray_dir, mesh.bbox, meshTMin, meshTMax)) {
                    // Only test triangles if the mesh's bounding box was hit
                    for (const Triangle& triangle : mesh.triangles) {
                        float triTMin, triTMax;
                        if (!rayAABBIntersect(camera_pos, ray_dir, triangle.bbox, triTMin, triTMax)) {
                            // Ray doesn't hit this triangle's bounding box, skip it
                            continue;
                        }
                        
                        // Only perform the expensive ray-triangle intersection test if the bounding box was hit
                        float t, u, v;
                        if (rayTriangleIntersect(camera_pos, ray_dir, triangle, t, u, v)) {
                            if (t < closest_t) {
                                closest_t = t;
                                hit_found = true;
                                
                                // Barycentric interpolation of normals for smooth shading
                                float w = 1.0f - u - v;
                                hit_normal = (triangle.normals[0] * w + 
                                            triangle.normals[1] * u + 
                                            triangle.normals[2] * v).normalize();
                            }
                        }
                    }
                }
            }
            
            // If we hit something, calculate shading
            if (hit_found) {
                Vec3 hit_point = camera_pos + ray_dir * closest_t;
                Vec3 color = phong_shading(hit_normal, hit_point, camera_pos, light, material, use_diffuse, use_specular);
                
                // Clamp color values and convert to 8-bit
                color.x = std::min(std::max(color.x, 0.0f), 1.0f);
                color.y = std::min(std::max(color.y, 0.0f), 1.0f);
                color.z = std::min(std::max(color.z, 0.0f), 1.0f);
                
                image(x, y, 0) = static_cast<unsigned char>(color.x * 255.0f);
                image(x, y, 1) = static_cast<unsigned char>(color.y * 255.0f);
                image(x, y, 2) = static_cast<unsigned char>(color.z * 255.0f);
            } else {
                // Background color
                image(x, y, 0) = static_cast<unsigned char>(hit_color.x * 255.0f);
                image(x, y, 1) = static_cast<unsigned char>(hit_color.y * 255.0f);
                image(x, y, 2) = static_cast<unsigned char>(hit_color.z * 255.0f);
            }
        }
    }

    // Save the image
    std::string output_filename = "sphere_no_diffuse.png";
    image.save(output_filename.c_str());
    std::cout << "Image saved to " << output_filename << std::endl;
    

    return 0;
}