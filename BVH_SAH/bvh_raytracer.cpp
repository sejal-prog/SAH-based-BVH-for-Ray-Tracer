#define cimg_display 0

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <memory>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
#pragma GCC diagnostic ignored "-Wall"
#include "/home/sejal/Documents/Computer Graphics Lab /Simple_ray_tracer/CImg-3.5.4/CImg.h"
#pragma GCC diagnostic pop

using namespace cimg_library;
using namespace std;

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

struct AABB {
    Vec3 min;
    Vec3 max;
    
    AABB() : min(INFINITY, INFINITY, INFINITY), max(-INFINITY, -INFINITY, -INFINITY) {}
    
    AABB(const Vec3& v1, const Vec3& v2) {
        min = Vec3(std::min(v1.x, v2.x), std::min(v1.y, v2.y), std::min(v1.z, v2.z));
        max = Vec3(std::max(v1.x, v2.x), std::max(v1.y, v2.y), std::max(v1.z, v2.z));
    }
    
    void expand(const Vec3& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }
    
    void expand(const AABB& other) {
        if (other.min.x != INFINITY) {
            expand(other.min);
            expand(other.max);
        }
    }
    
    Vec3 center() const {
        return (min + max) * 0.5f;
    }
    
    float surfaceArea() const {
        if (min.x == INFINITY) return 0.0f;
        Vec3 d = max - min;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
    
    int longestAxis() const {
        Vec3 d = max - min;
        if (d.x > d.y && d.x > d.z) return 0;
        if (d.y > d.z) return 1;
        return 2;
    }
};

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
            if (origin < boxMin || origin > boxMax) {
                return false;
            }
        } else {
            float invDir = 1.0f / direction;
            float t1 = (boxMin - origin) * invDir;
            float t2 = (boxMax - origin) * invDir;
            
            if (t1 > t2) std::swap(t1, t2);
            
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            
            if (tMin > tMax) return false;
        }
    }
    return true;
}

struct Triangle {
    Vec3 vertices[3];
    Vec3 normals[3];
    AABB bbox;
    int id;
    
    Triangle() : id(-1) {}
    
    void computeBBox() {
        bbox = AABB();
        bbox.expand(vertices[0]);
        bbox.expand(vertices[1]);
        bbox.expand(vertices[2]);
    }
    
    Vec3 centroid() const {
        return (vertices[0] + vertices[1] + vertices[2]) * (1.0f / 3.0f);
    }
};

struct BVHNode {
    AABB bbox;
    std::unique_ptr<BVHNode> left;
    std::unique_ptr<BVHNode> right;
    std::vector<int> triangleIndices;
    bool isLeaf;
    int nodeId;
    int depth;
    float splitPos;
    int splitAxis;
    
    BVHNode() : isLeaf(false), nodeId(-1), depth(0), splitPos(0), splitAxis(-1) {}
    
    bool isLeafNode() const {
        return isLeaf && !triangleIndices.empty();
    }
};

struct SAHBucket {
    AABB bbox;
    int count;
    
    SAHBucket() : count(0) {}
};

class BucketedSAHBuilder {
private:
    std::vector<Triangle>* triangles;
    int maxTrianglesPerLeaf;
    int maxDepth;
    int nodeCounter;
    static const int NUM_BUCKETS = 32;
    
public:
    BucketedSAHBuilder(std::vector<Triangle>* tris, int maxTrisPerLeaf = 8, int maxD = 20) 
        : triangles(tris), maxTrianglesPerLeaf(maxTrisPerLeaf), maxDepth(maxD), nodeCounter(0) {}
    
    std::unique_ptr<BVHNode> build(std::vector<int>& triangleIndices, int depth = 0) {
        auto node = std::make_unique<BVHNode>();
        node->nodeId = nodeCounter++;
        node->depth = depth;
        
        for (int idx : triangleIndices) {
            node->bbox.expand((*triangles)[idx].bbox);
        }
        
        if (triangleIndices.size() <= maxTrianglesPerLeaf || depth >= maxDepth) {
            node->isLeaf = true;
            node->triangleIndices = triangleIndices;
            return node;
        }
        
        float bestCost = INFINITY;
        int bestAxis = -1;
        int bestSplitBucket = -1;
        
        for (int axis = 0; axis < 3; axis++) {
            float minBound = (axis == 0) ? node->bbox.min.x : 
                           (axis == 1) ? node->bbox.min.y : node->bbox.min.z;
            float maxBound = (axis == 0) ? node->bbox.max.x : 
                           (axis == 1) ? node->bbox.max.y : node->bbox.max.z;
            
            if (minBound == maxBound) continue;
            
            SAHBucket buckets[NUM_BUCKETS];
            
            for (int idx : triangleIndices) {
                Vec3 centroid = (*triangles)[idx].centroid();
                float centroidAxis = (axis == 0) ? centroid.x : 
                                   (axis == 1) ? centroid.y : centroid.z;
                
                int bucketIdx = (int)(NUM_BUCKETS * (centroidAxis - minBound) / (maxBound - minBound));
                bucketIdx = std::min(bucketIdx, NUM_BUCKETS - 1);
                bucketIdx = std::max(bucketIdx, 0);
                
                buckets[bucketIdx].count++;
                buckets[bucketIdx].bbox.expand((*triangles)[idx].bbox);
            }
            
            for (int i = 0; i < NUM_BUCKETS - 1; i++) {
                AABB leftBox, rightBox;
                int leftCount = 0, rightCount = 0;
                
                for (int j = 0; j <= i; j++) {
                    if (buckets[j].count > 0) {
                        leftBox.expand(buckets[j].bbox);
                        leftCount += buckets[j].count;
                    }
                }
                
                for (int j = i + 1; j < NUM_BUCKETS; j++) {
                    if (buckets[j].count > 0) {
                        rightBox.expand(buckets[j].bbox);
                        rightCount += buckets[j].count;
                    }
                }
                
                if (leftCount == 0 || rightCount == 0) continue;
                
                float leftSA = leftBox.surfaceArea();
                float rightSA = rightBox.surfaceArea();
                float cost = 0.125f + (leftSA * leftCount + rightSA * rightCount) / node->bbox.surfaceArea();
                
                if (cost < bestCost) {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplitBucket = i;
                }
            }
        }
        
        float leafCost = (float)triangleIndices.size();
        if (bestCost >= leafCost || bestAxis == -1) {
            node->isLeaf = true;
            node->triangleIndices = triangleIndices;
            return node;
        }
        
        float minBound = (bestAxis == 0) ? node->bbox.min.x : 
                        (bestAxis == 1) ? node->bbox.min.y : node->bbox.min.z;
        float maxBound = (bestAxis == 0) ? node->bbox.max.x : 
                        (bestAxis == 1) ? node->bbox.max.y : node->bbox.max.z;
        
        float splitPos = minBound + (maxBound - minBound) * (bestSplitBucket + 1) / NUM_BUCKETS;
        
        std::vector<int> leftTriangles, rightTriangles;
        
        for (int idx : triangleIndices) {
            Vec3 centroid = (*triangles)[idx].centroid();
            float centroidAxis = (bestAxis == 0) ? centroid.x : 
                               (bestAxis == 1) ? centroid.y : centroid.z;
            
            if (centroidAxis < splitPos) {
                leftTriangles.push_back(idx);
            } else {
                rightTriangles.push_back(idx);
            }
        }
        
        if (leftTriangles.empty() || rightTriangles.empty()) {
            size_t mid = triangleIndices.size() / 2;
            leftTriangles.assign(triangleIndices.begin(), triangleIndices.begin() + mid);
            rightTriangles.assign(triangleIndices.begin() + mid, triangleIndices.end());
        }
        
        node->splitAxis = bestAxis;
        node->splitPos = splitPos;
        
        node->left = build(leftTriangles, depth + 1);
        node->right = build(rightTriangles, depth + 1);
        
        return node;
    }
    
    int getTotalNodes() const { return nodeCounter; }
};

class TreeVisualizer {
public:
    static void printTree(const std::unique_ptr<BVHNode>& node, const std::string& prefix = "", bool isLast = true) {
        if (!node) return;
        
        std::cout << prefix;
        std::cout << (isLast ? "└── " : "├── ");
        
        if (node->isLeafNode()) {
            std::cout << "🍃 LEAF [" << node->triangleIndices.size() << " triangles]";
        } else {
            std::string axisName = (node->splitAxis == 0) ? "X" : 
                                  (node->splitAxis == 1) ? "Y" : "Z";
            std::cout << "🌿 NODE [" << axisName << "=" << std::fixed << std::setprecision(2) 
                      << node->splitPos << "]";
        }
        
        std::cout << " (ID:" << node->nodeId << ", D:" << node->depth << ")";
        std::cout << " bbox[" << std::fixed << std::setprecision(2) 
                  << node->bbox.min.x << "," << node->bbox.min.y << "," << node->bbox.min.z 
                  << " → " << node->bbox.max.x << "," << node->bbox.max.y << "," << node->bbox.max.z << "]" << std::endl;
        
        std::string newPrefix = prefix + (isLast ? "    " : "│   ");
        if (node->left || node->right) {
            if (node->left) {
                printTree(node->left, newPrefix, !node->right);
            }
            if (node->right) {
                printTree(node->right, newPrefix, true);
            }
        }
    }
    
    static void printTreeStats(const std::unique_ptr<BVHNode>& node, int& totalNodes, int& leafNodes, 
                              int& maxDepth, int& totalTriangles) {
        if (!node) return;
        
        totalNodes++;
        maxDepth = std::max(maxDepth, node->depth);
        
        if (node->isLeafNode()) {
            leafNodes++;
            totalTriangles += node->triangleIndices.size();
        }
        
        printTreeStats(node->left, totalNodes, leafNodes, maxDepth, totalTriangles);
        printTreeStats(node->right, totalNodes, leafNodes, maxDepth, totalTriangles);
    }
};

class BVHTraverser {
private:
    std::vector<Triangle>* triangles;
    std::unique_ptr<BVHNode>* root;
    mutable int nodeTests;
    mutable int triangleTests;
    
public:
    BVHTraverser(std::vector<Triangle>* tris, std::unique_ptr<BVHNode>* r) 
        : triangles(tris), root(r), nodeTests(0), triangleTests(0) {}
    
    bool intersect(const Vec3& rayOrigin, const Vec3& rayDir, 
                   float& closestT, Vec3& hitNormal, int& hitTriangleId) const {
        nodeTests = 0;
        triangleTests = 0;
        closestT = INFINITY;
        bool hit = false;
        
        intersectNode(root->get(), rayOrigin, rayDir, closestT, hitNormal, hitTriangleId, hit);
        
        return hit;
    }
    
    void getStatistics(int& nodes, int& triangles) const {
        nodes = nodeTests;
        triangles = triangleTests;
    }
    
private:
    void intersectNode(BVHNode* node, const Vec3& rayOrigin, const Vec3& rayDir,
                       float& closestT, Vec3& hitNormal, int& hitTriangleId, bool& hit) const {
        if (!node) return;
        
        nodeTests++;
        
        float tMin, tMax;
        if (!rayAABBIntersect(rayOrigin, rayDir, node->bbox, tMin, tMax)) {
            return;
        }
        
        if (tMin > closestT) return;
        
        if (node->isLeafNode()) {
            for (int triangleIdx : node->triangleIndices) {
                triangleTests++;
                
                float t, u, v;
                if (rayTriangleIntersectGeometric(rayOrigin, rayDir, (*triangles)[triangleIdx], t, u, v)) {
                    if (t < closestT && t > 1e-6f) {
                        closestT = t;
                        hit = true;
                        hitTriangleId = triangleIdx;
                        
                        const Triangle& tri = (*triangles)[triangleIdx];
                        float w = 1.0f - u - v;
                        hitNormal = (tri.normals[0] * w + tri.normals[1] * u + tri.normals[2] * v).normalize();
                    }
                }
            }
        } else {
            intersectNode(node->left.get(), rayOrigin, rayDir, closestT, hitNormal, hitTriangleId, hit);
            intersectNode(node->right.get(), rayOrigin, rayDir, closestT, hitNormal, hitTriangleId, hit);
        }
    }
    
    bool rayTriangleIntersectGeometric(const Vec3& rayOrigin, const Vec3& rayDir, const Triangle& triangle, 
        float& t, float& u, float& v) const {
        const Vec3& A = triangle.vertices[0];
        const Vec3& B = triangle.vertices[1]; 
        const Vec3& C = triangle.vertices[2];

        Vec3 AB = B - A;
        Vec3 AC = C - A;
        Vec3 normal = AB.cross(AC);

        float NdotRay = normal.dot(rayDir);
        if (std::abs(NdotRay) < 1e-6f) {
            return false;
        }

        float d = normal.dot(A);
        t = (d - normal.dot(rayOrigin)) / NdotRay;

        if (t < 1e-6f) {
            return false;
        }

        Vec3 P = rayOrigin + rayDir * t;
        Vec3 AP = P - A;
        Vec3 n = normal.normalize();
        Vec3 u_axis = AB.normalize();
        Vec3 v_axis = n.cross(u_axis);

        float AP_u = AP.dot(u_axis);
        float AP_v = AP.dot(v_axis);
        float AB_u = AB.dot(u_axis);
        float AB_v = AB.dot(v_axis);
        float AC_u = AC.dot(u_axis);
        float AC_v = AC.dot(v_axis);

        float det = AB_u * AC_v - AB_v * AC_u;
        if (std::abs(det) < 1e-6f) {
            return false;
        }

        u = (AP_u * AC_v - AP_v * AC_u) / det;
        v = (AB_u * AP_v - AB_v * AP_u) / det;
        float w = 1.0f - u - v;

        return (u >= 0.0f && v >= 0.0f && w >= 0.0f);
    }
};

struct Light {
    Vec3 position = Vec3(0, 0, 5);
    Vec3 intensity = Vec3(1, 1, 1);
};

struct Material {
    Vec3 ambient = Vec3(0.1, 0.1, 0.1);
    Vec3 diffuse = Vec3(0.7, 0.2, 0.2);
    Vec3 specular = Vec3(1.0, 1.0, 1.0);
    float shininess = 32.0f;
};

class OBJLoader {
public:
    static std::vector<Triangle> loadOBJ(const std::string& filename) {
        std::vector<Triangle> triangles;
        std::vector<Vec3> vertices;
        std::vector<Vec3> normals;
        
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open OBJ file: " << filename << std::endl;
            return triangles;
        }
        
        std::string line;
        int triangleId = 0;
        bool hasNormals = false;
        
        std::cout << "Loading OBJ file: " << filename << std::endl;
        
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;
            
            if (prefix == "v") {
                float x, y, z;
                iss >> x >> y >> z;
                vertices.push_back(Vec3(x, y, z));
            }
            else if (prefix == "vn") {
                float x, y, z;
                iss >> x >> y >> z;
                normals.push_back(Vec3(x, y, z));
                hasNormals = true;
            }
            else if (prefix == "f") {
                std::string v1, v2, v3;
                iss >> v1 >> v2 >> v3;
                
                auto parseVertex = [&](const std::string& vertexStr) -> std::pair<int, int> {
                    if (vertexStr.find("//") != std::string::npos) {
                        size_t doubleSlash = vertexStr.find("//");
                        int vertexIndex = std::stoi(vertexStr.substr(0, doubleSlash)) - 1;
                        int normalIndex = std::stoi(vertexStr.substr(doubleSlash + 2)) - 1;
                        return {vertexIndex, normalIndex};
                    } else if (vertexStr.find('/') != std::string::npos) {
                        std::vector<std::string> parts;
                        std::stringstream ss(vertexStr);
                        std::string part;
                        while (std::getline(ss, part, '/')) {
                            parts.push_back(part);
                        }
                        
                        int vertexIndex = std::stoi(parts[0]) - 1;
                        int normalIndex = -1;
                        if (parts.size() == 3 && !parts[2].empty()) {
                            normalIndex = std::stoi(parts[2]) - 1;
                        }
                        return {vertexIndex, normalIndex};
                    } else {
                        int vertexIndex = std::stoi(vertexStr) - 1;
                        return {vertexIndex, -1};
                    }
                };
                
                auto [v1Idx, n1Idx] = parseVertex(v1);
                auto [v2Idx, n2Idx] = parseVertex(v2);
                auto [v3Idx, n3Idx] = parseVertex(v3);
                
                if (v1Idx < 0 || v1Idx >= (int)vertices.size() ||
                    v2Idx < 0 || v2Idx >= (int)vertices.size() ||
                    v3Idx < 0 || v3Idx >= (int)vertices.size()) {
                    continue;
                }
                
                Triangle tri;
                tri.vertices[0] = vertices[v1Idx];
                tri.vertices[1] = vertices[v2Idx];
                tri.vertices[2] = vertices[v3Idx];
                
                if (hasNormals && n1Idx >= 0 && n2Idx >= 0 && n3Idx >= 0 &&
                    n1Idx < (int)normals.size() && n2Idx < (int)normals.size() && n3Idx < (int)normals.size()) {
                    tri.normals[0] = normals[n1Idx];
                    tri.normals[1] = normals[n2Idx];
                    tri.normals[2] = normals[n3Idx];
                } else {
                    Vec3 edge1 = tri.vertices[1] - tri.vertices[0];
                    Vec3 edge2 = tri.vertices[2] - tri.vertices[0];
                    Vec3 faceNormal = edge1.cross(edge2).normalize();
                    
                    tri.normals[0] = faceNormal;
                    tri.normals[1] = faceNormal;
                    tri.normals[2] = faceNormal;
                }
                
                tri.id = triangleId++;
                tri.computeBBox();
                
                triangles.push_back(tri);
            }
        }
        
        file.close();
        
        std::cout << "Loaded " << vertices.size() << " vertices, " 
                  << normals.size() << " normals, " 
                  << triangles.size() << " triangles" << std::endl;
        
        if (!hasNormals) {
            std::cout << "Note: No vertex normals found, using computed face normals" << std::endl;
        }
        
        return triangles;
    }
};

Vec3 phong_shading(const Vec3& normal, const Vec3& position, const Vec3& view_pos,
                   const Light& light, const Material& material,
                   bool use_diffuse = true, bool use_specular = true) {
    Vec3 N = normal.normalize();
    Vec3 L = (light.position - position).normalize();
    Vec3 V = (view_pos - position).normalize();
    Vec3 R = (L * -1).reflect(N);

    Vec3 ambient = material.ambient;
    Vec3 diffuse = Vec3(0, 0, 0);
    Vec3 specular = Vec3(0, 0, 0);
    
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

int main(int argc, char** argv) {
    float diffuse_r = 0.6f, diffuse_g = 0.6f, diffuse_b = 0.6f;
    float specular_r = 0.8f, specular_g = 0.8f, specular_b = 0.8f;
    float shininess = 64.0f;
    bool use_diffuse = true, use_specular = true;
    bool show_stats = false;
    bool show_tree = false;
    std::string obj_filename = "hairball.obj";

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
        } else if (arg == "--obj" && i + 1 < argc) {
            obj_filename = argv[i+1];
            i += 1;
        } else if (arg == "--no-diffuse") {
            use_diffuse = false;
        } else if (arg == "--no-specular") {
            use_specular = false;
        } else if (arg == "--stats") {
            show_stats = true;
        } else if (arg == "--tree") {
            show_tree = true;
        }
    }

    std::cout << "Bucketed SAH BVH Ray Tracer:" << std::endl;
    
    std::vector<Triangle> triangles = OBJLoader::loadOBJ(obj_filename);
    
    if (triangles.empty()) {
        std::cerr << "Failed to load OBJ file: " << obj_filename << std::endl;
        return 1;
    }
    
    auto buildStart = std::chrono::high_resolution_clock::now();
    
    std::vector<int> triangleIndices;
    for (size_t i = 0; i < triangles.size(); i++) {
        triangleIndices.push_back(i);
    }
    
    BucketedSAHBuilder builder(&triangles, 4, 15);
    auto bvhRoot = builder.build(triangleIndices);
    
    auto buildEnd = std::chrono::high_resolution_clock::now();
    auto buildTime = std::chrono::duration_cast<std::chrono::milliseconds>(buildEnd - buildStart);
    std::cout << "BVH built in " << buildTime.count() << "ms" << std::endl;
    std::cout << "Total nodes created: " << builder.getTotalNodes() << std::endl;

    if (show_tree) {
        std::cout << "\nBVH Tree Structure:" << std::endl;
        std::cout << "=====================" << std::endl;
        TreeVisualizer::printTree(bvhRoot);
        
        int totalNodes = 0, leafNodes = 0, maxDepth = 0, totalTriangles = 0;
        TreeVisualizer::printTreeStats(bvhRoot, totalNodes, leafNodes, maxDepth, totalTriangles);
        
        std::cout << "\nTree Statistics:" << std::endl;
        std::cout << "  Total nodes: " << totalNodes << std::endl;
        std::cout << "  Leaf nodes: " << leafNodes << std::endl;
        std::cout << "  Internal nodes: " << (totalNodes - leafNodes) << std::endl;
        std::cout << "  Max depth: " << maxDepth << std::endl;
        std::cout << "  Triangles in leaves: " << totalTriangles << std::endl;
        std::cout << "  Average triangles per leaf: " << (float)totalTriangles / leafNodes << std::endl;
    }

    int width = 800, height = 600;
    CImg<unsigned char> image(width, height, 1, 3, 0);

    Light light;
    Material material;
    material.diffuse = Vec3(diffuse_r, diffuse_g, diffuse_b);
    material.specular = Vec3(specular_r, specular_g, specular_b);
    material.ambient = Vec3(0.15f, 0.15f, 0.15f);
    material.shininess = shininess;
    
    Vec3 camera_pos(0, 0, 3);
    float camera_fov = 35.0f * M_PI / 180.0f;

    AABB modelBounds;
    for (const auto& tri : triangles) {
        modelBounds.expand(tri.bbox);
    }

    Vec3 modelCenter = modelBounds.center();
    Vec3 modelSize = modelBounds.max - modelBounds.min;
    float maxDimension = std::max({modelSize.x, modelSize.y, modelSize.z});
    
    camera_pos = modelCenter + Vec3(maxDimension * 1.2f, maxDimension * 0.6f, maxDimension * 1.8f);
    light.position = modelCenter + Vec3(maxDimension * 1.8f, maxDimension * 2.0f, maxDimension * 1.5f);
    light.intensity = Vec3(1.3f, 1.3f, 1.3f);

    Vec3 cameraTarget = modelCenter;
    Vec3 cameraUp = Vec3(0, 1, 0);
    Vec3 cameraForward = (cameraTarget - camera_pos).normalize();
    Vec3 cameraRight = cameraForward.cross(cameraUp).normalize();
    cameraUp = cameraRight.cross(cameraForward).normalize();

    BVHTraverser traverser(&triangles, &bvhRoot);
    
    auto renderStart = std::chrono::high_resolution_clock::now();
    
    long totalNodeTests = 0;
    long totalTriangleTests = 0;
    
    #pragma omp parallel for reduction(+:totalNodeTests,totalTriangleTests)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float u = (float(x) + 0.5f) / float(width);
            float v = (float(y) + 0.5f) / float(height);
            
            float ndc_x = 2.0f * u - 1.0f;
            float ndc_y = 1.0f - 2.0f * v;
            
            float aspect_ratio = float(width) / float(height);
            
            Vec3 ray_dir = cameraForward + 
                          cameraRight * (ndc_x * aspect_ratio * tan(camera_fov / 2.0f)) +
                          cameraUp * (ndc_y * tan(camera_fov / 2.0f));
            ray_dir = ray_dir.normalize();
            
            float closest_t;
            Vec3 hit_normal;
            int hitTriangleId;
            
            Vec3 hit_color(0.0f, 0.0f, 0.0f);
            
            if (traverser.intersect(camera_pos, ray_dir, closest_t, hit_normal, hitTriangleId)) {
                Vec3 hit_point = camera_pos + ray_dir * closest_t;
                Vec3 color = phong_shading(hit_normal, hit_point, camera_pos, light, material, use_diffuse, use_specular);
                
                color.x = std::min(std::max(color.x, 0.0f), 1.0f);
                color.y = std::min(std::max(color.y, 0.0f), 1.0f);
                color.z = std::min(std::max(color.z, 0.0f), 1.0f);
                
                hit_color = color;
                
                if (show_stats) {
                    int nodeTests, triangleTests;
                    traverser.getStatistics(nodeTests, triangleTests);
                    totalNodeTests += nodeTests;
                    totalTriangleTests += triangleTests;
                }
            }
            
            image(x, y, 0) = static_cast<unsigned char>(hit_color.x * 255.0f);
            image(x, y, 1) = static_cast<unsigned char>(hit_color.y * 255.0f);
            image(x, y, 2) = static_cast<unsigned char>(hit_color.z * 255.0f);
        }
    }
    
    auto renderEnd = std::chrono::high_resolution_clock::now();
    auto renderTime = std::chrono::duration_cast<std::chrono::milliseconds>(renderEnd - renderStart);
    
    std::string output_filename = "hairball_32.png";
    image.save(output_filename.c_str());
    
    std::cout << "\nRendering completed!" << std::endl;
    std::cout << "Render time: " << renderTime.count() << "ms" << std::endl;
    std::cout << "Triangles: " << triangles.size() << std::endl;
    std::cout << "Image saved: " << output_filename << std::endl;
    
    if (show_stats) {
        std::cout << "\nBVH Performance Statistics:" << std::endl;
        std::cout << "Total node tests: " << totalNodeTests << std::endl;
        std::cout << "Total triangle tests: " << totalTriangleTests << std::endl;
        std::cout << "Average node tests per ray: " << (float)totalNodeTests / (width * height) << std::endl;
        std::cout << "Average triangle tests per ray: " << (float)totalTriangleTests / (width * height) << std::endl;
    }
    
    return 0;
}