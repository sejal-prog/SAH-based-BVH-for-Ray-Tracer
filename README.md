# SAH-Based BVH Ray Tracer

A high-performance ray tracer implementing Bounding Volume Hierarchy (BVH) acceleration structures optimized with the Surface Area Heuristic (SAH) for efficient rendering of complex 3D scenes.

## Project Structure

```
SIMPLE_RAY_TRACER/
├── Bounding_box/           # Simple bounding box ray tracer
│   ├── sphere.cpp          # Main implementation
│   ├── sphere_raytracer    # Compiled executable
│   ├── sphere.obj          # Test sphere model
│   ├── tiny_obj_loader.h   # OBJ file loading library
│   └── *.png              # Rendered output images
├── BVH_SAH/               # Advanced BVH ray tracer
│   ├── bvh_raytracer.cpp  # BVH implementation with SAH
│   ├── bvh_raytracer      # Compiled executable
│   ├── Dragon.obj         # Stanford Dragon model (249K triangles)
│   ├── hairball.obj       # Complex procedural model (2.88M triangles)
│   └── *.png             # Performance test results
├── triangle/              # Basic triangle ray tracer
│   ├── ray_tracer.cpp     # Simple triangle intersection
│   └── output.png         # Basic rendering output
└── CImg-3.5.4/           # CImg library for image processing
    └── CImg.h             # Single-header image library
```

## Features

### Simple Bounding Box Ray Tracer (`Bounding_box/`)
- **Basic acceleration**: Two-level AABB hierarchy
- **OBJ file loading**: Support for triangle meshes
- **Phong shading**: Ambient, diffuse, and specular lighting
- **Multiple modes**: Procedural sphere, OBJ loading, analytic sphere
- **Configurable materials**: Adjustable diffuse, specular, and shininess

### Advanced BVH Ray Tracer (`BVH_SAH/`)
- **Hierarchical BVH**: Binary tree spatial acceleration structure
- **Surface Area Heuristic**: Intelligent split selection for optimal tree quality
- **Bucketed SAH**: O(n log n) construction with configurable bucket counts (4-32)
- **Performance analysis**: Detailed traversal statistics and timing measurements
- **Complex scene support**: Handles models with millions of triangles
- **Tree visualization**: Debug output showing BVH structure

## Building and Running

### Prerequisites
```bash
# Install required packages
sudo apt-get install build-essential libomp-dev imagemagick

# For PNG support (optional)
sudo apt-get install libpng-dev
```

### Compilation

**Simple Bounding Box Ray Tracer:**
```bash
cd Bounding_box
g++ -O3 -fopenmp sphere.cpp -o sphere_raytracer
```

**Advanced BVH Ray Tracer:**
```bash
cd BVH_SAH  
g++ -O3 -fopenmp bvh_raytracer.cpp -o bvh_raytracer
```

### Usage Examples

**Simple Ray Tracer:**
```bash
# Render sphere with default settings
./sphere_raytracer --obj sphere.obj

# Only diffuse lighting
./sphere_raytracer --obj sphere.obj --no-specular

# Only specular highlights  
./sphere_raytracer --obj sphere.obj --no-diffuse

# Custom material properties
./sphere_raytracer --obj sphere.obj --diffuse 0.8 0.2 0.2 --shininess 128

# Analytic sphere (perfect mathematical sphere)
./sphere_raytracer --analytic
```

**BVH Ray Tracer:**
```bash
# Render with default 12 buckets
./bvh_raytracer --obj Dragon.obj --stats

# Test different bucket counts
./bvh_raytracer --obj Dragon.obj --buckets 4 --stats
./bvh_raytracer --obj Dragon.obj --buckets 32 --stats

# Render complex scene
./bvh_raytracer --obj hairball.obj --stats

# Show tree structure (for debugging)
./bvh_raytracer --obj Dragon.obj --tree

# Custom material settings
./bvh_raytracer --obj Dragon.obj --diffuse 0.6 0.6 0.6 --shininess 64
```

## Performance Results

### Stanford Dragon (249,882 triangles)
| Buckets | Build Time | Render Time | Node Tests/Ray |
|---------|------------|-------------|----------------|
| 4       | 107ms      | 93ms        | 4.4           |
| 8       | 110ms      | 78ms        | 4.9           |
| 12      | 121ms      | 78ms        | 4.3           |
| 18      | 170ms      | 80ms        | 3.8           |
| 32      | 276ms      | 74ms        | 3.5           |

### Hairball Model (2,880,000 triangles)
| Buckets | Build Time | Render Time | Node Tests/Ray |
|---------|------------|-------------|----------------|
| 4       | 1146ms     | 10711ms     | 101.85        |
| 12      | 1143ms     | 8393ms      | 106.16        |
| 32      | 1414ms     | 6168ms      | 101.16        |

## Algorithm Implementation

### Core Data Structures
- **Vec3**: 3D vector operations (dot, cross, normalize)
- **AABB**: Axis-aligned bounding boxes with expansion and surface area
- **Triangle**: Geometric primitive with vertices, normals, and bounding box
- **BVHNode**: Tree nodes with spatial subdivision information

### Ray-Object Intersection
- **Ray-Triangle**: Barycentric coordinate method for robust intersection
- **Ray-AABB**: Slab method for efficient bounding box tests
- **Early termination**: Aggressive pruning during tree traversal

### BVH Construction
- **Object partitioning**: Centroid-based primitive assignment
- **SAH optimization**: Cost-based split selection using surface area ratios
- **Bucketed discretization**: Reduces complexity from O(n²) to O(n log n)
- **Fallback mechanisms**: Median splitting for degenerate cases

## Test Models

### Included Models
- **sphere.obj**: Simple test geometry (low complexity)
- **Dragon.obj**: Stanford Dragon model (medium complexity, 249K triangles)
- **hairball.obj**: Procedural model (high complexity, 2.88M triangles)

### Performance Characteristics
- **Simple scenes**: Optimal performance with 8-12 buckets
- **Complex scenes**: Continue benefiting from higher bucket counts up to 32
- **Memory usage**: Linear scaling with primitive count
- **Traversal efficiency**: 10-106 node tests per ray depending on scene complexity

## Technical Details

### Rendering Pipeline
1. **Geometric loading**: OBJ file parsing with normal generation
2. **BVH construction**: Hierarchical acceleration structure building
3. **Ray generation**: Perspective camera with configurable FOV
4. **Intersection testing**: Efficient ray-scene queries using BVH
5. **Shading**: Phong illumination model with ambient, diffuse, specular
6. **Image output**: PNG generation with tone mapping

### Optimization Features
- **Smart pointers**: Automatic memory management
- **Cache-friendly layout**: Triangle indices for better memory access
- **Parallel rendering**: OpenMP multi-threading for ray casting
- **Statistics collection**: Detailed performance analysis capabilities

## Future Enhancements

- **Linear BVH (LBVH)**: GPU-friendly construction using Morton codes
- **Ambient Occlusion**: Enhanced realism through secondary ray sampling
- **Advanced materials**: Support for textures and complex BRDF models
- **Real-time updates**: Dynamic scene modifications with incremental rebuilding

## References

- Pharr, M., Jakob, W., & Humphreys, G. (2016). *Physically Based Rendering: From Theory to Implementation*
- Wald, I. (2007). *On fast construction of SAH-based bounding volume hierarchies*
- Computer Graphics course materials, University of Freiburg

## Author

**Sejal Jadhav**  
Computer Graphics Lab Course  
University of Freiburg  
Supervised by: Prof. Dr. Ing. Matthias Teschner

---

*For detailed implementation analysis and performance evaluation, refer to the accompanying technical report.*
