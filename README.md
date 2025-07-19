# SAH-Based BVH Ray Tracer

A high-performance ray tracer implementation featuring Surface Area Heuristic (SAH) based Bounding Volume Hierarchy (BVH) acceleration for efficient ray-object intersection testing.

## Features

- **BVH Acceleration**: Implements Surface Area Heuristic (SAH) for optimal BVH construction
- **Multiple Geometric Primitives**: Support for spheres, triangles, and complex meshes
- **Efficient Bounding Box Implementation**: Fast AABB intersection testing
- **High-Quality Rendering**: Phong shading with diffuse, specular, and ambient lighting
- **Model Loading**: Support for OBJ file format using tiny_obj_loader
- **Image Output**: PNG image generation using CImg library

## Project Structure

```
Simple_ray_tracer/
├── Bounding_box/          # Sphere ray tracing with bounding box optimization
│   ├── sphere.cpp         # Main sphere ray tracer implementation
│   ├── tiny_obj_loader.h  # OBJ file loader
│   └── *.png             # Rendered sphere images
├── BVH_SAH/              # BVH with Surface Area Heuristic implementation
│   ├── bvh_raytracer.cpp # Main BVH ray tracer
│   ├── tiny_obj_loader.h # OBJ file loader
│   └── *.png             # Dragon and hairball renderings
├── triangle/             # Basic triangle ray tracing
│   ├── ray_tracer.cpp    # Simple triangle ray tracer
│   └── output.png        # Triangle rendering output
└── CImg-3.5.4/           # Image processing library
```

## Dependencies

- **CImg**: Image processing and output (included)
- **tiny_obj_loader**: OBJ file parsing (included)
- **C++11** or later
- **OpenMP** (optional, for parallelization)

## Installation and Setup

### Prerequisites
```bash
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install libx11-dev  # For CImg display capabilities
```

### Model Files Setup

**Important**: This repository does not include 3D model files (.obj) due to their large file sizes. You must provide your own OBJ files for testing the ray tracer.

**Required Model Files:**
- For BVH ray tracer: Place your OBJ files in the `BVH_SAH/` directory
- The code defaults to loading "hairball.obj" but can be changed via command line arguments
- For sphere ray tracer: Place sphere model in the `Bounding_box/` directory

**Recommended Test Models:**
- Stanford Dragon model
- Hairball model (default in code)
- Any complex mesh with sufficient triangle count for BVH testing

You can download test models from:
- Stanford 3D Scanning Repository
- Computer Graphics research datasets
- Open-source 3D model repositories

### Compilation and Execution

**Sphere Ray Tracer:**
```bash
cd Bounding_box/
g++ -O3 -fopenmp sphere.cpp -o sphere_raytracer -lX11 -lpthread
./sphere_raytracer
```

**BVH Ray Tracer:**
```bash
cd BVH_SAH/
g++ -O3 -fopenmp bvh_raytracer.cpp -o bvh_raytracer -lX11 -lpthread
./bvh_raytracer [options]

# Command line options:
# --obj filename.obj          # Specify OBJ file (default: hairball.obj)
# --buckets N                 # Number of SAH buckets (default: 12)
# --diffuse R G B            # Diffuse color (0.0-1.0)
# --specular R G B           # Specular color (0.0-1.0)
# --shininess N              # Shininess factor (default: 64.0)
# --no-diffuse               # Disable diffuse shading
# --no-specular              # Disable specular shading
# --stats                    # Show traversal statistics
# --tree                     # Print BVH tree structure

# Example usage:
./bvh_raytracer --obj dragon.obj --buckets 16 --stats --tree
```

**Triangle Ray Tracer:**
```bash
cd triangle/
g++ -O3 ray_tracer.cpp -o raytracer -lX11 -lpthread
./raytracer
```

## Performance Analysis

**BVH Acceleration Structure:**
- **Bucketed SAH Implementation**: Uses configurable bucket count (default: 12) for optimal split evaluation
- **Surface Area Heuristic (SAH)**: Minimizes expected ray traversal cost
- **Adaptive Tree Construction**: Automatically balances depth vs. triangle count
- **Efficient ray-box intersection testing**
- **Logarithmic time complexity** for ray-object intersections

**Optimization Techniques:**
- **Early ray termination** for closest intersection
- **OpenMP parallelization** support for multi-core rendering
- **Bounding box hierarchies** for fast spatial culling
- **Memory-efficient node storage** with smart pointers

**Configurable Parameters:**
- **Maximum triangles per leaf**: 4 (configurable in code)
- **Maximum tree depth**: 15 (configurable in code)
- **SAH bucket count**: 12 (configurable via --buckets parameter)
- **Split cost evaluation**: 0.125 traversal cost + intersection costs

**Performance Metrics:**
- **Build time**: BVH construction time in milliseconds
- **Render time**: Total ray tracing time
- **Node tests per ray**: Average BVH nodes visited per ray (with --stats)
- **Triangle tests per ray**: Average triangle intersections tested per ray

## Technical Implementation

**Ray-Sphere Intersection:**
- Analytical solution using quadratic formula
- Efficient normal computation
- Multiple intersection handling

**Ray-Triangle Intersection:**
- **Custom geometric algorithm** implementation
- **Barycentric coordinate calculation** for interpolation
- **Normal interpolation** using vertex normals when available
- **Backface culling** support
- **Numerical stability** improvements with epsilon testing

**BVH Construction Algorithm:**
- **Bucketed SAH evaluation** with configurable bucket count
- **Recursive tree building** with depth limiting
- **Fallback splitting** when SAH fails to find good splits
- **Automatic leaf creation** when cost analysis favors it
- **Memory-efficient node storage** using unique_ptr

## Configuration

**Model File Requirements:**
- **OBJ format** with triangular faces
- **Vertex normals** (recommended for smooth shading, computed if missing)
- **Reasonable triangle count** (1K-1M triangles for testing)
- **Well-formed geometry** (no degenerate triangles)

**BVH Parameters (configurable in code):**
- **maxTrianglesPerLeaf**: 4 (controls leaf node size)
- **maxDepth**: 15 (prevents excessive tree depth)
- **buckets**: 12 (SAH evaluation granularity, configurable via command line)

**Rendering Parameters:**
- **Image resolution**: 800x600 (configurable in main function)
- **Camera parameters**: Position, FOV, automatically computed from model bounds
- **Lighting**: Single point light, automatically positioned relative to model
- **Material properties**: Configurable via command line arguments

**Shading Options:**
- **Phong shading model** with ambient, diffuse, and specular components
- **Configurable material properties** (diffuse color, specular color, shininess)
- **Optional component disabling** (--no-diffuse, --no-specular)
- **Automatic tone mapping** and color clamping

## Performance Benchmarking

The implementation provides comprehensive timing and statistics:

**Build Phase Metrics:**
- **BVH construction time** (milliseconds)
- **Total nodes created** (internal + leaf nodes)
- **Tree depth statistics** (with --tree option)
- **Memory usage** (nodes and triangles)

**Rendering Phase Metrics:**
- **Total rendering time** (milliseconds)
- **Rays per second** throughput
- **Average traversal statistics** (with --stats option):
  - Node tests per ray
  - Triangle tests per ray

**Tree Analysis (with --tree option):**
- **Complete tree structure** visualization
- **Node-by-node breakdown** with bounding boxes
- **Split axis and position** information
- **Leaf triangle distribution** statistics

## Future Enhancements

- Advanced materials (glass, metal, etc.)
- Global illumination techniques
