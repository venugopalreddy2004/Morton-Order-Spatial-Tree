# Morton-Ordered Spatial Tree (MOST)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Language: C++17](https://img.shields.io/badge/Language-C++17-blue.svg)](https://isocpp.org/)

A novel C++17 implementation of a spatial index that significantly outperforms traditional R*-trees for dynamic, high-dimensional workloads. **MOST (Morton-Ordered Spatial Tree)** maps 6D coordinates to a 1D Z-order curve and leverages a high-performance B-Tree, demonstrating a scalable and efficient solution for challenges in robotics, autonomous systems, and real-time tracking.

The core achievement is a **>790x speedup in query performance** against the state-of-the-art Boost.Geometry R*-tree on the industry-standard KITTI dataset.

---

## Benchmark Analysis

Performance was evaluated against the `Boost.Geometry R*-tree` (a state-of-the-art implementation) using the complete **KITTI Object Tracking training dataset**. The workload simulates a real-world scenario by processing object data frame-by-frame.

**Machine Configuration:** `12th Gen Intel® Core™ i7-12650H × 16, 16GB DDR5 RAM, GCC 13.3.0 on Ubuntu 24.04`

| Metric (Full KITTI Training Set) | MOST (This Project) | Boost R*-tree (SOTA) | Performance Gain |
| :--- | :--- | :--- | :--- |
| **Total Insert Time** | **`0.0757 s`** | `0.4842 s` | **6.4x Faster** 👑 |
| **Total Query Time** | **`0.0004 s`** | `0.3309 s` | **790x Faster** 🔥 |

---

## The Problem: The Curse of Dimensionality

Standard spatial indexes like R*-trees are exceptionally efficient in 2D or 3D. However, their performance degrades rapidly in higher dimensions (6D+), a phenomenon known as the "Curse of Dimensionality." For a 6-DOF object pose (`x, y, z, roll, pitch, yaw`), R*-tree bounding boxes become inefficient, suffering from high volume and significant overlap. In dynamic workloads with frequent insertions, this leads to costly tree restructuring and slow query resolution.

## The MOST Architecture

This index overcomes these limitations with a specialized, three-stage architecture:

1.  **Morton Encoding (Z-Order Curve):** The 6D search space is collapsed into a 1D representation. By interleaving the bits of the 6 coordinates, we generate a single, high-precision `__int128_t` key that preserves data locality. Points that are close in 6D space are highly likely to be close on the 1D number line.

2.  **`tlx::BTreeMultiMap`:** Instead of a complex tree of overlapping hyper-rectangles, we leverage a cache-friendly, high-performance B-Tree. B-Trees are masters of 1D sorted data, enabling extremely fast insertions and 1D range scans, which are fundamental to our query strategy.

3.  **Filter-and-Refine Query Strategy:** We perform a rapid 1D range scan on the B-Tree to fetch a small superset of potential candidates (`filter`). Only then do we perform the precise, but more expensive, 6D distance calculations on this small, pre-filtered subset (`refine`). This avoids the geometric complexity that slows down R*-trees.

## Applications & Use Cases

The demonstrated performance gains are not just academic; they unlock new capabilities in several mission-critical domains:

### 1. Robotics and Autonomous Systems
*   **What:** Real-time collision avoidance, sensor fusion (LiDAR/RADAR), and Simultaneous Localization and Mapping (SLAM).
*   **Why:** An autonomous agent's world-view is composed of hundreds of objects, each with a 6-DOF pose. This world-state changes every millisecond.
    *   The **6.4x faster insertion speed** allows the system to update its understanding of the world with minimal latency, reducing the time between perception and decision.
    *   The **790x faster query speed** is mission-critical for proximity checks. It enables faster, more reliable collision prediction, allowing the vehicle to operate safely at higher speeds or in more complex environments.

### 2. Real-time Physics and Simulation (Game Dev, VR/AR)
*   **What:** Broad-phase collision detection in physics engines, and object/hand tracking in VR/AR.
*   **Why:** Interactive applications live on a strict time budget (e.g., 16ms for 60 FPS). The massive query speedup allows for more complex scenes with thousands of dynamic objects without dropping frames, crucial for the "broad-phase culling" step. The fast insertion speed is ideal for scenarios with frequently spawned or destroyed objects.

### 3. High-Dimensional Database Indexing
*   **What:** Building specialized indexing engines for databases that handle more than just 2D/3D geospatial data (e.g., location + time + sensor readings).
*   **Why:** For databases with high-throughput, write-intensive workloads, the insertion performance of the index is often the primary bottleneck. MOST provides a powerful alternative to R-trees for these specialized use cases.

### 4. Computational Biology & Molecular Dynamics
*   **What:** Simulating molecular interactions where proximity searches are the most computationally expensive step.
*   **Why:** Molecules are complex 3D structures whose interactions are defined by their 6-DOF pose. A significant speedup in this core operation could drastically reduce the time required for drug discovery simulations, making previously infeasible large-scale analyses possible.

---

## Project Structure
```
.
├── include/              # Header files for the spatial index
│   ├── spatial_tree.hpp
│   └── point_adapter.hpp
├── src/                  # Main benchmark source code
│   └── benchmark.cpp
├── third_party/          # External library dependencies
│   ├── tlx/
│   └── boost_1_86_0/
├── data/                 # Placeholder for KITTI dataset labels
│   └── training/
│       └── label_02/
└── README.md
```

## Getting Started

### Prerequisites
*   A C++17 compliant compiler (e.g., GCC 9+ or Clang 10+)
*   The `tlx` library for the B-Tree implementation.
*   The `Boost` library (v1.74+ recommended for full compatibility).
*   The KITTI Object Tracking dataset labels.

### Build & Run Instructions

1.  **Clone this repository:**
    ```bash
    git clone https://github.com/venugopalreddy2004/Morton-Order-Spatial-Tree.git
    cd Morton-Order-Spatial-Tree
    ```

2.  **Set up dependencies:**
    ```bash
    # Clone the tlx library into the third_party directory
    git clone https://github.com/tlx/tlx.git third_party/tlx

    # Download the latest Boost library and extract it into third_party/
    # Example for Boost 1.86.0:
    wget https://boostorg.jfrog.io/artifactory/main/release/1.86.0/source/boost_1_86_0.tar.gz
    tar -xzf boost_1_86_0.tar.gz -C third_party/
    ```

3.  **Download the dataset:**
    Download the **"training labels of tracking data set (9 MB)"** from the [KITTI Object Tracking Website](http://www.kitti-vision.org/data_tracking.php) and extract its contents into the `data/` directory.

4.  **Compile and Run the Benchmark:**
    From the root directory of the project, execute the compilation command:

    ```bash
    g++ -std=c++17 -O3 -I include -I third_party/boost_1_86_0 -I third_party/tlx src/benchmark.cpp -o bench
    ```
    Then, run the executable:
    ```bash
    ./bench
    ```
    The final benchmark results will be printed to the console upon completion.

## Future Work

- [ ] Implement and benchmark a robust `remove()` operation to evaluate performance on a fully dynamic (insert-update-delete) workload.
- [ ] Develop a dynamic `delta` calculation for the query filter, mathematically derived from the search radius and data distribution.
- [ ] Compare performance against other space-filling curves, such as the Hilbert curve, to analyze locality preservation trade-offs.
