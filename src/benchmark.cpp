#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>

#include "spatial_tree.hpp"

#include "point_adapter.hpp"

#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using KittiSequence = std::map<int, std::vector<Point6D>>;

KittiSequence load_kitti_sequence(const std::string &filepath)
{
    KittiSequence sequence_data;
    std::ifstream file(filepath);
    std::string line;

    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open the file " << filepath << std::endl;
        return sequence_data;
    }

    int frame, track_id;
    std::string type, temp_str;
    float x, y, z, rot_y;
    float temp_h, temp_w, temp_l;

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        ss >> frame >> track_id >> type;
        if (type == "DontCare")
            continue;
        for (int i = 0; i < 7; ++i)
            ss >> temp_str;
        ss >> temp_h >> temp_w >> temp_l;
        ss >> x >> y >> z >> rot_y;

        const float scale = 1000.0f;
        Point6D p = {
            (uint32_t)((x + 50.0f) * scale),
            (uint32_t)((y + 50.0f) * scale),
            (uint32_t)((z + 50.0f) * scale),
            (uint32_t)((rot_y + 3.14f) * scale),
            0, 0};
        sequence_data[frame].push_back(p);
    }
    std::cout << "Loaded " << sequence_data.size() << " frames from " << filepath << std::endl;
    return sequence_data;
}

int main()
{
    std::string filepath = "data/data_tracking_label_2/all_data.txt";
    auto sequence = load_kitti_sequence(filepath);
    if (sequence.empty())
        return 1;

    SpatialTree<D> my_index;
    bgi::rtree<Point6D, bgi::rstar<16>> rtree;

    double my_total_insert_time = 0.0, rtree_total_insert_time = 0.0;
    double my_total_query_time = 0.0, rtree_total_query_time = 0.0;

    std::cout << "\n--- Benchmark Simulation ---" << std::endl;

    for (auto const &[frame_num, points_in_frame] : sequence)
    {
        if (frame_num % 10 == 0)
        {
            std::cout << "Processing Frame: " << frame_num << " with " << points_in_frame.size() << " objects." << std::endl;
        }

        auto start_my_ins = std::chrono::high_resolution_clock::now();
        for (const auto &p : points_in_frame)
            my_index.insert(p);
        auto end_my_ins = std::chrono::high_resolution_clock::now();
        my_total_insert_time += std::chrono::duration<double>(end_my_ins - start_my_ins).count();

        auto start_rtree_ins = std::chrono::high_resolution_clock::now();
        for (const auto &p : points_in_frame)
            rtree.insert(p);
        auto end_rtree_ins = std::chrono::high_resolution_clock::now();
        rtree_total_insert_time += std::chrono::duration<double>(end_rtree_ins - start_rtree_ins).count();

        if (frame_num % 5 == 0 && !points_in_frame.empty())
        {
            Point6D query_point = points_in_frame.front();
            double radius = 1000.0;

            auto start_my_q = std::chrono::high_resolution_clock::now();
            my_index.query_radius(query_point, radius, [](const Point<D> &result) {});
            auto end_my_q = std::chrono::high_resolution_clock::now();
            my_total_query_time += std::chrono::duration<double>(end_my_q - start_my_q).count();

            auto start_rtree_q = std::chrono::high_resolution_clock::now();
            std::vector<Point6D> result_rtree;
            rtree.query(bgi::nearest(query_point, 100), std::back_inserter(result_rtree));
            auto end_rtree_q = std::chrono::high_resolution_clock::now();
            rtree_total_query_time += std::chrono::duration<double>(end_rtree_q - start_rtree_q).count();
        }
    }

    std::cout << "\n--- Final Benchmark Results ---" << std::endl;
    std::cout << "--- Total Insert Time ---" << std::endl;
    std::cout << "My Index : " << my_total_insert_time << " seconds" << std::endl;
    std::cout << "R*-Tree  : " << rtree_total_insert_time << " seconds" << std::endl;
    std::cout << "\n--- Total Query Time ---" << std::endl;
    std::cout << "My Index : " << my_total_query_time << " seconds" << std::endl;
    std::cout << "R*-Tree  : " << rtree_total_query_time << " seconds" << std::endl;

    return 0;
}