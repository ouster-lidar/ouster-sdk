#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "ouster/mesh.h"
#include "ouster/point_viz.h"
#include "ouster/typedefs.h"

using namespace ouster::sdk;

constexpr float M_2PI = 2.0f * M_PI;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: viz_mesh_example [stl file path]" << std::endl;
        return 1;
    }

    // Load a mesh from an STL file
    core::Mesh mesh;
    if (!mesh.load_from_stl(argv[1])) {
        std::cerr << "Unable to load file" << std::endl;
        return 1;
    }

    // Initialize visualizer and add keyboard/mouse callbacks
    viz::PointViz viz("Viz example");
    viz::add_default_controls(viz);

    // Add the mesh to the viz and update the viz state
    auto mesh_ptr = std::make_shared<viz::Mesh>(
        std::move(viz::Mesh::from_simple_mesh(mesh)));
    viz.add(mesh_ptr);

    // Make the window visible
    viz.visible(true);

    auto start = std::chrono::steady_clock::now();
    while (viz.running()) {
        auto time_since_start = std::chrono::duration<float>(
                                    std::chrono::steady_clock::now() - start)
                                    .count();
        while (time_since_start > M_2PI) {
            time_since_start -= M_2PI;
        }

        // Rotate around Z axis
        core::mat4d rotation = core::mat4d::Identity();
        rotation(0, 0) = cos(time_since_start);
        rotation(0, 1) = -sin(time_since_start);
        rotation(1, 0) = sin(time_since_start);
        rotation(1, 1) = cos(time_since_start);
        mesh_ptr->set_transform(core::mat4d_to_array(rotation));

        // Cycle through colors
        mesh_ptr->set_edge_rgba(
            {(std::sin(time_since_start) + 1.0f) / 2.0f,
             (std::sin(time_since_start / 2.0f) + 1.0f) / 2.0f,
             (std::cos(time_since_start) + 1.0f) / 2.0f, 1.0f});

        // Update the viz and run one iteration of the event loop
        viz.update();
        viz.run_once();
    }
}
