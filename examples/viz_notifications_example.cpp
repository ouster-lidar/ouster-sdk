#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "ouster/core/mesh.h"
#include "ouster/core/typedefs.h"
#include "ouster/viz/point_viz.h"

using namespace ouster::sdk;
int main(void) {
    // Initialize visualizer and add keyboard/mouse callbacks
    viz::PointViz viz("Viz example");
    viz::add_default_controls(viz);

    // Make the window visible
    viz.visible(true);
    viz.notifications_enabled = true;

    constexpr float m_2pi = 2.0f * M_PI;
    auto start = std::chrono::steady_clock::now();

    // Show quick welcome notification with default duration (2.0 seconds)
    viz.set_notification("Hello!");

    // For longer messages that need more reading time, use a custom duration
    double custom_notification_duration = 3.0;
    std::array<std::string, 3> messages = {"Press ESC to quit.",
                                           "Notifications display in the top-right corner.",
                                           "The cuboid rotates and changes color."};

    viz::vec4f rgba = {1.0f, 0.0f, 0.0f, 1.0f};
    core::mat4d transform = core::mat4d::Identity();
    auto cuboid_ptr = std::make_shared<viz::Cuboid>(transform, rgba);
    viz.add(cuboid_ptr);

    while (viz.running()) {
        auto elapsed =
            std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        auto seconds_since_start = static_cast<int>(elapsed);
        if (!viz.notification_active()) {
            // Cycle through multiple instructional messages
            viz.set_notification(
                messages[static_cast<int>(seconds_since_start / custom_notification_duration) %
                         messages.size()],
                custom_notification_duration);
        }

        float phase = elapsed;
        while (phase > m_2pi) {
            phase -= m_2pi;
        }

        // Rotate around Z axis
        double scale = 4.0 * cos(phase) + 8.0;
        transform.block<3, 3>(0, 0) =
            scale * Eigen::AngleAxisd(static_cast<double>(phase), Eigen::Vector3d::UnitZ())
                        .toRotationMatrix();
        cuboid_ptr->set_transform(transform);

        // Cycle through colors
        cuboid_ptr->set_rgba({(std::sin(phase) + 1.0f) / 2.0f,
                              (std::sin(phase / 2.0f) + 1.0f) / 2.0f,
                              (std::cos(phase) + 1.0f) / 2.0f, 1.0f});

        // Update the viz and run one iteration of the event loop
        viz.update();
        viz.run_once();
    }
}
