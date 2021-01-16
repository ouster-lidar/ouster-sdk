#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

class Diagnostics {

    double expected_frequency;
    diagnostic_updater::FrequencyStatus frequency_status;
    diagnostic_updater::Updater diagnostic_updater;

public:
    Diagnostics(double expected_freq, std::string name, std::string hardware_id, double tolerance = 0.2, int window_size=5) :
        expected_frequency(expected_freq),
        frequency_status(diagnostic_updater::FrequencyStatusParam(&expected_frequency, &expected_frequency, tolerance, window_size)),
        diagnostic_updater(ros::NodeHandle(), ros::NodeHandle("~"), ros::this_node::getName())
	{

        ROS_INFO("Expected frequency for %s = %.5f", name.c_str(), expected_frequency);

        diagnostic_updater.setHardwareID(hardware_id);
        diagnostic_updater.add(frequency_status);
    }

    void warn(const char * msg) {
        diagnostic_updater.broadcast(diagnostic_msgs::DiagnosticStatus::WARN, msg);
    }

    void fail(const char * msg) {
        diagnostic_updater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, msg);
    }

    void success(const char * msg) {
        diagnostic_updater.broadcast(diagnostic_msgs::DiagnosticStatus::OK, msg);
    }

    void tick() {
        frequency_status.tick();
    }

    void update() {
        diagnostic_updater.update();
    }
};

#endif // DIAGNOSTICS_H
