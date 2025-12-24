#include "ouster/constraint_config.h"

#include <Eigen/Dense>
#include <cmath>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/jsonpath/json_query.hpp>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "ouster/impl/json_tools.h"
#include "ouster/metadata.h"
#include "ouster/typedefs.h"

using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {

// Helper class for parsing constraints
class ConstraintParserImpl : public core::impl::JsonTools {
   public:
    ConstraintParserImpl(const jsoncons::json& root,
                         ouster::sdk::core::ValidatorIssues& issues,
                         mapping::SolverConfig& solver_config)
        : JsonTools(root, issues) {
        parse_and_validate_solver_config(solver_config);
    }

    void parse_and_validate_solver_config(
        mapping::SolverConfig& solver_config) {
        // Parse solver configuration parameters - make them optional with
        // defaults

        if (!parse_and_validate_item(
                issues_.information, "$.key_frame_distance",
                solver_config.key_frame_distance,
                make_verify_in_bounds<double>(
                    0.0, std::numeric_limits<double>::max()),
                true)) {
            solver_config.key_frame_distance = 1.0;
            default_message("$.key_frame_distance");
        }

        if (!parse_and_validate_item(issues_.information,
                                     "$.traj_rotation_weight",
                                     solver_config.traj_rotation_weight,
                                     make_verify_in_bounds<double>(
                                         std::numeric_limits<double>::epsilon(),
                                         std::numeric_limits<double>::max()),
                                     true)) {
            solver_config.traj_rotation_weight = 10.0;
            default_message("$.traj_rotation_weight");
        }

        if (!parse_and_validate_item(issues_.information,
                                     "$.traj_translation_weight",
                                     solver_config.traj_translation_weight,
                                     make_verify_in_bounds<double>(
                                         std::numeric_limits<double>::epsilon(),
                                         std::numeric_limits<double>::max()),
                                     true)) {
            solver_config.traj_translation_weight = 10.0;
            default_message("$.traj_translation_weight");
        }

        if (!parse_and_validate_item(
                issues_.information, "$.max_num_iterations",
                solver_config.max_num_iterations,
                make_verify_in_bounds<uint64_t>(1, UINT64_MAX))) {
            solver_config.max_num_iterations = 100;
            default_message("$.max_num_iterations");
        }

        if (!parse_and_validate_item(
                issues_.information, "$.function_tolerance",
                solver_config.function_tolerance,
                make_verify_in_bounds<double>(
                    0.0, std::numeric_limits<double>::max()))) {
            solver_config.function_tolerance = 1e-6;
            default_message("$.function_tolerance");
        }

        if (!parse_and_validate_item(
                issues_.information, "$.gradient_tolerance",
                solver_config.gradient_tolerance,
                make_verify_in_bounds<double>(
                    0.0, std::numeric_limits<double>::max()))) {
            solver_config.gradient_tolerance = 1e-10;
            default_message("$.gradient_tolerance");
        }

        if (!parse_and_validate_item(
                issues_.information, "$.parameter_tolerance",
                solver_config.parameter_tolerance,
                make_verify_in_bounds<double>(
                    0.0, std::numeric_limits<double>::max()))) {
            solver_config.parameter_tolerance = 1e-8;
            default_message("$.parameter_tolerance");
        }

        if (!parse_and_validate_item(issues_.information, "$.process_printout",
                                     solver_config.process_printout)) {
            solver_config.process_printout = true;
            default_message("$.process_printout");
        }

        // Parse loss_function with enum validation - optional with default
        if (!parse_and_validate_enum<std::string>(
                issues_.information, "$.loss_function",
                solver_config.loss_function,
                mapping::loss_function_from_string)) {
            solver_config.loss_function = mapping::LossFunction::TRIVIAL_LOSS;
            default_message("$.loss_function");
        }

        if (!parse_and_validate_item(
                issues_.information, "$.loss_scale", solver_config.loss_scale,
                make_verify_in_bounds<double>(
                    0.0, std::numeric_limits<double>::max()))) {
            solver_config.loss_scale = 1.0;
            default_message("$.loss_scale");
        }

        if (!parse_and_validate_item(issues_.information, "$.fix_first_node",
                                     solver_config.fix_first_node)) {
            solver_config.fix_first_node = false;
            default_message("$.fix_first_node");
        }

        // Parse constraints array using vectorized callback pattern
        auto parse_constraint_callback =
            [&](const std::string& constraint_path,
                const jsoncons::json& constraint_json) {
                return parse_single_constraint(constraint_path, constraint_json,
                                               solver_config);
            };

        // Use vectorized json_query to iterate through constraints array
        jsoncons::jsonpath::json_query(root_, "$.constraints[*]",
                                       parse_constraint_callback);
    }

   private:
    bool parse_single_constraint(const std::string& constraint_path,
                                 const jsoncons::json& constraint_json,
                                 mapping::SolverConfig& solver_config);
};

bool ConstraintParserImpl::parse_single_constraint(
    const std::string& constraint_path, const jsoncons::json& constraint_json,
    mapping::SolverConfig& solver_config) {
    if (!constraint_json.is_object()) {
        issues_.critical.emplace_back(constraint_path,
                                      "Constraint must be an object");
        return false;
    }

    std::string type_str;
    if (!parse_and_validate_item(issues_.critical, constraint_path + ".type",
                                 type_str)) {
        return false;
    }

    try {
        auto parse_weight_vector = [&](const std::string& key,
                                       std::vector<double>& output) {
            const std::string path = constraint_path + "." + key;
            if (!path_exists(path)) {
                return false;
            }
            return parse_and_validate_item(issues_.information, path + "[*]",
                                           output, static_cast<size_t>(3));
        };

        auto parse_rotation_weight = [&](double& output) {
            const std::string path = constraint_path + ".rotation_weight";
            if (!path_exists(path)) {
                return false;
            }
            jsoncons::json value_array =
                jsoncons::jsonpath::json_query(root_, path);
            if (value_array.empty()) {
                issues_.critical.emplace_back(
                    path, "rotation_weight is present but empty");
                return false;
            }
            const auto& value = value_array[0];
            if (value.is_number()) {
                output = value.as<double>();
                return true;
            }
            issues_.critical.emplace_back(
                path, "rotation_weight must be a single numeric value");
            return false;
        };

        auto constraint_type = mapping::constraint_type_from_string(type_str);
        std::unique_ptr<mapping::Constraint> constraint;

        switch (constraint_type) {
            case mapping::ConstraintType::ABSOLUTE_POSE: {
                auto abs_constraint =
                    std::make_unique<mapping::AbsolutePoseConstraint>();

                // Hard requirements: only timestamp and pose
                if (!parse_and_validate_item(issues_.critical,
                                             constraint_path + ".timestamp",
                                             abs_constraint->timestamp)) {
                    return false;
                }

                // Parse pose - handle different formats
                jsoncons::json pose_json = jsoncons::jsonpath::json_query(
                    root_, constraint_path + ".pose");
                if (pose_json.empty()) {
                    issues_.critical.emplace_back(
                        constraint_path + ".pose",
                        "Missing required pose field");
                    return false;
                }

                auto pose_value = pose_json[0];
                mat4d temp_pose = mat4d::Identity();

                if (pose_value.is_array() && pose_value.size() == 16) {
                    // Handle array format [16 elements]
                    std::vector<double> pose_data =
                        pose_value.as<std::vector<double>>();
                    JsonTools::decode_transform_array(temp_pose, pose_data);
                } else if (pose_value.is_object()) {
                    // Handle object format {"x": val, "y": val, "z":
                    // val, "rx": val, "ry": val, "rz": val}
                    double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
                    if (pose_value.contains("x")) {
                        x = pose_value["x"].as<double>();
                    }
                    if (pose_value.contains("y")) {
                        y = pose_value["y"].as<double>();
                    }
                    if (pose_value.contains("z")) {
                        z = pose_value["z"].as<double>();
                    }
                    if (pose_value.contains("rx")) {
                        rx = pose_value["rx"].as<double>();
                    }
                    if (pose_value.contains("ry")) {
                        ry = pose_value["ry"].as<double>();
                    }
                    if (pose_value.contains("rz")) {
                        rz = pose_value["rz"].as<double>();
                    }

                    // Convert euler angles to transformation matrix
                    Eigen::Array3d translation(x, y, z);
                    Eigen::Array3d rotation(rx, ry, rz);

                    // Create rotation matrix from Euler angles (ZYX
                    // order)
                    Eigen::AngleAxisd roll_angle(rotation.x(),
                                                 Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitch_angle(rotation.y(),
                                                  Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yaw_angle(rotation.z(),
                                                Eigen::Vector3d::UnitZ());

                    Eigen::Quaterniond quaternion =
                        yaw_angle * pitch_angle * roll_angle;
                    temp_pose.block<3, 3>(0, 0) = quaternion.matrix();
                    temp_pose.block<3, 1>(0, 3) = translation;
                } else {
                    issues_.critical.emplace_back(
                        constraint_path + ".pose",
                        "Pose must be either a 16-element array or "
                        "object with x,y,z,rx,ry,rz fields");
                    return false;
                }

                abs_constraint->pose = temp_pose;

                // Parse optional weights
                double rotation_weight = abs_constraint->rotation_weight;
                if (parse_rotation_weight(rotation_weight)) {
                    abs_constraint->rotation_weight = rotation_weight;
                }

                std::vector<double> trans_weights;

                // Parse translation_weight (optional)
                if (parse_weight_vector("translation_weight", trans_weights)) {
                    abs_constraint->translation_weights = Eigen::Array3d(
                        trans_weights[0], trans_weights[1], trans_weights[2]);
                }

                constraint = std::move(abs_constraint);
                break;
            }

            case mapping::ConstraintType::POSE_TO_POSE: {
                // Implementation similar to absolute pose but for
                // relative constraints
                auto pose_constraint =
                    std::make_unique<mapping::PoseToPoseConstraint>();

                // Parse timestamps
                if (!parse_and_validate_item(issues_.critical,
                                             constraint_path + ".timestamp1",
                                             pose_constraint->timestamp1) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".timestamp2",
                                             pose_constraint->timestamp2)) {
                    return false;
                }

                // Parse transformation (optional, defaults to identity)
                mat4d temp_pose = mat4d::Identity();
                jsoncons::json trans_json = jsoncons::jsonpath::json_query(
                    root_, constraint_path + ".transformation");
                if (!trans_json.empty()) {
                    auto trans_value = trans_json[0];
                    if (trans_value.is_array() && trans_value.size() == 16) {
                        std::vector<double> trans_data =
                            trans_value.as<std::vector<double>>();
                        JsonTools::decode_transform_array(temp_pose,
                                                          trans_data);
                    } else if (trans_value.is_object()) {
                        double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
                        if (trans_value.contains("x")) {
                            x = trans_value["x"].as<double>();
                        }
                        if (trans_value.contains("y")) {
                            y = trans_value["y"].as<double>();
                        }
                        if (trans_value.contains("z")) {
                            z = trans_value["z"].as<double>();
                        }
                        if (trans_value.contains("rx")) {
                            rx = trans_value["rx"].as<double>();
                        }
                        if (trans_value.contains("ry")) {
                            ry = trans_value["ry"].as<double>();
                        }
                        if (trans_value.contains("rz")) {
                            rz = trans_value["rz"].as<double>();
                        }

                        Eigen::Array3d translation(x, y, z);
                        Eigen::Array3d rotation(rx, ry, rz);
                        Eigen::AngleAxisd roll_angle(rotation.x(),
                                                     Eigen::Vector3d::UnitX());
                        Eigen::AngleAxisd pitch_angle(rotation.y(),
                                                      Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd yaw_angle(rotation.z(),
                                                    Eigen::Vector3d::UnitZ());
                        Eigen::Quaterniond quaternion =
                            yaw_angle * pitch_angle * roll_angle;
                        temp_pose.block<3, 3>(0, 0) = quaternion.matrix();
                        temp_pose.block<3, 1>(0, 3) = translation;
                    } else {
                        issues_.warning.push_back(
                            ValidatorIssues::ValidatorEntry(
                                constraint_path + ".transformation",
                                "Transformation must be either a 16-element "
                                "array or object with x,y,z,rx,ry,rz fields"));
                    }
                }
                pose_constraint->relative_pose = temp_pose;

                // Optional weights
                std::vector<double> trans_weights;
                double rotation_weight = pose_constraint->rotation_weight;

                if (parse_rotation_weight(rotation_weight)) {
                    pose_constraint->rotation_weight = rotation_weight;
                }

                if (parse_weight_vector("translation_weight", trans_weights)) {
                    pose_constraint->translation_weights = Eigen::Array3d(
                        trans_weights[0], trans_weights[1], trans_weights[2]);
                }

                constraint = std::move(pose_constraint);
                break;
            }

            case mapping::ConstraintType::POINT_TO_POINT: {
                auto pt_constraint =
                    std::make_unique<mapping::PointToPointConstraint>();

                // Parse all required fields
                if (!parse_and_validate_item(issues_.critical,
                                             constraint_path + ".timestamp1",
                                             pt_constraint->timestamp1) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".timestamp2",
                                             pt_constraint->timestamp2) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".row1",
                                             pt_constraint->row1) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".col1",
                                             pt_constraint->col1) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".return_idx1",
                                             pt_constraint->return_idx1) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".row2",
                                             pt_constraint->row2) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".col2",
                                             pt_constraint->col2) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".return_idx2",
                                             pt_constraint->return_idx2)) {
                    return false;
                }

                std::vector<double> trans_weights;
                if (parse_weight_vector("translation_weight", trans_weights)) {
                    pt_constraint->translation_weights = Eigen::Array3d(
                        trans_weights[0], trans_weights[1], trans_weights[2]);
                }

                constraint = std::move(pt_constraint);
                break;
            }

            case mapping::ConstraintType::ABSOLUTE_POINT: {
                auto abs_pt =
                    std::make_unique<mapping::AbsolutePointConstraint>();

                // Required fields
                if (!parse_and_validate_item(issues_.critical,
                                             constraint_path + ".timestamp",
                                             abs_pt->timestamp) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".row",
                                             abs_pt->row) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".col",
                                             abs_pt->col) ||
                    !parse_and_validate_item(issues_.critical,
                                             constraint_path + ".return_idx",
                                             abs_pt->return_idx)) {
                    return false;
                }

                jsoncons::json pos_json = jsoncons::jsonpath::json_query(
                    root_, constraint_path + ".absolute_position");
                if (pos_json.empty()) {
                    issues_.critical.emplace_back(
                        constraint_path + ".absolute_position",
                        "Missing required absolute_position field");
                    return false;
                }
                auto pos_val = pos_json[0];
                if (pos_val.is_object()) {
                    double x = 0, y = 0, z = 0;
                    if (pos_val.contains("x")) {
                        x = pos_val["x"].as<double>();
                    }
                    if (pos_val.contains("y")) {
                        y = pos_val["y"].as<double>();
                    }
                    if (pos_val.contains("z")) {
                        z = pos_val["z"].as<double>();
                    }
                    abs_pt->absolute_position = Eigen::Vector3d(x, y, z);
                } else if (pos_val.is_array() && pos_val.size() == 3) {
                    std::vector<double> arr = pos_val.as<std::vector<double>>();
                    abs_pt->absolute_position =
                        Eigen::Vector3d(arr[0], arr[1], arr[2]);
                } else {
                    issues_.critical.emplace_back(
                        constraint_path + ".absolute_position",
                        "absolute_position must be object {x,y,z} or array of "
                        "3 elements");
                    return false;
                }

                std::vector<double> trans_weights;
                if (parse_weight_vector("translation_weight", trans_weights)) {
                    abs_pt->translation_weights = Eigen::Array3d(
                        trans_weights[0], trans_weights[1], trans_weights[2]);
                }

                constraint = std::move(abs_pt);
                break;
            }

            default:
                issues_.critical.emplace_back(
                    constraint_path + ".type",
                    "Unsupported constraint type: " + type_str);
                return false;
        }

        if (constraint) {
            solver_config.constraints.push_back(std::move(constraint));
        }

    } catch (const std::exception& e) {
        issues_.critical.emplace_back(
            constraint_path,
            std::string("Failed to parse constraint: ") + e.what());
        return false;
    }
    return true;
}

namespace mapping {

bool parse_and_validate_constraints(const std::string& json_data,
                                    SolverConfig& solver_config,
                                    ValidatorIssues& issues) {
    try {
        auto json = jsoncons::json::parse(json_data);

        ConstraintParserImpl parser(json, issues, solver_config);

        return issues.critical.empty();

    } catch (const std::exception& ex) {
        issues.critical.emplace_back(
            "$", std::string("Failed to parse JSON: ") + ex.what());
        return false;
    }
}

bool parse_and_validate_constraints(const std::string& json_data,
                                    ValidatorIssues& issues) {
    // Use the full implementation but discard the SolverConfig result
    SolverConfig temp_config;
    return parse_and_validate_constraints(json_data, temp_config, issues);
}

std::string serialize_constraints_to_json(const SolverConfig& solver_config) {
    jsoncons::json root;

    // Serialize solver configuration parameters
    root["key_frame_distance"] = solver_config.key_frame_distance;
    root["traj_rotation_weight"] = solver_config.traj_rotation_weight;
    root["traj_translation_weight"] = solver_config.traj_translation_weight;
    root["max_num_iterations"] = solver_config.max_num_iterations;
    root["function_tolerance"] = solver_config.function_tolerance;
    root["gradient_tolerance"] = solver_config.gradient_tolerance;
    root["parameter_tolerance"] = solver_config.parameter_tolerance;
    root["process_printout"] = solver_config.process_printout;

    // Serialize loss function
    root["loss_function"] = to_string(solver_config.loss_function);
    root["loss_scale"] = solver_config.loss_scale;
    root["fix_first_node"] = solver_config.fix_first_node;

    // Serialize constraints array
    jsoncons::json constraints_array = jsoncons::json::array();
    for (const auto& constraint : solver_config.constraints) {
        if (!constraint) continue;

        jsoncons::json constraint_json;

        switch (constraint->get_type()) {
            case ConstraintType::ABSOLUTE_POSE: {
                auto abs_constraint =
                    dynamic_cast<const AbsolutePoseConstraint*>(
                        constraint.get());
                if (abs_constraint) {
                    constraint_json["type"] = "ABSOLUTE_POSE";
                    constraint_json["timestamp"] = abs_constraint->timestamp;

                    // Serialize 4x4 pose matrix as flat array
                    jsoncons::json pose_array = jsoncons::json::array();
                    for (int i = 0; i < 4; ++i) {
                        for (int j = 0; j < 4; ++j) {
                            pose_array.push_back(abs_constraint->pose(i, j));
                        }
                    }
                    constraint_json["pose"] = pose_array;

                    // Serialize optional weights
                    if (std::abs(abs_constraint->rotation_weight - 1.0) >
                        std::numeric_limits<double>::epsilon()) {
                        constraint_json["rotation_weight"] =
                            abs_constraint->rotation_weight;
                    }

                    if ((abs_constraint->translation_weights !=
                         Eigen::Array3d::Ones())
                            .any()) {
                        jsoncons::json trans_weights = jsoncons::json::array();
                        for (int i = 0; i < 3; ++i) {
                            trans_weights.push_back(
                                abs_constraint->translation_weights(i));
                        }
                        constraint_json["translation_weight"] = trans_weights;
                    }
                }
                break;
            }

            case ConstraintType::POSE_TO_POSE: {
                auto pose_constraint =
                    dynamic_cast<const PoseToPoseConstraint*>(constraint.get());
                if (pose_constraint) {
                    constraint_json["type"] = "POSE_TO_POSE";
                    constraint_json["timestamp1"] = pose_constraint->timestamp1;
                    constraint_json["timestamp2"] = pose_constraint->timestamp2;

                    // Serialize 4x4 relative pose matrix as flat array
                    jsoncons::json rel_pose_array = jsoncons::json::array();
                    for (int i = 0; i < 4; ++i) {
                        for (int j = 0; j < 4; ++j) {
                            rel_pose_array.push_back(
                                pose_constraint->relative_pose(i, j));
                        }
                    }
                    constraint_json["relative_pose"] = rel_pose_array;

                    // Serialize optional weights
                    if (std::abs(pose_constraint->rotation_weight - 1.0) >
                        std::numeric_limits<double>::epsilon()) {
                        constraint_json["rotation_weight"] =
                            pose_constraint->rotation_weight;
                    }

                    if ((pose_constraint->translation_weights !=
                         Eigen::Array3d::Ones())
                            .any()) {
                        jsoncons::json trans_weights = jsoncons::json::array();
                        for (int i = 0; i < 3; ++i) {
                            trans_weights.push_back(
                                pose_constraint->translation_weights(i));
                        }
                        constraint_json["translation_weight"] = trans_weights;
                    }
                }
                break;
            }

            case ConstraintType::POINT_TO_POINT: {
                auto pt_constraint =
                    dynamic_cast<const PointToPointConstraint*>(
                        constraint.get());
                if (pt_constraint) {
                    constraint_json["type"] = "POINT_TO_POINT";
                    constraint_json["timestamp1"] = pt_constraint->timestamp1;
                    constraint_json["timestamp2"] = pt_constraint->timestamp2;
                    constraint_json["row1"] = pt_constraint->row1;
                    constraint_json["col1"] = pt_constraint->col1;
                    constraint_json["return_idx1"] = pt_constraint->return_idx1;
                    constraint_json["row2"] = pt_constraint->row2;
                    constraint_json["col2"] = pt_constraint->col2;
                    constraint_json["return_idx2"] = pt_constraint->return_idx2;

                    // Serialize optional translation weights
                    if ((pt_constraint->translation_weights !=
                         Eigen::Array3d::Ones())
                            .any()) {
                        jsoncons::json trans_weights = jsoncons::json::array();
                        for (int i = 0; i < 3; ++i) {
                            trans_weights.push_back(
                                pt_constraint->translation_weights(i));
                        }
                        constraint_json["translation_weight"] = trans_weights;
                    }
                }
                break;
            }

            case ConstraintType::ABSOLUTE_POINT: {
                auto abs_pt = dynamic_cast<const AbsolutePointConstraint*>(
                    constraint.get());
                if (abs_pt) {
                    constraint_json["type"] = "ABSOLUTE_POINT";
                    constraint_json["timestamp"] = abs_pt->timestamp;
                    constraint_json["row"] = abs_pt->row;
                    constraint_json["col"] = abs_pt->col;
                    constraint_json["return_idx"] = abs_pt->return_idx;

                    jsoncons::json pos_obj = jsoncons::json::object();
                    pos_obj["x"] = abs_pt->absolute_position.x();
                    pos_obj["y"] = abs_pt->absolute_position.y();
                    pos_obj["z"] = abs_pt->absolute_position.z();
                    constraint_json["absolute_position"] = pos_obj;

                    if ((abs_pt->translation_weights != Eigen::Array3d::Ones())
                            .any()) {
                        jsoncons::json trans_weights = jsoncons::json::array();
                        for (int i = 0; i < 3; ++i) {
                            trans_weights.push_back(
                                abs_pt->translation_weights(i));
                        }
                        constraint_json["translation_weight"] = trans_weights;
                    }
                }
                break;
            }
        }

        if (!constraint_json.empty()) {
            constraints_array.push_back(constraint_json);
        }
    }

    root["constraints"] = constraints_array;

    return root.as<std::string>();
}

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
