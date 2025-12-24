#pragma once

#include <string>

#include "ouster/pose_optimizer_constraint.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {

// Forward declaration - ValidatorIssues is defined in metadata.h
namespace core {
struct ValidatorIssues;
}

namespace mapping {

/**
 * Parse and validate constraints configuration from JSON.
 *
 * This function parses a JSON string containing constraints configuration
 * and validates the format without populating any specific structures.
 *
 * @throw runtime_error if the text is not valid json.
 *
 * @param[in] json_data JSON string containing constraints configuration.
 * @param[out] issues The specific issues parsing the constraints configuration.
 *
 * @return If parsing was successful (no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_constraints(const std::string& json_data,
                                    core::ValidatorIssues& issues);

/**
 * Parse and validate constraints configuration from JSON into a SolverConfig.
 *
 * This function parses a JSON string containing solver configuration and
 * constraints, populating the provided SolverConfig object with validated
 * parameters and constraint objects.
 *
 * @throw runtime_error if the text is not valid json.
 *
 * @param[in] json_data JSON string containing constraints configuration.
 * @param[out] solver_config The solver config object to populate with
 *                               parsed constraints and parameters.
 * @param[out] issues The specific issues parsing the constraints configuration.
 *
 * @return If parsing was successful (no critical issues)
 */
OUSTER_API_FUNCTION
bool parse_and_validate_constraints(const std::string& json_data,
                                    SolverConfig& solver_config,
                                    core::ValidatorIssues& issues);

/**
 * Serialize the solver config to JSON.
 *
 * @param[in] solver_config The solver config to serialize.
 *
 * @return A string containing the JSON serialization of the solver config.
 */
OUSTER_API_FUNCTION
std::string serialize_constraints_to_json(const SolverConfig& solver_config);

}  // namespace mapping
}  // namespace sdk
}  // namespace ouster
