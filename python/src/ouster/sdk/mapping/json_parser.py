import json
from typing import Tuple, Optional, List, Any, Dict


def validate_constraints_json(constraints_json_file: str) -> Tuple[Optional[Dict[str, Any]], bool, List[str]]:
    """
    Loads and validates the JSON file containing Pose Optimizer constraints.

    The function checks that:
      - The JSON file can be loaded.
      - The top-level "constraints" field exists.
      - Each constraint includes all required fields based on its type.

    It also validates that rotation_weight(s) and translation_weight(s)
    are either single floats/integers or lists of exactly three floats/integers.

    Additionally, it returns a boolean value indicating whether the first node
    needs to be fixed. (If no ABSOLUTE_POSE constraint exists, fix_first_node is True.)

    Returns:
        A tuple of (data, fix_first_node, errors) where:
          - data: Parsed JSON data if valid, otherwise None.
          - fix_first_node: True if no ABSOLUTE_POSE constraint exists, False otherwise.
          - errors: A list of error messages found during validation.
    """
    errors: List[str] = []
    try:
        with open(constraints_json_file, 'r') as f:
            data: Any = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        errors.append(f"Error reading JSON file: {e}")
        return None, False, errors

    if not isinstance(data, dict):
        errors.append("Validation error: JSON file does not contain an object at the top level.")
        return None, False, errors

    if "constraints" not in data:
        errors.append("Validation error: 'constraints' field is missing in the JSON file.")
        return None, False, errors

    constraints = data.get("constraints", [])
    found_absolute_pose = False

    def check_6x1_values(container: dict, index: int, key_prefix: str, constraint_label: str):
        """Helper to validate that a dictionary has rx, ry, rz, x, y, z."""
        for comp in ["rx", "ry", "rz", "x", "y", "z"]:
            if comp not in container:
                errors.append(
                    f"Validation error in constraint {index+1}: "
                    f"{constraint_label} component '{comp}' is missing."
                )

    def check_list_6_or_16(val: list, index: int, constraint_label: str) -> None:
        """Helper to check if the list is length 6 (6x1) or 16 (4x4)."""
        if len(val) not in (6, 16):
            errors.append(
                f"Validation error in constraint {index+1}: "
                f"{constraint_label} list must have length 6 or 16, got {len(val)}."
            )

    def validate_weight_field_absolute_constraint(field_value, field_name: str, i: int):
        """
        Validates that a weight field is either a single float/int or a list of exactly 3 floats/ints.
        Appends error messages to the 'errors' list if invalid.
        """
        if isinstance(field_value, (float, int)):
            # Single value is acceptable
            return
        if isinstance(field_value, list):
            if len(field_value) == 3 and all(isinstance(x, (float, int)) for x in field_value):
                return
            else:
                errors.append(
                    f"Validation error in constraint {i+1}: '{field_name}' must be a list of exactly three "
                    f"floats/ints. Provided list has {len(field_value)} elements and contains values: {field_value}."
                )
        else:
            errors.append(
                f"Validation error in constraint {i+1}: '{field_name}' must be either a single float/int or a list of "
                f"three floats/ints. Provided value is of type {type(field_value).__name__} with value: {field_value}."
            )

    def validate_weight_field_other_constraint(field_value, field_name: str, i: int):
        """
        Validates that a weight field is a single float/int. Appends error messages to the 'errors' list if invalid.
        """
        if isinstance(field_value, (float, int)):
            # Single value is acceptable only
            return
        # Otherwise invalid
        errors.append(
            f"Validation error in constraint {i+1}: '{field_name}' must be either "
            f"a single float/int."
        )

    for i, constraint in enumerate(constraints):
        constraint_type = constraint.get("type", "UNKNOWN")

        if constraint_type == "ABSOLUTE_POSE":
            found_absolute_pose = True
            pose_ts = constraint.get("timestamp", 0)
            if not pose_ts:
                errors.append(f"Validation error in constraint {i+1}: 'timestamp' is missing or invalid.")

            # Validate the pose field
            pose = constraint.get("pose")
            if pose is None:
                errors.append(f"Validation error in constraint {i+1}: 'pose' is missing.")
            else:
                if isinstance(pose, dict):
                    check_6x1_values(pose, i, "pose", "Pose")
                elif isinstance(pose, list):
                    check_list_6_or_16(pose, i, "pose")
                else:
                    errors.append(
                        f"Validation error in constraint {i+1}: 'pose' "
                        f"must be a dictionary or list, got {type(pose)}."
                    )

            # Validate optional transformation field
            transformation = constraint.get("transformation")
            if transformation is not None:
                if isinstance(transformation, dict):
                    check_6x1_values(transformation, i, "transformation", "Transformation")
                elif isinstance(transformation, list):
                    check_list_6_or_16(transformation, i, "transformation")
                else:
                    errors.append(
                        f"Validation error in constraint {i+1}: 'transformation' must "
                        f"be a dictionary or list, got {type(transformation)}."
                    )

            # Validate rotation_weight / translation_weight (may be single value or list of 3)
            rot_w = constraint.get("rotation_weight", 1.0)
            trans_w = constraint.get("translation_weight", 1.0)
            validate_weight_field_absolute_constraint(rot_w, "rotation_weight", i)
            validate_weight_field_absolute_constraint(trans_w, "translation_weight", i)

        elif constraint_type == "RELATIVE_POINT_TO_POINT":
            point_a = constraint.get("point_a")
            point_b = constraint.get("point_b")
            if not isinstance(point_a, dict) or not point_a:
                errors.append(f"Validation error in constraint {i+1}: 'point_a' is missing or invalid.")
            if not isinstance(point_b, dict) or not point_b:
                errors.append(f"Validation error in constraint {i+1}: 'point_b' is missing or invalid.")

            for label, point in [("point_a", point_a), ("point_b", point_b)]:
                if not isinstance(point, dict):
                    continue  # The error is already recorded above
                row = point.get("row", 0)
                col = point.get("col", 0)
                ts = point.get("timestamp", 0)
                ret = point.get("return_idx", None)
                if row <= 0:
                    errors.append(f"Validation error in constraint {i+1}: {label} row is missing or invalid.")
                if col <= 0:
                    errors.append(f"Validation error in constraint {i+1}: {label} col is missing or invalid.")
                if ts <= 0:
                    errors.append(f"Validation error in constraint {i+1}: {label} timestamp is missing or invalid.")
                if ret not in (1, 2):
                    errors.append(f"Validation error in constraint {i+1}: {label} return_idx ({ret}) is not 1 or 2.")

            translation_weight = constraint.get("translation_weight", 1.0)
            validate_weight_field_other_constraint(translation_weight, "translation_weight", i)

        elif constraint_type == "RELATIVE_POSE_TO_POSE":
            pose_a = constraint.get("pose_a")
            pose_b = constraint.get("pose_b")
            if not isinstance(pose_a, dict) or not pose_a:
                errors.append(f"Validation error in constraint {i+1}: 'pose_a' is missing or invalid.")
            if not isinstance(pose_b, dict) or not pose_b:
                errors.append(f"Validation error in constraint {i+1}: 'pose_b' is missing or invalid.")
            if isinstance(pose_a, dict) and pose_a:
                pose_a_ts = pose_a.get("timestamp", 0)
                if not pose_a_ts:
                    errors.append(f"Validation error in constraint {i+1}: 'pose_a' timestamp is missing or invalid.")
            if isinstance(pose_b, dict) and pose_b:
                pose_b_ts = pose_b.get("timestamp", 0)
                if not pose_b_ts:
                    errors.append(f"Validation error in constraint {i+1}: 'pose_b' timestamp is missing or invalid.")

            # transformation can be 4x4 (list of 16), 6x1 (list of 6), or dict
            transformation = constraint.get("transformation")
            if transformation is not None:
                if isinstance(transformation, dict):
                    check_6x1_values(transformation, i, "transformation", "Transformation")
                elif isinstance(transformation, list):
                    check_list_6_or_16(transformation, i, "transformation")
                else:
                    errors.append(
                        f"Validation error in constraint {i+1}: 'transformation' must be a dictionary or list, "
                        f"got {type(transformation)}."
                    )

            rot_w = constraint.get("rotation_weight", 1.0)
            trans_w = constraint.get("translation_weight", 1.0)
            validate_weight_field_other_constraint(rot_w, "rotation_weight", i)
            validate_weight_field_other_constraint(trans_w, "translation_weight", i)

        else:
            errors.append(f"Validation error in constraint {i+1}: Unknown constraint type '{constraint_type}'.")

    # If no ABSOLUTE_POSE constraint, first node should be fixed
    fix_first_node = not found_absolute_pose

    if errors:
        return None, fix_first_node, errors

    if "loss_function" in data:
        lf = data["loss_function"]
        if not isinstance(lf, str):
            errors.append(
                "Validation error: 'loss_function' must be a string, "
                f"got {type(lf).__name__}."
            )
        else:
            valid_losses = {"HuberLoss", "CauchyLoss", "SoftLOneLoss", "ArctanLoss", "TrivialLoss"}
            if lf not in valid_losses:
                errors.append(
                    f"Validation error: Unknown 'loss_function' '{lf}'. "
                    f"Valid options are: {', '.join(sorted(valid_losses))}."
                )

    if "loss_scale" in data:
        ls = data["loss_scale"]
        if not isinstance(ls, (int, float)):
            errors.append(
                "Validation error: 'loss_scale' must be a number (float), "
                f"got {type(ls).__name__}."
            )
    return data, fix_first_node, errors
