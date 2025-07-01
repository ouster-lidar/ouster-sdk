Use Pose Optimizer with the Ouster-CLI
======================================


.. _ouster-cli-pose-optimizer:


The pose optimizer tool can be used with the ``ouster-cli`` to refine the trajectory produced by a SLAM algorithm.
To learn how to use the Pose Optimizer with python API, refer to the :ref:`Pose Optimizer Python API Example <pose-optimize-example>`.

Prerequisites
-------------

Before using the Pose Optimizer, one must have: 

- A :ref:`SLAM processed OSF file <ouster-cli-mapping>` as input which contains the trajectory.
- A JSON file that defines the constraints for optimization. The constraints are used to impose limitations 
  and define relationship between poses or points in the trajectory to guide the optimization process.

The following section describes the expected structure and content of the JSON file and provides
guidelines for creating it. The constraints mentioned in examples below correspond to the `Sample Data`_ .

Constraint JSON File Structure
------------------------------

The constraint file must include:

- A top-level key ``constraints`` that maps to a list of constraint objects.
- A ``key_frame_distance`` parameter which defines the distance (in meters) between the key nodes.

**Example:**

.. code-block:: json

    {
        "key_frame_distance": 1.0,
        "constraints": [
            // List of constraint objects explained in the next sections
        ]
    }

.. _weight-fields:

Weight Fields: Rotation and Translation
---------------------------------------

Several constraint types (explained later) support optional weight fields: ``rotation_weight`` and ``translation_weight``. 
These influence optimizer by informing it of the confidence in accuracy of the rotational or translation information provided.

**rotation_weight**

- Can be a single number (float or int), or a list of three numbers.
- Applies to the angular components: **roll**, **pitch**, and **yaw**.
- Higher value signifies greater confidence in the accuracy of angular components.
- Lower values reduce the influence of rotation in the optimization — helpful when rotational measurements are noisy or unreliable.
- A value of `0` disables optimization for that rotation axis.
- If specified as a list of three values, the optimizer treats them as relative weights across the three axes.

**Example:**

.. code-block:: json

    "rotation_weight": [2.0, 1.0, 0.5]

This means:
- Roll is weighted 4× more than yaw.
- Pitch is weighted 2× more than yaw.
- The optimizer prioritizes roll corrections the most.

**translation_weight**

- Can also be a single number or a list of three numbers.
- Applies to the positional components: **x**, **y**, and **z**.
- Similar to ``rotation_weight``, higher values signal stronger confidence in the position, and lower values give the optimizer more freedom to adjust.
- If provided as a list, values are treated as relative weights for each axis.

**Example:**

.. code-block:: json

    "translation_weight": [1.0, 1.0, 0.1]

This enforces strict alignment in x and y, while allowing more flexibility in z.


.. note::

    If weights are omitted, the optimizer assigns default value of ``1`` internally.


Constraint Types
----------------

The Pose Optimizer supports three constraint types:

1. ``ABSOLUTE_POSE``
2. ``RELATIVE_POINT_TO_POINT``
3. ``RELATIVE_POSE_TO_POSE``


Each type requires specific fields and validations.


ABSOLUTE_POSE
~~~~~~~~~~~~~

Defines an absolute pose measurement relative to the world coordinate frame.


**Required fields:**

- ``type``: Must be exactly ``"ABSOLUTE_POSE"``.
- ``timestamp``: A numeric timestamp that must lie within the start and end timestamps of the OSF file, not limited to any specific LidarScan.
- ``pose``: One of:
  
  * A dictionary with keys: ``rx``, ``ry``, ``rz``, ``x``, ``y``, ``z``.
  * A list of 6 values (6x1 Euler angle and position vector) or 16 values (4x4 matrix).

**Optional fields:**

- ``transformation``: If provided, must match the format of ``pose``, either as a valid dictionary or as a list.
  
  * If the ``pose`` is provided as a 6x1 vector, then the ``transformation`` must also be a 6x1 vector (list of 6 numbers).
  * If the ``pose`` is provided as a 4x4 matrix, then the ``transformation``, must be a 4x4 matrix (list of 16 numbers).
- ``rotation_weight``:

  * A single number (float or int) or a list of three numbers.
  * Corresponds to **roll**, **pitch**, and **yaw** confidence.
  * Higher values increase sensitivity to rotational deviations.
- ``translation_weight``:
  
  * A single number or list of three values.
  * Corresponds to confidence in the **x**, **y**, and **z** position.

See :ref:`weight-fields` for a detailed explanation.

.. note::
  
  If no ``ABSOLUTE_POSE`` constraint is provided, the system will assume the first node to be fixed i.e. first node pose value will not be changed.

Below is an example JSON code block for an ABSOLUTE_POSE constraint:


.. code-block:: json

    {
      "constraints": [
        {
          "type": "ABSOLUTE_POSE",
          "timestamp": 411107223370,
          "pose": {
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "x": 12,
            "y": 30,
            "z": 0.0
          },
          "transformation": {
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
          },
          "rotation_weight": 1.0,
          "translation_weight": 1.0
        }
      ]
    }


RELATIVE_POINT_TO_POINT
~~~~~~~~~~~~~~~~~~~~~~~~
The ``RELATIVE_POINT_TO_POINT`` constraint is used to align a point from one point cloud to a corresponding point in another point cloud.

Each constraint of this type **must** include the following:

**Required fields:**

- ``type``: Must be ``"RELATIVE_POINT_TO_POINT"``.
- ``point_a`` and ``point_b``: Each must include:
  
  * ``row``: A numeric value > 0
  * ``col``: A numeric value > 0
  * ``timestamp``: The frame timestamp which is the lidar scan's first valid timestamp
  * ``return_idx`` (optional): Specifies which range return to use 1 for the first return or 2 for the second return. Default is 1

**Optional field:**

- ``translation_weight``: A number or list of three numbers.

See :ref:`weight-fields` for a detailed explanation.

**How to get valid timestamps:**

The frame timestamp can be obtained from the LidarScan object, which contains a list of valid timestamps for each point in the scan.

.. code-block:: python

    import numpy as np
    from ouster.sdk import open_source
    from ouster.sdk.core import LidarScan

    def main():
        osf = 'sample.osf'
        source = open_source(osf)

        idx = 0
        for scans in source:
            for sensor_id, scan in enumerate(scans):
                if not isinstance(scan, LidarScan):
                    continue

                first_valid_ts = scan.get_first_valid_column_timestamp()
    main()
                

Below is an example JSON code block for a ``RELATIVE_POINT_TO_POINT`` constraint:


.. code-block:: json
   
    {
      "constraints": [
        {
          "type": "RELATIVE_POINT_TO_POINT",
          "point_a": {
            "row": 9,
            "col": 542,
            "timestamp": 411107223370,
            "return_idx": 1
          },
          "point_b": {
            "row": 5,
            "col": 545,
            "timestamp": 531097181460,
            "return_idx": 1
          },
          "translation_weight": 1.0
        }
      ]
    }

RELATIVE_POSE_TO_POSE
~~~~~~~~~~~~~~~~~~~~~~

The ``RELATIVE_POSE_TO_POSE`` constraint defines a constraint between two poses in the trajectory.


**Required fields:**

- ``type``: Must be ``"RELATIVE_POSE_TO_POSE"``.
- ``pose_a`` and ``pose_b``: Each must include:
  
  * ``timestamp``: The frame timestamp which is the lidar scan's first valid timestamp

**Optional fields:**

- ``transformation``: Defines the relative pose from pose_a to pose_b
  
  * A dictionary (with ``rx``, ``ry``, ``rz``, ``x``, ``y``, ``z``), or
  * A 16-element list (4x4 matrix)
  * If omitted, transformation is auto-estimated using ICP matching.
- ``rotation_weight``: Number or list of three values.
- ``translation_weight``: Number or list of three values.

See :ref:`weight-fields` for a detailed explanation.

Below is an example JSON code block for a ``RELATIVE_POSE_TO_POSE`` constraint:


.. code-block:: json

    {
      "constraints": [
        {
          "type": "RELATIVE_POSE_TO_POSE",
          "pose_a": {
            "timestamp": 411107223370
          },
          "pose_b": {
            "timestamp": 531097181460
          },
          "transformation": [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
          ],
          "rotation_weight": 1.0,
          "translation_weight": 1.0
        }
      ]
    }


Example JSON File
-----------------

Below is an example JSON file that demonstrates all supported constraint types.
You can use this file with the downloaded OSF file to run the Pose Optimizer.


.. code-block:: json

        {
            "key_frame_distance": 2.0,
            "constraints": [
                {
                    "type": "ABSOLUTE_POSE",
                    "translation_weight": [1.0, 1.0, 1.0],
                    "rotation_weight": 2.0,
                    "timestamp": 411107223340,
                    "pose": {
                        "rx": 0,
                        "ry": 0,
                        "rz": 0,
                        "x": 12,
                        "y": 30,
                        "z": 1.3
                    }
                },
                {
                    "type": "RELATIVE_POINT_TO_POINT",
                    "translation_weight": 1.0,
                    "point_a": {
                        "row": 9,
                        "col": 542,
                        "return_idx": 1,
                        "timestamp": 411107223370
                    },
                    "point_b": {
                        "row": 5,
                        "col": 545,
                        "return_idx": 1,
                        "timestamp": 531097181460
                    }
                },
                {
                    "type": "RELATIVE_POSE_TO_POSE",
                    "translation_weight": 2.0,
                    "rotation_weight": 2.0,
                    "pose_a": {
                        "timestamp": 411107223370
                    },
                    "pose_b": {
                        "timestamp": 531097181460
                    },
                    "transformation": {
                        "rx": -0.000396419,
                        "ry": -0.0103704,
                        "rz": -0.0289703,
                        "x": 1.29628,
                        "y": 0.61089,
                        "z": -0.138204
                    }
                }
            ]
        }


Pose Optimizer Command
----------------------

This guide uses the `Sample Data`_ to demonstrate the Pose Optimizer. 
After installing the Ouster SDK, downloading the OSF file and creating the JSON constraints file as described earlier, you can run the Pose Optimizer using the command:

.. code:: bash

   ouster-cli source <OSF_FILENAME> pose_optimize <CONSTRAINT_JSON_FILE> <OUTPUT_OSF_FILENAME>


Visualizing the Results
-----------------------
After completing pose optimization, you can view the point cloud which is registered to the refined trajectory using these Ouster CLI commands:

- Replay the recording and visualize the full registered point cloud

.. code:: bash

   ouster-cli source <OUTPUT_OSF_FILENAME> viz --map

- Export a point cloud file and visualize the complete point cloud

.. code:: bash
   
   ouster-cli source <OUTPUT_OSF_FILENAME> save refined_point_cloud.ply
   ouster-cli source refined_point_cloud.ply viz

.. _Sample Data: https://studio.ouster.com/drive/92996?orgId=1
