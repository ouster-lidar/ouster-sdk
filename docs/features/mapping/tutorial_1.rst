
Obtain Lidar Pose and Calculate Pose Difference
-----------------------------------------------

The SLAM API outputs the sensor’s pose for each lidar frame, which you can use to determine the sensor’s orientation in your system. 
From the Lidar Poses, we can calculate the Pose difference between consecutive frames.

Once the SLAM is run on the current frames to compute poses using ``slam.update(frame_set)``.
Extract the first valid column in the frame, frame_pose and frame_ts.

The pose_differences function is used to compare the current pose to the previous one, calculating rotation and translation deltas between two poses.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/obtain_pose_calc_diff.py
         :language: python
         :start-after: [doc-stag-pose-diff]
         :end-before: [doc-etag-pose-diff]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/obtain_pose_calc_diff.py>`__

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude::  _snippets/cpp/obtain_pose_calc_diff.cpp
         :language: cpp
         :start-after: //! [doc-stag-pose-diff]
         :end-before: //! [doc-etag-pose-diff]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/cpp/obtain_pose_calc_diff.cpp>`__


.. rubric:: **Call pose_differences in the frame loop**

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/obtain_pose_calc_diff.py
         :language: python
         :start-after: [doc-stag-pose-diff-call]
         :end-before: [doc-etag-pose-diff-call]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/python/obtain_pose_calc_diff.py>`__

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/obtain_pose_calc_diff.cpp
         :language: cpp
         :start-after: //! [doc-stag-pose-diff-call]
         :end-before: //! [doc-etag-pose-diff-call]
         :class: doc-snippet scroll-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/_snippets/cpp/obtain_pose_calc_diff.cpp>`__