
Using the CLI
-------------

Using the ``detect`` command
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Beginning in Ouster SDK 1.0, the Ouster CLI has the ``detect`` command, which instantiates and runs a
``DetectionEngine``, which adds objects to each frame in the provided source. The ``detect`` command is meant to precede
a consuming command, such as ``save`` or ``viz``. For example, to run the default ``DetectionEngine`` on a source and
save the resulting objects to an OSF dataset, run

.. literalinclude:: /../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-source-detect]
   :end-before: [doc-etag-source-detect]
   :class: doc-snippet

The ``detect`` command accepts an option for each field of ``ClassicDetectionConfig``:

* ``--save-instance-id-fields/--no-save-instance-id-fields``: whether to save per-pixel instance id fields.
* ``--cluster-filter-min-side-length``: minimum side length a cluster must have to be kept (meters).
* ``--cluster-filter-max-side-length``: maximum side length a cluster may have to be kept (meters); ignored when
  negative.
* ``--cluster-filter-min-vertical-size``: minimum size a cluster must have along the vertical dimension (meters).
* ``--cluster-filter-max-volume``: maximum volume a cluster may have to be kept (cubic meters); ignored when negative.

Run ``ouster-cli source <SOURCE> detect --help`` to see the default values. For example, to disable the maximum
volume filter, run

.. literalinclude:: /../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-source-detect-options]
   :end-before: [doc-etag-source-detect-options]
   :class: doc-snippet

Visualizing objects
^^^^^^^^^^^^^^^^^^^

`ouster-cli` will visualize objects saved to an OSF dataset by default.

.. literalinclude:: /../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-source-viz]
   :end-before: [doc-etag-source-viz]
   :class: doc-snippet

When running a command like the above, the objects in the OSF will appear as translucent wireframe cuboids.
The following is a screenshot of the visualizer displaying objects from an annotated dataset:

.. figure:: /images/detect.png
    :align: center
    :alt: A screenshot of the Ouster CLI visualizer showing objects from an annotated dataset.
    :width: 100%

    A screenshot of the Ouster CLI visualizer showing objects from an annotated dataset.
