Ouster Sensor SDK
=================

.. include:: /../python/README.rst
    :start-after: [sdk-overview-start]
    :end-before: [sdk-overview-end]

Quick links
-----------

* :doc:`installation`
* :doc:`python/quickstart`
* :doc:`cpp/building`
* `Ouster ROS 1 driver`_
* :doc:`python/viz/index`
* :doc:`cli/getting-started`


.. figure:: /images/ouster-viz.png
    :align: center

    Ouster SDK CLI visualization of OS1 128 Rev 7 sample data

Compatibility with FW
---------------------

The Ouster SDK currently provides backwards compatibility with any FW 2.0 and later for all
releases. Older SDK versions are not, however, generally forward-compatible with later FW releases,
e.g., the SDK version 20210608 (ouster-sdk 0.2.0) is not compatible with FW 3.0.

.. note::

    Please note that compatibility does not indicate that upgrading between SDK versions can happen
    seamlessly. For information on breaking changes between releases, please the `Changelog`_.

The following table indicates the compatibility of each released SDK version and its FW compatibility:

===================================== ======= ======= ======= ======= ======= ======= ======= ======= =======
SDK Tag (Release) / Python SDK        FW 1.13 FW 2.0  FW 2.1  FW 2.2  FW 2.3  FW 2.4  FW 2.5  FW 3.0  FW 3.1
===================================== ======= ======= ======= ======= ======= ======= ======= ======= =======
C++ SDK 20231031 /  Python SDK 0.10.0 no      **yes** **yes** **yes** **yes** **yes** **yes** **yes** **yes**
C++ SDK 20230710 /  Python SDK 0.9.0  no      **yes** **yes** **yes** **yes** **yes** **yes** **yes** **yes**
C++ SDK 20230403 /  Python SDK 0.8.1  no      **yes** **yes** **yes** **yes** **yes** **yes** **yes** **yes**
C++ SDK 20230114 /  Python SDK 0.7.1  no      **yes** **yes** **yes** **yes** **yes** **yes** **yes** **yes**
C++ SDK 20220927 /  Python SDK 0.5.2  no      **yes** **yes** **yes** **yes** **yes** no      no      no
C++ SDK 20220826 /  Python SDK 0.5.1  no      **yes** **yes** **yes** **yes** **yes** no      no      no
C++ SDK 20220608 /  Python SDK 0.4.1  **yes** **yes** **yes** **yes** **yes** no      no      no      no
C++ SDK 20220504 /  Python SDK 0.4.0  **yes** **yes** **yes** **yes** **yes** no      no      no      no
C++ SDK 20220107 /  Python SDK 0.3.0  **yes** **yes** **yes** **yes** no      no      no      no      no
C++ SDK 20210608 /  Python SDK 0.2.1  **yes** **yes** **yes** no      no      no      no      no      no
C++ SDK 20201209 /    n/a             **yes** **yes** **yes** no      no      no      no      no      no
C++ SDK v1.13.0  /    n/a             **yes** no      no      no      no      no      no      no      no
===================================== ======= ======= ======= ======= ======= ======= ======= ======= =======

If you are a C++ SDK user who has upgraded to the latest FW but requires an older SDK version,
please contact our customer support or the Field Applications Engineer who works with you.

See the following section regarding when we will drop support for various FWs.

Supported Platforms and Deprecations
------------------------------------

Upcoming deprecations/dropped support can be found at our GH announcement of `Lifecycle Policies`_


Status and Contact Info
-----------------------

The Ouster SDK is currently provided as an early pre-1.0 preview. The APIs are subject to change in
every release.

For questions about using your Ouster hardware, you may find it useful to reference the `Ouster
sensor documentation`_ and/or contact `Ouster support`_. For issues specific to the SDK please use
the `GitHub issue tracker`_. Announcements for the Ouster SDK are posted in the `Ouster Github
announcements`_

.. _Ouster sensor documentation: https://static.ouster.dev/sensor-docs/index.html
.. _Ouster support: https://ouster.atlassian.net/servicedesk/customer/portal/8
.. _Github issue tracker: https://github.com/ouster-lidar/ouster_example/issues
.. _Ouster Github announcements: https://github.com/ouster-lidar/ouster_example/discussions/categories/announcements
.. _Changelog: https://github.com/ouster-lidar/ouster_example/blob/master/CHANGELOG.rst
.. _Ouster ROS 1 driver: https://github.com/ouster-lidar/ouster-ros
.. _Lifecycle Policies: https://github.com/ouster-lidar/ouster_example/discussions/532
