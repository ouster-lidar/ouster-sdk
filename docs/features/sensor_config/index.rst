Sensor Configuration
====================    

The Ouster SDK provides a powerful and flexible way to discover, inspect, and configure your Ouster sensors.
This section of the documentation guides you through the process of adjusting a sensor's mutable settings (``sensor_config``)
and understanding its immutable properties (``sensor_info``).

Whether you need to optimize performance, manage network bandwidth, or ensure precise timing,
the SDK gives you full control over your sensor's operational parameters.

Key Capabilities
^^^^^^^^^^^^^^^^

* **Runtime Configuration**: Dynamically adjust key parameters like returns profile, lidar mode, and azimuth window.
* **Performance Tuning**: Optimize data rate and network bandwidth by reducing returns or narrowing the field of view.
* **Synchronization**: Seamlessly switch between internal, PTP, and GPS timing sources for multi-sensor precision.
* **Network Control**: Redirect data streams by updating UDP destination IPs and ports.
* **Transient vs. Persistent Changes**: Apply changes temporarily for testing, then persist them to memory to survive a reboot.


Best Practices
^^^^^^^^^^^^^^

- **Read before editing:** Fetch the current ``sensor_config`` first and change only the fields you specifically need to update.
- **Test transiently, then persist:** Apply updates without persistence, confirm the behavior, and persist only when you are satisfied.
- **Refresh metadata:** Call ``get_sensor_info()`` after changing returns, lidar mode, or ``azimuth_window`` so downstream code uses the correct layout.
- **Confirm external sync:** Ensure a PTP lock or GPS pulse is present before switching timing sources in production.

Key Tunable Parameters
^^^^^^^^^^^^^^^^^^^^^^

- **Returns profile & lidar mode:** Primary controls for balancing vertical resolution against packet bandwidth.
- **azimuth_window:** Narrows the horizontal field of view to trim data rate or focus on a region of interest.
- **UDP destination (udp_dest_host, lidar_port, imu_port):** Routes lidar and IMU packets to the proper host/ports.
- **Timing source:** Selects between internal clock, PTP 1588, or GPS for multi-sensor synchronization.

.. toctree::
   :hidden:
   :maxdepth: 1

   using-cli
   using-the-api
