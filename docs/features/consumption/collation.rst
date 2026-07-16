
FrameSetSource Collation
====================

Natively each :ouster:class:`FrameSetSource <py=ouster.sdk.core.FrameSetSource|cpp=ouster::sdk::core::FrameSetSource>` produces a time-ordered stream of :ouster:class:`FrameSet <py=ouster.sdk.core.FrameSet|cpp=ouster::sdk::core::FrameSet>` objects each containing a single frame from a single sensor. In the event the source contains multiple sensors the resulting stream will cycle between a single frame from each sensor. This is sufficient by itself if you want to process each sensor's data independently. However, if the sensors have an overlapping field of view it can help to combine the frames from each sensor to get a better picture of the surroundings for applications such as object detection, localization and SLAM. 

To do this, the SDK uses an approach called "Frame Collation". It consumes frames from an underlying FrameSetSource and attempts to group them into a ``FrameSet`` with a frame from each sensor while balancing overall latency (time waiting for the last frame in a set). Due to this latency constraint, there is no guarantee that each output ``FrameSet`` contains a frame from every sensor so you must be careful to handle that possibility in any downstream application. To minimize incomplete FrameSets, you should make sure your sensors are phase locked and outputting at the same output frequency.

You can see a visualization of the difference in output between a collated and uncollated ``FrameSetSource`` below:

.. figure:: /images/CollationDiagram.drawio.png
   :alt: Collation before and after
   :align: center

As you can see from the example below, there are many possible ways to collate a given set of frames and not every method is appropriate for every application. Each color represents a ``FrameSet`` output from a collation process.

.. figure:: /images/CollationExample.drawio.png
   :alt: Collation possibilities
   :align: center

Because of this and the variety of lidar setups collation in the SDK has two modes:

* Unsynchronized: Enabled manually or if sensors are not phase locked. Maximum latency in the collation is specified by a single parameter.
* Synchronized: Enabled if sensors are phase locked. Uses this property to more aggressively minimize latency using only the configuration of each lidar.

Each methodology is further explained in the sections below. Since the collation mechanism is composed over the base FrameSetSources (via the :ouster:class:`Collator <py=ouster.sdk.core.Collator|cpp=ouster::sdk::core::Collator>`) it is easy to implement your own if these do not fit your requirements.


Unsynchronized Collation
========================

Uses a very simple yet effective methodology and a timeout parameter ``dt_ns``. Iterates over the frames adding them to a list returning the current ``FrameSet`` and starting a new one if any of the following occurs:

Before adding the frame to the current set:

* If the set already contains a frame from this sensor.
* If the first frame in the current set was more than ``dt_ns`` nanoseconds earlier than the new frame.

After adding the frame to the current set:

* If the set contains one frame from each sensor.


Synchronized Collation
======================

If sensors are phase locked, they are spinning at approximately the same interval and relative phases. This allows making additional assumptions about how frames will arrive that can be used to limit latency.

This methodology starts by calculating the expected difference in completion times between the frames of each sensor based on their configuration. This is affected by settings including azimuth window, phase offset and if IMU or Zone Monitor packets are enabled.

With this information it then calculates the optimal ordering of sensors such that the first and last frame in a complete set are the smallest possible interval apart. 

It then iterates over the frames adding them to a list returning the current ``FrameSet`` if any of the following occurs:

Before adding the frame to the current set:

* If the new frame is more than 50% of a period later than one would expect relative to the first frame in the set.

After adding the frame to the current set:

* If the set contains one frame from each sensor.

