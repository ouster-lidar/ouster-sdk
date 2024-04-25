==================================
Using the new ScanSource interface
==================================

.. contents::
   :local:
   :depth: 3

.. _scan-source-example:

In this example we are going to demonstrate the use of the new ScanSource API.


Using the open_source method
============================

The new API introduces a new method name ``open_source`` which allows users to handle different source types
using the same API. Current supported source types are live sensor, pcap file, or osf file. For example, to
open the same pcap file referenced in the main :ref:`Quick Start <quick-start>` using the simplified API
can be accomplished as follows:

.. code:: python

   >>> pcap_path = '<SAMPLE_DATA_PCAP_PATH>'
   >>> metadata_path = '<SAMPLE_DATA_JSON_PATH>'
   >>> from ouster.sdk import open_source
   >>> source = open_source(pcap_path, sensor_idx=0, meta=[metadata_path])


The ``source`` handle here acts the same as the handle returned by the ``pcap.Pcap`` constructor, with some
extra capabilities that we will cover later.  

Notice here that besides the ``pcap_path`` we pass two additional parameters: ``sensor_idx`` with a value
of zero, and ``meta`` which we set to the ``metadata_path`` to point to the sensor metadata associated with
the pcap file we are trying to open. Both parameters are optional and can be omitted. In case the ``meta``
parameter was not omitted, the ``open_source`` method will attempt to locate the metadata associated with
the pcap file based on location and the pcap file prefix. That being said, if ``SAMPLE_DATA_JSON_PATH`` is
located in the same folder as ``SAMPLE_DATA_PCAP_PATH`` and the two files share a prefix we can simplify the
above call to:

.. code:: python
   >>> source = open_source(pcap_path, sensor_idx=0)


The second parameter, ``sensor_idx``, allows users to select a specific sensor from the selected source.
That is because starting with ouster-sdk v0.11 Ouster added support for working with sensor data collected from
multiple sensors. The ``open_source`` method by default returns the more complete interface ``MultiScanSource``
which has the capability to interact with multiple sensor streams, which we will in next section. Setting the
value of ``sensor_idx`` to zero tells ``open_source`` we are only interested in LidarScan data coming from the
first sensor from this specific pcap file in case the file had more than one sensor. By doing so, the
``open_source`` method returns a less sophisticated interface ``ScanSource`` which is more familiar to SDK users
from previous versions.

The main different between the ``MultiScanSource`` and the ``ScanSource`` is the expected return of some
of the object methods. For example, when creating an iterator for a ``ScanSource`` object, the user would get
a single ``LidarScan`` object per iteration. Iterating over the contents of a ``MultiScanSource`` object always
yields a **list** of ``LidarScan(s)`` per iteration corresponding to the number of sensors stored in the pcap
file or whatever source type is being used. This is true even when the pcap file contains data for a single sensor.


On the other hand, if the user wants to open an osf file or access the a live sensor, all that changes is url
of the source. For example, to interact with a live sensor the user can execute the following snippet:

.. code:: python

   >>> sensor_url = '<SENSOR-HOSTNAME-OR-IP>'
   >>> from ouster.sdk import open_source
   >>> source = open_source(sensor_url, sensor_idx=0)


Obtaining sensor metadata
=========================
Every ScanSource holds a reference to the sensor metadata, which has crucial information that is important when
when processing the invidivual scans. Continuing the example, a user this can access the metadata through the
``metadata`` property of a ``ScanSource`` object: 

.. code:: python

   >>> print(source.metadata)


Iterating over Scans
====================

Once we have successfully obtain a handle to the ScanSource we can iterate over ``LidarScan`` objects stored in the
pcap file and manipulate each one individually. For example, let's say we want to print the frame id of the first 10
scans. We can achieve that using:

.. code:: python

   >>> ctr = 0
   >>> source_iter = iter(source)
   >>> for scan in source_iter:
   ...     print(scan.frame_id)
   ...     ctr += 1
   ...     if ctr == 10:
   ...         break


As we noted earlier, if we don't supply ``sensor_idx=0`` to the ``open_source`` method, the method will construct a
``MultiScanSource``, which always addresses a group of sensors. Thus, when iterating over the ``source`` the user
receives a collated set of scans from the addressed sensors per iteration. The ``MultiScanSource`` examines the
timestamp of every scan from every sensor and returns a list of scans that fit within the same time window as single
batch. The size of the batch is fixed corresponding to how many sensors contained in the pcap or osf file. However,
the collation could yield a null value if one or more of the sensors didn't produce a ``LidarScan`` object that fits
within the time frame of current batch or iteration. Thus, depending on the operation at hand it is crticial to check
if we got a valid ``LidarScan`` object when examining the iteration output of a ``MultiScanSource``.  If we are to
perform the same example as above when ``source`` is a handle to ``MultiScanSource`` and display the frame_id of
``LidarScan`` objects the belongs to the same batch on the same line the code needs to updated to the following:

.. code:: python

   >>> ctr = 0
   >>> source_iter = iter(source)
   >>> for scans in source_iter:
   ...     for scan in scans:    # source_iter here returns a list of scans
   ...         if scan:          # check if invidiual scan object is valid
   ...             print(scan.frame_id, end=', ')
   ...     print()   # new line for next batch
   ...     ctr += 1
   ...     if ctr == 10:
   ...         break


Note that when iterating over a ``MultiScanSource`` object, it always a list of scans, even when the underlying scan
source has only a single sensor. In this case, the iterator will yield a list with a single element per iteration.



Using indexing and slicing capabilities of a ScanSource
========================================================

One of the most prominent new features of the ScanSource API, (besides being able to address multi sensors), is the
ability to use indexing and slicing when accessing the stored scans within the ``LidarScan`` source. Currently, this
capability is only supported for non-live sources. That is to say, the functionality we are discussing can only be used
when accessing a pcap or an osf file. To enable this functionality we need to indicate that we want to manipulate the
source as an indexed one upon opening. Revisitng the previous pcap open example, that would be achieved as follows:
:


.. code:: python

   >>> pcap_path = '<SAMPLE_DATA_PCAP_PATH>'
   >>> from ouster.sdk import open_source
   >>> source = open_source(pcap_path, sensor_idx=0, index=True)

First note that we omitted the ``meta`` parameter since it can be populated automatically as we explained earlier.
Second you will noticed that we introduced a new parameter ``index`` with its value set to ``True`` (default is false),
The same parameter can be applied to when dealing with an osf file but not a live sensor.

Depending on the file size and the underlying file format there can be some delay before the file is fully indexed (OSF
file take much less time than pcap file to index). A progress bar will appear to indicate progress of the indexing.

Once the index is built up, then we can start using utilizing and interact with the ``ScanSource`` object to access scans
in the same manner we are dealing with a python list that holds reference to LidarScan objects.

For example to access the 10th LidarScan and print its frame id, we can do the following:

.. code:: python

   >>> print(source[10].frame_id)

Similarly we can access the last LidarScan object and print its frame_id using:

.. code:: python

   >>> print(source[-1].frame_id)


Alternatively we can instead request a range of scans using the python slice operator. For example, to request the first 10
scans from a ScanSource and print their frame ids, we can do the following:

.. code:: python

   >>> for scan in source[0:10]:
   ...     print(scan.frame_id)


Note we don't need to add any break here since the operation `source[0:10]` will only yield the first 10 ``LidarScan(s)``.

To print frame_id of the last 10 LidarScans we do:

.. code:: python

   >>> for scan in source[-11:-1]:
   ...     print(scan.frame_id)


Finally, as you would expect from a typical slice operation, you can also using negative step and also use a reversed
iteration as shown in the following example:

.. code:: python

   >>> for scan in source[0:10:2]:     # prints the frame_id of every second scan of the first 10 scans
   ...     print(scan.frame_id)

   >>> for scan in source[10:0:-1]:     # prints the frame_id of every scan of the first 10 scans in reverse
   ...     print(scan.frame_id)
