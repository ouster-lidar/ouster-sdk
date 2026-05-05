Unit Normal Estimation

This example demonstrates how to compute normal value from point cloud and range data.
This doc will be added to 1.0 SDK documentation. Currently it's a standalone example for
internal review and future usage.

.. code-block:: python

    import numpy as np
    from ouster.sdk import open_source
    from ouster.sdk import viz
    from ouster.sdk.core import (
        ChanField,
        XYZLut,
        normals,
        destagger,
        dewarp,
        stagger,
    )

    # Open a data source
    source = open_source("path/to/file.osf")
    infos = source.sensor_info

    # Build XYZ values in destaggered space
    luts = [XYZLut(info, use_extrinsics=True) for info in infos]

    sviz = viz.SimpleViz(infos)

    def scans_with_normals():
        # Use per-column poses to dewarp points into the world frame, and pass
        # per-column sensor origins in the same frame to the normals call. If you
        # want purely sensor-frame normals, skip dewarp and pass zeros for
        # sensor_origins_xyz instead.
        viz_modes_initialized = False
        for scans in source:
            for idx, scan in enumerate(scans):
                if scan is None:
                    continue
                info = infos[idx]
                lut = luts[idx]
                h, w = scan.h, scan.w

                range_field = scan.field(ChanField.RANGE)
                xyz = lut(range_field).reshape(h, w, 3)

                range_destaggered = destagger(info, range_field)
                xyz_destaggered = destagger(info, xyz)

                poses = scan.pose
                xyz_destaggered = dewarp(xyz_destaggered, poses)
                sensor_origins_xyz = (poses @ info.extrinsic)[:, :3, 3]

                normals2_destaggered = None

                if scan.has_field(ChanField.RANGE2):
                    range2_field = scan.field(ChanField.RANGE2)
                    xyz2 = lut(range2_field).reshape(h, w, 3)

                    range2_destaggered = destagger(info, range2_field)
                    xyz2_destaggered = destagger(info, xyz2)
                    xyz2_destaggered = dewarp(xyz2_destaggered, poses)

                    normals_destaggered, normals2_destaggered = normals(
                        xyz_destaggered,
                        range_destaggered,
                        xyz2_destaggered,
                        range2_destaggered,
                        sensor_origins_xyz=sensor_origins_xyz,
                    )
                else:
                    normals_destaggered = normals(
                        xyz_destaggered,
                        range_destaggered,
                        sensor_origins_xyz=sensor_origins_xyz,
                    )

                # Restagger and add the results back to the scan
                normals_staggered = stagger(info, normals_destaggered).astype(np.float32, copy=False)
                if scan.has_field("NORMALS"):
                    scan.del_field("NORMALS")
                scan.add_field("NORMALS", np.float32, (3,))[:] = normals_staggered

                if normals2_destaggered is not None:
                    normals2_staggered = stagger(info, normals2_destaggered).astype(np.float32, copy=False)
                    if scan.has_field("NORMALS2"):
                        scan.del_field("NORMALS2")
                    scan.add_field("NORMALS2", np.float32, (3,))[:] = normals2_staggered

            if not viz_modes_initialized:
                sviz._scan_viz.update(scans)
                sviz._scan_viz.select_cloud_mode("NORMALS")
                sviz._scan_viz.select_img_mode(0, "NORMALS")
                viz_modes_initialized = True

            yield scans

    # Visualize the point cloud colored by normals
    sviz.run(scans_with_normals())
