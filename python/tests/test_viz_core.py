import numpy as np
from .conftest import MockPointViz
from ouster.sdk.core import ChanField, LidarScan, SensorInfo, LidarMode, Version, FieldClass
from ouster.sdk.viz import WindowCtx, MouseButtonEvent, MouseButton, EventModifierKeys
from ouster.sdk.viz.model import LidarScanVizModel, SensorModel, Palettes
from ouster.sdk.viz.core import LidarScanViz, _Seekable
from ouster.sdk.viz.view_mode import SimpleMode, RGBMode


def test_use_default_view_modes_1():
    """It will pick sensible default view modes depending on what's in the first scan."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    model = LidarScanVizModel([meta], _img_aspect_ratio=0)
    scan = LidarScan(meta.h, meta.w)
    model._amend_view_modes_all([scan])
    assert model._cloud_mode_name == ChanField.REFLECTIVITY
    assert model._image_mode_names[0] == ChanField.REFLECTIVITY
    assert model._image_mode_names[1] == ChanField.NEAR_IR


def test_use_default_view_modes_2():
    """It won't try to initialize default view modes until a scan with fields has been added."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    model = LidarScanVizModel([meta], _img_aspect_ratio=0)
    scan = LidarScan(meta.h, meta.w, [])
    model._amend_view_modes_all([scan])
    assert model._cloud_mode_name == ''
    assert model._image_mode_names == ['', '']


def test_use_default_view_modes_3():
    """It'll pick whatever's available if REFLECTIVITY/REFLECTIVITY2/NEAR_IR aren't present."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    model = LidarScanVizModel([meta], _img_aspect_ratio=0)
    scan = LidarScan(meta.h, meta.w, [])
    scan.add_field("customfield", np.ndarray((meta.h, meta.w), dtype=np.uint8))
    model._amend_view_modes_all([scan])
    assert model._cloud_mode_name == 'customfield'
    assert model._image_mode_names == ['customfield', 'customfield']


def test_create_view_mode_for_field_1():
    """Because the view mode depends on the scan dimensions,
    it should not add a view mode for a field that doesn't exist in the provided scan."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)
    scan = LidarScan(meta.h, meta.w, [])
    assert not scan.fields

    # FIXME use a dict for cloud and image modes, cycle through them by key
    assert sensor._create_view_mode_for_field(ChanField.RANGE, scan) is None


def test_create_view_mode_for_field_2():
    """It should not add a view mode for 2nd-return fields because these are
    handled by the view modes of the 1st-return fields. While this may sound weird,
    it's because the first and second return fields share autoexposure state."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)
    scan = LidarScan(meta.h, meta.w, [])
    assert sensor._create_view_mode_for_field(ChanField.FLAGS2, scan) is None
    assert sensor._create_view_mode_for_field(ChanField.RANGE2, scan) is None
    assert sensor._create_view_mode_for_field(ChanField.REFLECTIVITY2, scan) is None
    assert sensor._create_view_mode_for_field(ChanField.SIGNAL2, scan) is None


def test_create_view_mode_for_field_3():
    """It should add a SimpleMode with BeamUniformityCorrector for a NEAR_IR field."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)
    scan = LidarScan(meta.h, meta.w, [])
    assert not scan.fields
    scan.add_field(ChanField.NEAR_IR, np.ndarray((meta.h, meta.w), dtype=np.uint8))

    mode = sensor._create_view_mode_for_field(ChanField.NEAR_IR, scan)
    assert type(mode) is SimpleMode
    assert mode._ae is not None
    assert mode._buc is not None


def test_create_view_mode_for_field_4():
    """It should add an RGBMode for a field with a 3rd dimension of length 3."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)
    scan = LidarScan(meta.h, meta.w, [])
    assert not scan.fields
    scan.add_field("rgb", np.ndarray((meta.h, meta.w, 3), dtype=np.uint8))
    mode = sensor._create_view_mode_for_field("rgb", scan)
    assert type(mode) is RGBMode


def test_create_view_mode_for_field_5():
    """It should add a regular SimpleMode with AutoExposure but without
    BeamUniformityCorrector for a 2d field."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)
    scan = LidarScan(meta.h, meta.w, [])
    assert not scan.fields
    scan.add_field("customfield", np.ndarray((meta.h, meta.w), dtype=np.uint16))

    mode = sensor._create_view_mode_for_field("customfield", scan)
    assert type(mode) is SimpleMode
    assert mode._ae is not None
    assert mode._buc is None  # in contrast to NEAR_IR


def test_amend_view_modes_1():
    """It should add image and cloud view modes for the sensor
    for each new field name (provided as a list) and the given scan.
    """
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)

    # By default it has no cloud or image modes.
    assert [m for m in sensor._cloud_modes] == ['RING']
    assert sensor._image_modes == {}

    # Let's create a scan that originated from this sensor.
    scan = LidarScan(meta.h, meta.w)
    expected_scan_fields = [
        ChanField.FLAGS, ChanField.NEAR_IR, ChanField.RANGE, ChanField.REFLECTIVITY, ChanField.SIGNAL
    ]
    assert scan.fields == expected_scan_fields

    sensor._amend_view_modes(scan)

    expected_cloud_names = ['RING', ChanField.NEAR_IR,
        ChanField.RANGE, ChanField.REFLECTIVITY, ChanField.SIGNAL]
    expected_image_names = ['NEAR_IR', 'RANGE', 'RANGE2',
                            'REFLECTIVITY', 'REFLECTIVITY2', 'SIGNAL', 'SIGNAL2']
    assert [m for m in sensor._cloud_modes] == expected_cloud_names
    assert [m for m in sensor._image_modes] == expected_image_names

    # ensure proper construction of dual-return image modes
    # which are wrapped with an ImgModeItem to share state between the related fields
    for field1, field2 in [
        (ChanField.RANGE, ChanField.RANGE2),
        (ChanField.REFLECTIVITY, ChanField.REFLECTIVITY2),
        (ChanField.SIGNAL, ChanField.SIGNAL2)
    ]:
        mode1 = sensor._image_modes[field1]
        mode2 = sensor._image_modes[field2]
        assert mode1.name == field1
        assert mode2.name == field2
        assert mode1.mode.names == [field1, field2]
        assert mode2.mode.names == [field1, field2]
        assert mode1.return_num == 0
        assert mode2.return_num == 1


def test_amend_view_modes_2():
    """It should add image and cloud view modes for all sensors
    given a list of scans.
    """
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)
    meta2 = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor2 = SensorModel(meta2)
    model = LidarScanVizModel([meta, meta2], _img_aspect_ratio = 0)
    model._sensors = [sensor, sensor2]

    # Let's create a test scan
    scan = LidarScan(meta.h, meta.w)
    expected_scan_fields = [
        ChanField.FLAGS, ChanField.NEAR_IR, ChanField.RANGE, ChanField.REFLECTIVITY, ChanField.SIGNAL
    ]
    assert scan.fields == expected_scan_fields

    # Update the model with the supplied scan, one scan per sensor.
    model._amend_view_modes_all([scan, scan])

    expected_cloud_names = [ChanField.NEAR_IR,
        ChanField.RANGE, ChanField.REFLECTIVITY, 'RING', ChanField.SIGNAL]
    expected_image_names = ['NEAR_IR', 'RANGE', 'RANGE2',
                            'REFLECTIVITY', 'REFLECTIVITY2', 'SIGNAL', 'SIGNAL2']

    # The new view modes should be present for both sensors.
    assert sorted([m for m in sensor._cloud_modes]) == expected_cloud_names
    assert sorted([m for m in sensor._image_modes]) == expected_image_names
    assert sorted([m for m in sensor2._cloud_modes]) == expected_cloud_names
    assert sorted([m for m in sensor2._image_modes]) == expected_image_names


def test_amend_view_modes_3():
    """It should add image and cloud view modes to a sensor
    even if a previous scan didn't have the necessary fields."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)
    meta2 = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor2 = SensorModel(meta2)
    model = LidarScanVizModel([meta, meta2], _img_aspect_ratio = 0)
    model._sensors = [sensor, sensor2]

    # Let's create a test scan
    scan = LidarScan(meta.h, meta.w)
    scan2 = LidarScan(meta.h, meta.w)
    expected_scan_fields = [
        ChanField.FLAGS, ChanField.NEAR_IR, ChanField.RANGE, ChanField.REFLECTIVITY, ChanField.SIGNAL
    ]
    assert scan.fields == expected_scan_fields

    # Delete a field from the 2nd sensor's scan
    scan2.del_field(ChanField.RANGE)
    assert ChanField.RANGE not in scan2.fields

    # Update the model with the supplied scan, one scan per sensor.
    model._amend_view_modes_all([scan, scan2])

    # Observe that there is no view mode for the deleted sensor for the 2nd sensor.
    assert ChanField.RANGE in sensor._image_modes
    assert ChanField.RANGE not in sensor2._image_modes

    # Now update the model with the scan that has all fields
    model._amend_view_modes_all([scan, scan])

    # Now all view modes are present
    expected_cloud_names = [ChanField.NEAR_IR,
        ChanField.RANGE, ChanField.REFLECTIVITY, 'RING', ChanField.SIGNAL]
    expected_image_names = [ChanField.NEAR_IR, ChanField.RANGE, ChanField.RANGE2,
        ChanField.REFLECTIVITY, ChanField.REFLECTIVITY2, ChanField.SIGNAL, ChanField.SIGNAL2]

    # The new view modes should be present for both sensors.
    assert sorted([m for m in sensor._cloud_modes]) == expected_cloud_names
    assert sorted([m for m in sensor._image_modes]) == expected_image_names
    assert sorted([m for m in sensor2._cloud_modes]) == expected_cloud_names
    assert sorted([m for m in sensor2._image_modes]) == expected_image_names


def test_palette_lengths():
    """Check that LidarScanViz has matching palette lengths."""
    palettes = Palettes([])
    assert len(palettes._refl_cloud_palettes) == len(palettes._cloud_palettes)


def test_format_version():
    v = Version()
    v.major, v.minor, v.patch = 1, 2, 3
    assert LidarScanViz._format_version(v) == "v1.2.3"

    v.major, v.minor, v.patch, v.prerelease = 1, 2, 3, 'alpha1'
    assert LidarScanViz._format_version(v) == "v1.2.3-alpha1"


def test_seekable_next():
    """
    next_ind should increment when next() is called.
    It should be set to the seek position when seek() is called.
    """
    scan = LidarScan(1, 1)
    scan.timestamp[0] = 1
    scan.status[0] = 1
    assert scan.get_first_valid_column_timestamp() == 1
    scan2 = LidarScan(1, 1)
    scan2.timestamp[0] = 2

    def scan_generator():
        for i in range(100):
            yield (scan,)
            yield (scan2,)
            yield []  # signal a loop

    seekable = _Seekable(scan_generator(), 10)
    next(seekable)
    assert seekable.scan_num == 1
    next(seekable)
    assert seekable.scan_num == 2
    next(seekable)
    assert seekable.scan_num == 0  # wrap back around to the beginning

    # next_ind is affected by seek
    assert not seekable.seek(-1)
    assert seekable.seek(0)
    assert seekable.scan_num == 0
    next(seekable)
    assert seekable.scan_num == 1
    assert seekable.seek(2)
    assert seekable.scan_num == 2
    next(seekable)  # there is no next scan, so we'll wrap around to the beginning
    assert seekable.scan_num == 0

    # next after seek works as expected
    assert seekable.seek(0)
    assert (scan,) == next(seekable)
    assert (scan2,) == next(seekable)
    assert [] == next(seekable)
    assert (scan,) == next(seekable)


def test_seekable_next_2():
    """
    It evicts old scans when the buf size is exceeded.
    """
    scan = LidarScan(1, 1)
    scan2 = LidarScan(1, 1)
    buf_size = 10

    def scan_generator():
        for _ in range(buf_size):
            yield (scan,)
        yield (scan2,)

    seekable = _Seekable(scan_generator(), buf_size)

    for _ in range(buf_size):
        assert (scan,) == next(seekable)
    assert (scan2,) == next(seekable)
    assert not seekable.seek(0)  # first scan was evicted
    assert seekable.seek(1)      # second scan is still present


def test_lidarscanviz_update_sets_default_view_mode():
    """
    It should set a default view mode when update is called.
    """
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    scan = LidarScan(meta)
    assert ChanField.REFLECTIVITY in scan.fields  # a precondition
    viz = LidarScanViz([meta], MockPointViz())
    viz.update([scan])
    assert viz._model._cloud_mode_name == ChanField.REFLECTIVITY


def test_lidarscanviz_cycle_img_mode_updates_images():
    """
    When paused, cycling the img mode should update the images.
    """
    class MockImage:
        """
        TWS 20241007: this kind of mock would be unnecessary
        if we bind the various properties of the viz::Image class.
        """
        def __init__(self):
            self.set_image_called = False

        def clear_palette(self, *args, **kwargs):
            pass

        def set_image(self, *args, **kwargs):
            self.set_image_called = True

    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    scan = LidarScan(meta)
    assert ChanField.REFLECTIVITY in scan.fields  # a precondition
    viz = LidarScanViz([meta], MockPointViz())
    viz.update([scan])

    # precondition: the default image modes
    assert viz._model._image_mode_names == [ChanField.REFLECTIVITY, ChanField.NEAR_IR]

    viz._model._sensors[0]._images = [MockImage(), MockImage()]
    assert not viz._model._sensors[0]._images[0].set_image_called
    assert not viz._model._sensors[0]._images[1].set_image_called

    viz.cycle_img_mode(0)

    assert viz._model._image_mode_names == [ChanField.SIGNAL, ChanField.NEAR_IR]
    assert viz._model._sensors[0]._images[0].set_image_called
    assert viz._model._sensors[0]._images[1].set_image_called


def test_lidarscanviz_highlight_second_doesnt_crash_with_no_scan():
    """
    Highlighting the second return should not cause a crash if there is no scan yet.
    """
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    viz = LidarScanViz([meta], MockPointViz())
    # check preconditions
    assert not viz._scans
    viz.update_flags_mode(LidarScanViz.FlagsMode.HIGHLIGHT_SECOND)


def test_lidarscanviz_highlight_second_doesnt_crash_with_no_second_return():
    """
    Highlighting the second return should not cause a crash if there is no second return.
    """
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    scan = LidarScan(meta)
    # check preconditions
    assert ChanField.REFLECTIVITY in scan.fields
    assert ChanField.REFLECTIVITY2 not in scan.fields
    viz = LidarScanViz([meta], MockPointViz())
    viz.update([scan])
    viz.update_flags_mode(LidarScanViz.FlagsMode.HIGHLIGHT_SECOND)


def test_osd_state():
    """Hiding the help should reset the OSD to its previous state."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    viz = LidarScanViz([meta], MockPointViz())
    assert viz.osd_state == LidarScanViz.OsdState.DEFAULT
    viz.toggle_osd()
    assert viz.osd_state == LidarScanViz.OsdState.NONE

    # if state is DEFAULT, toggling help twice resets to DEFAULT
    viz.toggle_osd()
    assert viz.osd_state == LidarScanViz.OsdState.DEFAULT
    viz.toggle_help()
    assert viz.osd_state == LidarScanViz.OsdState.HELP
    viz.toggle_help()
    assert viz.osd_state == LidarScanViz.OsdState.DEFAULT

    # if state is NONE, toggling help twice resets to NONE
    viz.toggle_osd()
    assert viz.osd_state == LidarScanViz.OsdState.NONE
    viz.toggle_help()
    assert viz.osd_state == LidarScanViz.OsdState.HELP
    viz.toggle_help()
    assert viz.osd_state == LidarScanViz.OsdState.NONE


def test_create_view_mode_for_field_6():
    """It should only create a view mode for pixel fields."""
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor = SensorModel(meta)

    scan = LidarScan(meta.h, meta.w, [])
    scan.add_field("customfield", np.zeros((1024, 1024)), FieldClass.COLUMN_FIELD)
    assert sensor._create_view_mode_for_field("customfield", scan) is None

    scan.add_field("customfield2", np.zeros((1024, 1024)), FieldClass.SCAN_FIELD)
    assert sensor._create_view_mode_for_field("customfield2", scan) is None


def test_setup_sensor_toggle_keys():
    """It should only set up toggle keys for the keys 1 through 9."""

    # there's only one toggle key for a single sensor
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    sensor_model_1 = [SensorModel(meta)]
    viz = LidarScanViz([meta], MockPointViz())
    key_bindings = {}
    viz._setup_sensor_toggle_keys(sensor_model_1, key_bindings)
    assert list(key_bindings.keys()) == [(ord('1'), 2)]

    # there's only MAX_SENSOR_TOGGLE_KEYS for > MAX_SENSOR_TOGGLE_KEYS sensors
    sensor_model_2 = [SensorModel(meta) for _ in range(LidarScanViz.MAX_SENSOR_TOGGLE_KEYS + 1)]
    viz = LidarScanViz([meta] * (LidarScanViz.MAX_SENSOR_TOGGLE_KEYS + 1), MockPointViz())
    viz._setup_sensor_toggle_keys(sensor_model_2, key_bindings)
    assert len(key_bindings.keys()) == LidarScanViz.MAX_SENSOR_TOGGLE_KEYS

    for i, key in enumerate(key_bindings.keys()):
        assert key == (ord('1') + i, 2)


def test_update_model_even_for_sensors_not_enabled():
    """Clouds and images should be updated even when the sensor isn't enabled."""

    # TODO: yet another example of how it should be possible to access Cloud and Image attrs
    update_cloud_called = False
    update_image_called = False

    def update_cloud_mock(*args, **kwargs):
        nonlocal update_cloud_called
        update_cloud_called = True

    def update_image_mock(*args, **kwargs):
        nonlocal update_image_called
        update_image_called = True

    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    model = LidarScanVizModel([meta], _img_aspect_ratio=0)
    scan = LidarScan(meta.h, meta.w)
    sensor = model._sensors[0]
    sensor.update_cloud = update_cloud_mock
    sensor.update_image = update_image_mock

    # sensor is disabled
    sensor._enabled = False

    assert not update_cloud_called and not update_image_called

    model.update([scan])

    assert not sensor._enabled and update_cloud_called and update_image_called


def test_viz_doesnt_crash_when_image_sizes_zero():
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    model = LidarScanVizModel([meta], _img_aspect_ratio=0)
    image = model._sensors[0]._images[0]
    image.set_position(0, 0, 0, 0)
    model._img_size_fraction = 0
    model.mouse_button_handler(
        WindowCtx(),
        MouseButton.MOUSE_BUTTON_RIGHT,
        MouseButtonEvent.MOUSE_BUTTON_PRESSED,
        EventModifierKeys.MOD_NONE
    )


def test_viz_doesnt_crash_when_scans_none():
    # OSDK-108: don't crash if no scans have been received yet
    # and some element (e.g. the OSD) is redrawn.
    meta = SensorInfo.from_default(LidarMode.MODE_1024x10)
    viz = LidarScanViz([meta], MockPointViz())
    assert viz._scans == []
    viz._draw_update_camera_pose()
