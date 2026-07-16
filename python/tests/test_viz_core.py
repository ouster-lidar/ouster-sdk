import numpy as np
import copy
from .conftest import MockPointViz
from ouster.sdk.core import (ChanField, LidarFrame, SensorInfo, LidarMode, Version, FieldClass, FrameSet,
                             Object, Pose, pose_at_timestamp)
from ouster.sdk.viz import WindowCtx, MouseButtonEvent, MouseButton, EventModifierKeys
from ouster.sdk.viz.model import (LidarFrameVizModel, SensorModel, Palettes, ObjectViewMode,
                                  _object_for_overlay_cuboid, _sync_object_pose_from_frame)
from ouster.sdk.viz.core import LidarFrameViz, _Seekable
from ouster.sdk.viz.view_mode import SimpleMode, RGBMode, HDRRGBMode
from ouster.sdk.viz.widgets import ToggleLabel


def _cuboid_matrix(obj: Object) -> np.ndarray:
    """4x4 model matrix produced by ``Cuboid.from_object()``."""
    object_in_world = obj.body_to_world * obj.object_to_body
    scale = np.diag([obj.dimensions[0], obj.dimensions[1], obj.dimensions[2], 1.0])
    return object_in_world.to_matrix() @ scale


def _overlay_cuboid_matrix(obj: Object, frame: LidarFrame) -> np.ndarray:
    return _cuboid_matrix(_object_for_overlay_cuboid(obj, frame))


def test_object_is_copyable():
    obj = Object()
    obj.id = 7
    obj.class_id = 3
    obj.timestamp = 123
    obj.velocity = np.array([1.0, 2.0, 3.0])
    obj.dimensions = np.array([4.0, 5.0, 6.0], dtype=np.float32)
    obj.object_to_body.position = np.array([0.1, 0.2, 0.3])
    obj.body_to_world.position = np.array([1.0, 2.0, 3.0])
    obj.properties['key'] = 'value'

    copied = copy.copy(obj)
    assert copied is not obj
    assert copied == obj

    copied.id = 99
    assert obj.id == 7


def test_enum_bitmask():
    """validate that the enum is a bitmask as expected"""
    EventModifierKeys(5)


def test_use_default_view_modes_1():
    """It will pick sensible default view modes depending on what's in the first frame."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    frame = LidarFrame(meta)
    model._amend_view_modes_all([frame])
    assert model._cloud_mode_name == ChanField.REFLECTIVITY
    assert model._image_mode_names[0] == ChanField.REFLECTIVITY
    assert model._image_mode_names[1] == ChanField.NEAR_IR


def test_use_default_view_modes_prefers_rgb_for_second_image():
    """It will prefer the RGB field for the second image when available."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    frame = LidarFrame(meta)
    frame.add_field("RGB", np.zeros((meta.h, meta.w, 3), dtype=np.uint8))

    model._amend_view_modes_all([frame])

    assert model._cloud_mode_name == ChanField.RGB
    assert model._image_mode_names[0] == ChanField.REFLECTIVITY
    assert model._image_mode_names[1] == ChanField.RGB


def test_use_default_view_modes_rgb_without_reflectivity():
    """It will use RGB for the second image even when reflectivity is unavailable."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    frame = LidarFrame(meta, [])
    frame.add_field("customfield", np.zeros((meta.h, meta.w), dtype=np.uint8))
    frame.add_field("RGB", np.zeros((meta.h, meta.w, 3), dtype=np.uint8))

    model._amend_view_modes_all([frame])

    assert model._cloud_mode_name == ChanField.RGB
    assert model._image_mode_names[0] == "customfield"
    assert model._image_mode_names[1] == ChanField.RGB


def test_use_default_view_modes_2():
    """It won't try to initialize default view modes until a frame with fields has been added."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    frame = LidarFrame(meta, [])
    model._amend_view_modes_all([frame])
    assert model._cloud_mode_name == ''
    assert model._image_mode_names == ['', '']


def test_use_default_view_modes_3():
    """It'll pick whatever's available if REFLECTIVITY/REFLECTIVITY2/NEAR_IR aren't present."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    frame = LidarFrame(meta, [])
    frame.add_field("customfield", np.ndarray((meta.h, meta.w), dtype=np.uint8))
    model._amend_view_modes_all([frame])
    assert model._cloud_mode_name == 'customfield'
    assert model._image_mode_names == ['customfield', 'customfield']


def test_create_view_mode_for_field_1():
    """Because the view mode depends on the frame dimensions,
    it should not add a view mode for a field that doesn't exist in the provided frame."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)
    frame = LidarFrame(meta, [])
    assert not frame.fields

    # FIXME use a dict for cloud and image modes, cycle through them by key
    assert sensor._create_view_mode_for_field(ChanField.RANGE, frame) is None


def test_create_view_mode_for_field_2():
    """It should not add a view mode for 2nd-return fields because these are
    handled by the view modes of the 1st-return fields. While this may sound weird,
    it's because the first and second return fields share autoexposure state."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)
    frame = LidarFrame(meta, [])
    assert sensor._create_view_mode_for_field(ChanField.FLAGS2, frame) is None
    assert sensor._create_view_mode_for_field(ChanField.RANGE2, frame) is None
    assert sensor._create_view_mode_for_field(ChanField.REFLECTIVITY2, frame) is None
    assert sensor._create_view_mode_for_field(ChanField.SIGNAL2, frame) is None


def test_create_view_mode_for_field_3():
    """It should add a SimpleMode with BeamUniformityCorrector for a NEAR_IR field."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)
    frame = LidarFrame(meta, [])
    assert not frame.fields
    frame.add_field(ChanField.NEAR_IR, np.ndarray((meta.h, meta.w), dtype=np.uint8))

    mode = sensor._create_view_mode_for_field(ChanField.NEAR_IR, frame)
    assert type(mode) is SimpleMode
    assert mode._ae is not None
    assert mode._buc is not None


def test_create_view_mode_for_field_4():
    """It should add an RGBMode for a field with a 3rd dimension of length 3."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)
    frame = LidarFrame(meta, [])
    assert not frame.fields
    frame.add_field("rgb", np.ndarray((meta.h, meta.w, 3), dtype=np.uint8))
    mode = sensor._create_view_mode_for_field("rgb", frame)
    assert type(mode) is RGBMode


def test_create_view_mode_for_field_5():
    """It should add a regular SimpleMode with AutoExposure but without
    BeamUniformityCorrector for a 2d field."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)
    frame = LidarFrame(meta, [])
    assert not frame.fields
    frame.add_field("customfield", np.ndarray((meta.h, meta.w), dtype=np.uint16))

    mode = sensor._create_view_mode_for_field("customfield", frame)
    assert type(mode) is SimpleMode
    assert mode._ae is not None
    assert mode._buc is None  # in contrast to NEAR_IR


def test_amend_view_modes_1():
    """It should add image and cloud view modes for the sensor
    for each new field name (provided as a list) and the given frame.
    """
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)

    # By default it has only the default cloud modes and no image modes.
    assert [m for m in sensor._cloud_modes] == ['RING', 'TIMESTAMP']
    assert sensor._image_modes == {}

    # Let's create a frame that originated from this sensor.
    frame = LidarFrame(meta)
    expected_frame_fields = [
        ChanField.FLAGS, ChanField.NEAR_IR, ChanField.RANGE, ChanField.REFLECTIVITY, ChanField.SIGNAL
    ]
    assert frame.fields == expected_frame_fields

    sensor._amend_view_modes(frame)

    expected_cloud_names = ['RING', 'TIMESTAMP', ChanField.NEAR_IR,
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
    given a list of frames.
    """
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)
    meta2 = SensorInfo.from_default(LidarMode._1024x10)
    sensor2 = SensorModel(viz, meta2)
    model = LidarFrameVizModel(viz, [meta, meta2], _img_aspect_ratio = 0)
    model._sensors = [sensor, sensor2]

    # Let's create a test frame
    frame = LidarFrame(meta)
    expected_frame_fields = [
        ChanField.FLAGS, ChanField.NEAR_IR, ChanField.RANGE, ChanField.REFLECTIVITY, ChanField.SIGNAL
    ]
    assert frame.fields == expected_frame_fields

    # Update the model with the supplied frame, one frame per sensor.
    model._amend_view_modes_all([frame, frame])

    expected_cloud_names = [ChanField.NEAR_IR,
        ChanField.RANGE, ChanField.REFLECTIVITY, 'RING', ChanField.SIGNAL, 'TIMESTAMP']
    expected_image_names = ['NEAR_IR', 'RANGE', 'RANGE2',
                            'REFLECTIVITY', 'REFLECTIVITY2', 'SIGNAL', 'SIGNAL2']

    # The new view modes should be present for both sensors.
    assert sorted([m for m in sensor._cloud_modes]) == expected_cloud_names
    assert sorted([m for m in sensor._image_modes]) == expected_image_names
    assert sorted([m for m in sensor2._cloud_modes]) == expected_cloud_names
    assert sorted([m for m in sensor2._image_modes]) == expected_image_names


def test_amend_view_modes_3():
    """It should add image and cloud view modes to a sensor
    even if a previous frame didn't have the necessary fields."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)
    meta2 = SensorInfo.from_default(LidarMode._1024x10)
    sensor2 = SensorModel(viz, meta2)
    model = LidarFrameVizModel(viz, [meta, meta2], _img_aspect_ratio = 0)
    model._sensors = [sensor, sensor2]

    # Let's create a test frame
    frame = LidarFrame(meta)
    frame2 = LidarFrame(meta)
    expected_frame_fields = [
        ChanField.FLAGS, ChanField.NEAR_IR, ChanField.RANGE, ChanField.REFLECTIVITY, ChanField.SIGNAL
    ]
    assert frame.fields == expected_frame_fields

    # Delete a field from the 2nd sensor's frame
    frame2.del_field(ChanField.RANGE)
    assert ChanField.RANGE not in frame2.fields

    # Update the model with the supplied frame, one frame per sensor.
    model._amend_view_modes_all([frame, frame2])

    # Observe that there is no view mode for the deleted sensor for the 2nd sensor.
    assert ChanField.RANGE in sensor._image_modes
    assert ChanField.RANGE not in sensor2._image_modes

    # Now update the model with the frame that has all fields
    model._amend_view_modes_all([frame, frame])

    # Now all view modes are present
    expected_cloud_names = [ChanField.NEAR_IR,
        ChanField.RANGE, ChanField.REFLECTIVITY, 'RING', ChanField.SIGNAL, 'TIMESTAMP']
    expected_image_names = [ChanField.NEAR_IR, ChanField.RANGE, ChanField.RANGE2,
        ChanField.REFLECTIVITY, ChanField.REFLECTIVITY2, ChanField.SIGNAL, ChanField.SIGNAL2]

    # The new view modes should be present for both sensors.
    assert sorted([m for m in sensor._cloud_modes]) == expected_cloud_names
    assert sorted([m for m in sensor._image_modes]) == expected_image_names
    assert sorted([m for m in sensor2._cloud_modes]) == expected_cloud_names
    assert sorted([m for m in sensor2._image_modes]) == expected_image_names


def test_palette_lengths():
    """Check that LidarFrameViz has matching palette lengths."""
    palettes = Palettes([])
    assert len(palettes._refl_cloud_palettes) == len(palettes._cloud_palettes)


def test_format_version():
    v = Version()
    v.major, v.minor, v.patch = 1, 2, 3
    assert LidarFrameViz._format_version(v) == "v1.2.3"

    v.major, v.minor, v.patch, v.prerelease = 1, 2, 3, 'alpha1'
    assert LidarFrameViz._format_version(v) == "v1.2.3-alpha1"


def test_seekable_next():
    """
    next_ind should increment when next() is called.
    It should be set to the seek position when seek() is called.
    """
    frame = LidarFrame(1, 1, [], 16)
    frame.timestamp[0] = 1
    frame.status[0] = 1
    assert frame.timestamp[frame.get_first_valid_column()] == 1
    frame2 = LidarFrame(1, 1, [], 16)
    frame2.timestamp[0] = 2

    def frame_generator():
        for i in range(100):
            yield (frame,)
            yield (frame2,)
            yield []  # signal a loop

    seekable = _Seekable(frame_generator(), 10)
    next(seekable)
    assert seekable.frame_num == 1
    next(seekable)
    assert seekable.frame_num == 2
    next(seekable)
    assert seekable.frame_num == 0  # wrap back around to the beginning

    # next_ind is affected by seek
    assert not seekable.seek(-1)
    assert seekable.seek(0)
    assert seekable.frame_num == 0
    next(seekable)
    assert seekable.frame_num == 1
    assert seekable.seek(2)
    assert seekable.frame_num == 2
    next(seekable)  # there is no next frame, so we'll wrap around to the beginning
    assert seekable.frame_num == 0

    # next after seek works as expected
    assert seekable.seek(0)
    assert (frame,) == next(seekable)
    assert (frame2,) == next(seekable)
    assert FrameSet() == next(seekable)
    assert (frame,) == next(seekable)


def test_seekable_next_2():
    """
    It evicts old frames when the buf size is exceeded.
    """
    frame = LidarFrame(1, 1, [], 16)
    frame2 = LidarFrame(1, 1, [], 16)
    buf_size = 10

    def frame_generator():
        for _ in range(buf_size):
            yield (frame,)
        yield (frame2,)

    seekable = _Seekable(frame_generator(), buf_size)

    for _ in range(buf_size):
        assert (frame,) == next(seekable)
    assert (frame2,) == next(seekable)
    assert not seekable.seek(0)  # first frame was evicted
    assert seekable.seek(1)      # second frame is still present


def test_lidar_frame_viz_update_sets_default_view_mode():
    """
    It should set a default view mode when update is called.
    """
    meta = SensorInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    assert ChanField.REFLECTIVITY in frame.fields  # a precondition
    viz = LidarFrameViz([meta], MockPointViz())
    viz.update([frame])
    assert viz._model._cloud_mode_name == ChanField.REFLECTIVITY


def test_lidar_frame_viz_cycle_img_mode_updates_images():
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

    meta = SensorInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    assert ChanField.REFLECTIVITY in frame.fields  # a precondition
    viz = LidarFrameViz([meta], MockPointViz())
    viz.update([frame])

    # precondition: the default image modes
    assert viz._model._image_mode_names == [ChanField.REFLECTIVITY, ChanField.NEAR_IR]

    viz._model._sensors[0]._images = [MockImage(), MockImage()]
    assert not viz._model._sensors[0]._images[0].set_image_called
    assert not viz._model._sensors[0]._images[1].set_image_called

    viz.cycle_img_mode(0)

    assert viz._model._image_mode_names == [ChanField.SIGNAL, ChanField.NEAR_IR]
    assert viz._model._sensors[0]._images[0].set_image_called
    assert viz._model._sensors[0]._images[1].set_image_called


def test_lidar_frame_viz_highlight_second_doesnt_crash_with_no_frame():
    """
    Highlighting the second return should not cause a crash if there is no frame yet.
    """
    meta = SensorInfo.from_default(LidarMode._1024x10)
    viz = LidarFrameViz([meta], MockPointViz())
    # check preconditions
    assert not viz._frame_set
    viz.update_flags_mode(LidarFrameViz.FlagsMode.HIGHLIGHT_SECOND)


def test_lidar_frame_viz_highlight_second_doesnt_crash_with_no_second_return():
    """
    Highlighting the second return should not cause a crash if there is no second return.
    """
    meta = SensorInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    # check preconditions
    assert ChanField.REFLECTIVITY in frame.fields
    assert ChanField.REFLECTIVITY2 not in frame.fields
    viz = LidarFrameViz([meta], MockPointViz())
    viz.update([frame])
    viz.update_flags_mode(LidarFrameViz.FlagsMode.HIGHLIGHT_SECOND)


def test_osd_state():
    """Hiding the help should reset the OSD to its previous state."""
    meta = SensorInfo.from_default(LidarMode._1024x10)
    viz = LidarFrameViz([meta], MockPointViz())
    assert viz.osd_state == LidarFrameViz.OsdState.DEFAULT
    viz.toggle_osd()
    assert viz.osd_state == LidarFrameViz.OsdState.NONE

    # if state is DEFAULT, toggling help twice resets to DEFAULT
    viz.toggle_osd()
    assert viz.osd_state == LidarFrameViz.OsdState.DEFAULT
    viz.toggle_help()
    assert viz.osd_state == LidarFrameViz.OsdState.HELP
    viz.toggle_help()
    assert viz.osd_state == LidarFrameViz.OsdState.DEFAULT

    # if state is NONE, toggling help twice resets to NONE
    viz.toggle_osd()
    assert viz.osd_state == LidarFrameViz.OsdState.NONE
    viz.toggle_help()
    assert viz.osd_state == LidarFrameViz.OsdState.HELP
    viz.toggle_help()
    assert viz.osd_state == LidarFrameViz.OsdState.NONE


def test_create_view_mode_for_field_6():
    """It should only create a view mode for pixel fields."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)

    frame = LidarFrame(meta, [])
    frame.add_field("customfield", np.zeros((1024, 1024)), FieldClass.COLUMN_FIELD)
    assert sensor._create_view_mode_for_field("customfield", frame) is None

    frame.add_field("customfield2", np.zeros((1024, 1024)), FieldClass.FRAME_FIELD)
    assert sensor._create_view_mode_for_field("customfield2", frame) is None


def test_create_view_mode_for_field_7():
    """It should add an HDRRGBMode for a 3-channel float16 RGB field."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    sensor = SensorModel(viz, meta)
    frame = LidarFrame(meta, [])
    assert not frame.fields
    frame.add_field("rgb", np.ndarray((meta.h, meta.w, 3), dtype=np.float16))
    mode = sensor._create_view_mode_for_field("rgb", frame)
    assert type(mode) is HDRRGBMode


def test_setup_sensor_toggle_keys():
    """It should only set up toggle keys for the keys 1 through 9."""

    # there's only one toggle key for a single sensor
    mockviz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    viz = LidarFrameViz([meta], MockPointViz())
    sensor_model_1 = [SensorModel(mockviz, meta)]
    key_bindings = {}
    viz._setup_sensor_toggle_keys(sensor_model_1, key_bindings)
    assert list(key_bindings.keys()) == [(ord('1'), 2)]

    # there's only MAX_SENSOR_TOGGLE_KEYS for > MAX_SENSOR_TOGGLE_KEYS sensors
    sensor_model_2 = [SensorModel(mockviz, meta) for _ in range(LidarFrameViz.MAX_SENSOR_TOGGLE_KEYS + 1)]
    viz = LidarFrameViz([meta] * (LidarFrameViz.MAX_SENSOR_TOGGLE_KEYS + 1), MockPointViz())
    viz._setup_sensor_toggle_keys(sensor_model_2, key_bindings)
    assert len(key_bindings.keys()) == LidarFrameViz.MAX_SENSOR_TOGGLE_KEYS

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

    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    frame = LidarFrame(meta)
    sensor = model._sensors[0]
    sensor.update_cloud = update_cloud_mock
    sensor.update_image = update_image_mock

    # sensor is disabled
    sensor._enabled = False

    assert not update_cloud_called and not update_image_called

    model.update([frame])

    assert not sensor._enabled and update_cloud_called and update_image_called


def test_viz_doesnt_crash_when_image_sizes_zero():
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    image = model._sensors[0]._images[0]
    image.set_position(0, 0, 0, 0)
    model._img_size_fraction = 0
    model.mouse_button_handler(
        WindowCtx(),
        MouseButton.MOUSE_BUTTON_RIGHT,
        MouseButtonEvent.MOUSE_BUTTON_PRESSED,
        EventModifierKeys.MOD_NONE
    )


def test_viz_doesnt_crash_when_frames_none():
    # OSDK-108: don't crash if no frames have been received yet
    # and some element (e.g. the OSD) is redrawn.
    meta = SensorInfo.from_default(LidarMode._1024x10)
    viz = LidarFrameViz([meta], MockPointViz())
    assert viz._frame_set == FrameSet()
    viz._draw_update_camera_pose()


def test_viz_imu_when_a_frame_is_none():
    # OSDK-300: don't crash if a frame is None
    meta = SensorInfo.from_default(LidarMode._1024x10)
    viz = LidarFrameViz([meta], MockPointViz())
    viz.imu_plot([[None]], viz._imu_viz_config)


def test_viz_safe_gif(tmp_path):
    from ouster.sdk.viz import SimpleViz
    """It should not crash when the duration between frames is negative or zero."""
    import PIL.Image as PILImage
    from unittest.mock import patch
    with patch('ouster.sdk.viz.core.add_default_controls'):
        test_image = PILImage.new('RGB', (10, 10))
        meta = SensorInfo.from_default(LidarMode._1024x10)
        viz = SimpleViz([meta], _override_pointviz=MockPointViz())
        viz._images = [(test_image, 100), (test_image, 50)]
        viz._save_gif(str(tmp_path / 'test.gif'))  # should not crash


def test_object_labels_follow_object_visibility():
    """Object labels should be visible only when objects are shown in the main 3D view."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)
    frame = LidarFrame(meta)
    frame_set = FrameSet([frame])

    obj = Object()
    obj.id = 42
    obj.class_id = 0
    obj.velocity = np.zeros(3)
    obj.object_to_body.position = np.zeros(3)
    obj.body_to_world.position = np.array([1.0, 2.0, 3.0])
    frame.objects['test'] = [obj]

    label = ToggleLabel(viz, "label", (1.0, 2.0, 3.0), initially_visible=True)
    model._object_labels[obj.id] = label

    for mode in (ObjectViewMode.OVERLAY_AND_MAIN_VIEW, ObjectViewMode.MAIN_VIEW_ONLY):
        model._object_view_mode = mode
        model.update_objects(frame_set)
        assert model._object_labels[obj.id].visible

    for mode in (ObjectViewMode.OVERLAY_ONLY, ObjectViewMode.NONE):
        model._object_view_mode = mode
        model.update_objects(frame_set)
        assert not model._object_labels[obj.id].visible


def test_object_selection_label_moves_with_object():
    """The selection label should follow the object when its position changes."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta], _img_aspect_ratio=0)

    def make_object(position):
        obj = Object()
        obj.id = 42
        obj.class_id = 0
        obj.velocity = np.zeros(3)
        obj.object_to_body.position = np.zeros(3)
        obj.body_to_world.position = position
        return obj

    frame1 = LidarFrame(meta)
    obj1 = make_object(np.array([1.0, 2.0, 3.0]))
    frame1.objects['test'] = [obj1]

    frame2 = LidarFrame(meta)
    obj2 = make_object(np.array([4.0, 5.0, 6.0]))
    frame2.objects['test'] = [obj2]

    label = ToggleLabel(viz, "label", (0.0, 0.0, 0.0), initially_visible=True)
    model._object_labels[obj1.id] = label

    model.update_objects(FrameSet([frame1]))
    expected = (obj1.body_to_world * obj1.object_to_body).position
    assert np.allclose(label.position, expected)

    model.update_objects(FrameSet([frame2]))
    expected = (obj2.body_to_world * obj2.object_to_body).position
    assert np.allclose(label.position, expected)


def test_sync_object_pose_from_frame_updates_body_to_world():
    meta = SensorInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    frame.status[:] = 1
    frame.timestamp[:] = np.array([100, 200, 300, 400] + [400] * (frame.w - 4), dtype=np.uint64)

    pose_before = np.eye(4)
    pose_before[0, 3] = 1.0
    pose_after = np.eye(4)
    pose_after[0, 3] = 5.0
    frame.body_to_world[0] = pose_before
    frame.body_to_world[1] = pose_before
    frame.body_to_world[2:] = pose_after

    obj = Object()
    obj.timestamp = 150
    obj.body_to_world = Pose()

    _sync_object_pose_from_frame(obj, frame)
    expected = pose_at_timestamp(frame, 150).to_matrix()
    assert np.allclose(obj.body_to_world.to_matrix(), expected)


def test_overlay_cuboid_returns_object_unchanged_when_timestamp_zero():
    """An object without a timestamp is used as-is for the overlay."""
    meta = SensorInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    frame.status[:] = 1
    ref_pose = np.eye(4)
    ref_pose[0, 3] = 7.0
    frame.body_to_world[:] = ref_pose

    obj = Object()
    obj.timestamp = 0
    obj.dimensions = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    obj.object_to_body.position = np.array([2.0, 0.0, 0.0])
    obj.body_to_world.position = np.array([99.0, 0.0, 0.0])

    assert _object_for_overlay_cuboid(obj, frame) is obj
    assert np.allclose(_overlay_cuboid_matrix(obj, frame), _cuboid_matrix(obj))


def test_overlay_cuboid_uses_first_valid_column_pose():
    """The overlay cuboid pose comes from the frame's first valid column pose,
    not the object's own body_to_world."""
    meta = SensorInfo.from_default(LidarMode._1024x10)
    frame = LidarFrame(meta)
    # make the first valid column something other than column 0 so we can tell
    # that the first *valid* column pose is the one being used
    frame.status[:] = 0
    frame.status[5:] = 1
    frame.body_to_world[:] = np.eye(4)
    first_valid_pose = np.eye(4)
    first_valid_pose[0, 3] = 7.0
    frame.body_to_world[5] = first_valid_pose

    obj = Object()
    obj.timestamp = 500
    obj.dimensions = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    obj.object_to_body.position = np.array([2.0, 0.0, 0.0])
    obj.body_to_world.position = np.array([99.0, 0.0, 0.0])  # ignored for the overlay

    overlay_obj = _object_for_overlay_cuboid(obj, frame)
    assert np.allclose(overlay_obj.body_to_world.to_matrix(), first_valid_pose)

    expected = first_valid_pose @ obj.object_to_body.to_matrix()
    assert np.allclose(_overlay_cuboid_matrix(obj, frame), expected)
    # the object's own body_to_world is not used for the overlay
    assert not np.allclose(_overlay_cuboid_matrix(obj, frame), _cuboid_matrix(obj))


def test_frameset_objects_use_matching_sensor_frame_for_overlay():
    """Each sensor's overlay uses that sensor's own frame pose."""
    viz = MockPointViz()
    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(viz, [meta, meta], _img_aspect_ratio=0)

    def make_frame(x_offset: float) -> LidarFrame:
        frame = LidarFrame(meta)
        frame.status[:] = 1
        pose = np.eye(4)
        pose[0, 3] = x_offset
        frame.body_to_world[:] = pose
        return frame

    frame0 = make_frame(0.0)
    frame1 = make_frame(10.0)
    frame_set = FrameSet([frame0, frame1])

    obj = Object()
    obj.id = 7
    obj.timestamp = 500
    obj.dimensions = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    obj.object_to_body.position = np.array([1.0, 0.0, 0.0])
    frame_set.objects['tracks'] = [obj]

    model._object_view_mode = ObjectViewMode.OVERLAY_ONLY
    model.update_objects(frame_set)

    assert len(model._objects) == 1
    overlay_obj = next(iter(model._objects.keys()))

    assert not np.allclose(
        _overlay_cuboid_matrix(overlay_obj, frame0),
        _overlay_cuboid_matrix(overlay_obj, frame1))


def test_overlay_pose_same_for_frameset_and_lidarframe_objects():
    """Overlay pose should not depend on whether the object is attached to the
    FrameSet or to an individual LidarFrame."""
    meta = SensorInfo.from_default(LidarMode._1024x10)

    def make_frame(x_offset: float) -> LidarFrame:
        frame = LidarFrame(meta)
        frame.status[:] = 1
        pose = np.eye(4)
        pose[0, 3] = x_offset
        frame.body_to_world[:] = pose
        return frame

    def make_object() -> Object:
        obj = Object()
        obj.id = 42
        obj.timestamp = 500
        obj.dimensions = np.array([2.0, 1.0, 3.0], dtype=np.float32)
        obj.object_to_body.position = np.array([1.0, 0.5, -0.25])
        obj.body_to_world.position = np.array([99.0, 0.0, 0.0])
        return obj

    def overlay_transform(frame_set: FrameSet, sensor_idx: int) -> np.ndarray:
        model = LidarFrameVizModel(MockPointViz(), [meta, meta], _img_aspect_ratio=0)  # type: ignore
        model._object_view_mode = ObjectViewMode.OVERLAY_ONLY
        model.update_objects(frame_set)
        overlay_obj = next(iter(model._objects.keys()))
        frame = frame_set[sensor_idx]
        assert frame is not None
        return _overlay_cuboid_matrix(overlay_obj, frame)

    for sensor_idx in (0, 1):
        frameset_scoped = FrameSet([make_frame(0.0), make_frame(5.0)])
        frameset_scoped.objects['tracks'] = [make_object()]

        frame_scoped = FrameSet([make_frame(0.0), make_frame(5.0)])
        frame_scoped[sensor_idx].objects['tracks'] = [make_object()]

        assert np.allclose(
            overlay_transform(frameset_scoped, sensor_idx),
            overlay_transform(frame_scoped, sensor_idx))


def test_update_object_cuboids_does_not_copy_main_view_cuboids(monkeypatch):
    """Overlay cuboids must be built separately so main-view pick callbacks stay registered."""
    import copy

    def fail_copy(_obj):
        raise AssertionError(
            "update_object_cuboids must not copy main-view cuboids; copies share "
            "select callbacks and unregister them on destruction")

    monkeypatch.setattr(copy, "copy", fail_copy)

    meta = SensorInfo.from_default(LidarMode._1024x10)
    model = LidarFrameVizModel(MockPointViz(), [meta], _img_aspect_ratio=0)

    frame = LidarFrame(meta)
    obj = Object()
    obj.id = 42
    obj.class_id = 0
    obj.velocity = np.zeros(3)
    obj.object_to_body.position = np.zeros(3)
    obj.body_to_world.position = np.array([1.0, 2.0, 3.0])
    frame.objects['test'] = [obj]

    model._object_view_mode = ObjectViewMode.OVERLAY_AND_MAIN_VIEW
    model.update_objects(FrameSet([frame]))


def test_cuboid_copy_does_not_copy_selection_state():
    """Cuboid copies used for overlay rendering should not inherit pick callbacks."""
    import copy
    import gc
    from ouster.sdk._bindings.viz import Cuboid

    obj = Object()
    obj.dimensions = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    obj.object_to_body.position = np.zeros(3)
    obj.body_to_world.position = np.zeros(3)

    cuboid = Cuboid.from_object(obj, (1.0, 0.0, 0.0, 0.3))
    cuboid.set_on_select(lambda _: None)

    overlay_cuboid = copy.copy(cuboid)
    assert cuboid.selectable
    assert not overlay_cuboid.selectable

    del overlay_cuboid
    gc.collect()
    assert cuboid.selectable
