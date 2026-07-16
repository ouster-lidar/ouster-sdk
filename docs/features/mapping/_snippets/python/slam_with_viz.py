from ouster.sdk import open_source
from ouster.sdk.viz import SimpleViz
from ouster.sdk import mapping

source_file_path = "/PATH_TO_THE_FILE"
data_source = open_source(source_file_path, sensor_idx=-1)
config = mapping.SlamConfig.create("lio")
config.min_range = 1
config.max_range = 75
config.voxel_size = 1.0
slam = mapping.SlamEngine.create(data_source.sensor_info, config)

frames_w_poses = (slam.update(x) for x in data_source)

SimpleViz(data_source.sensor_info, accum_max_num=10).run(frames_w_poses)
