import argparse
import datetime
import logging
import math
import os

import numpy as np

import ouster.sdk.client as client
import ouster.sdk.osf as osf
import ouster.sdk.util.pose_util as pu

try:
    import yaml  # type: ignore
    _no_yaml = False
except ImportError:
    # PyYAML is needed for this internal tool. It rely on internal users
    # manually install
    _no_yaml = True

try:
    import matplotlib.pyplot as plt  # type: ignore
    _no_plotlib = False
except ImportError:
    # matplotlib is needed for this internal tool. It rely on internal users
    # manually install
    _no_plotlib = True

logger = logging.getLogger()
logging.basicConfig(level=logging.INFO)

truth_TrajEval = None
cmpar_TrajEval = None
start_ts = None
end_ts = None
s_to_ns = 1e9
# valid if ls cnt bewteen two osf is less than 5%
valid_ls_cnt_diff = 0.05


def cal_dist(point):
    return math.sqrt(point[3]**2 + point[4]**2 + point[5]**2)


def cal_diff_percnt(diff_part, dist_part):
    return (diff_part / dist_part) * 100


def loadOSFPoses(ground_truth, comparison, out_yaml):
    """
    Load trajectory from OSF files and get overal drift info

    """
    def process_osf_messages(osf_scans, trajectory_poses):
        osf_start_ts = None
        osf_end_ts = None
        scan_cnt = 0
        for scan in osf_scans:
            start_col = client.core.first_valid_column(scan)
            end_col = client.core.last_valid_column(scan)
            start_ts = scan.timestamp[start_col]
            end_ts = scan.timestamp[end_col]
            if not osf_start_ts:
                osf_start_ts = start_ts
            osf_end_ts = end_ts
            trajectory_poses.append((start_ts, scan.pose[start_col].copy()))
            trajectory_poses.append((end_ts, scan.pose[end_col].copy()))
            scan_cnt += 1

        return {
                'ls_count': scan_cnt,
                'osf_start_ts': osf_start_ts,
                'osf_end_ts': osf_end_ts}

    truth_TrajPoses = []
    cmpar_TrajPoses = []

    global truth_TrajEval, cmpar_TrajEval, start_ts, end_ts

    truth_osf_scans = osf.Scans(ground_truth)

    truth_results = process_osf_messages(truth_osf_scans, truth_TrajPoses)

    # exit if too short
    if truth_results['ls_count'] < 15:
        logger.error("Too small. No need to compare. Nothing run and Exit")
        exit(1)
    truth_TrajEval = pu.TrajectoryEvaluator(truth_TrajPoses)

    cmpar_osf_scans = osf.Scans(comparison)
    cmpar_results = process_osf_messages(cmpar_osf_scans, cmpar_TrajPoses)
    cmpar_TrajEval = pu.TrajectoryEvaluator(cmpar_TrajPoses)

    start_ts = max(truth_results['osf_start_ts'], cmpar_results['osf_start_ts'])
    end_ts = min(truth_results['osf_end_ts'], cmpar_results['osf_end_ts'])

    # grouth truth / to compare
    result_numb_ratio = truth_results['ls_count'] / cmpar_results['ls_count']
    if result_numb_ratio < (1 - valid_ls_cnt_diff) or result_numb_ratio > (1 + valid_ls_cnt_diff):
        logger.error(f"Two OSF files' lidarscan obj are more than {valid_ls_cnt_diff*100} %")
        exit(1)

    pose_diff = pu.log_pose(
        np.matmul(
            np.linalg.inv(
                truth_TrajEval.pose_at(
                    end_ts - 1)),
            cmpar_TrajEval.pose_at(
                    end_ts - 1)))

    # Save 3 decimal. Pose [0:2] is angular and [3-5] is translation
    duration = round((end_ts - start_ts) / s_to_ns, 3)
    pose_diff_wx = round(pose_diff[0], 3)
    pose_diff_wy = round(pose_diff[1], 3)
    pose_diff_wz = round(pose_diff[2], 3)
    pose_diff_x = round(pose_diff[3], 3)
    pose_diff_y = round(pose_diff[4], 3)
    pose_diff_z = round(pose_diff[5], 3)
    pose_diff_dist = round(cal_dist(pose_diff), 3)
    logger.info(f"Duration {duration} second")
    logger.info(
            "Final pose angular diff (rad): \n"
            f"wx({pose_diff_wx}),  wy({pose_diff_wy}),  wz({pose_diff_wz})")
    logger.info(
            "Final pose translation diff (m): \n"
            f"x({pose_diff_x}),  y({pose_diff_y}),  z({pose_diff_z})")
    logger.info(f"Pose diff dist : {pose_diff_dist} meter")

    if out_yaml:
        data = {
            "Global Comparison": {
                "Duration (s)": f"{duration}",
                "Final pose angular diff (rad)": {
                    "wx": f"{pose_diff_wx}",
                    "wy": f"{pose_diff_wy}",
                    "wz": f"{pose_diff_wz}"
                },
                "Final pose translation diff (m)": {
                    "x": f"{pose_diff_x}",
                    "y": f"{pose_diff_y}",
                    "z": f"{pose_diff_z}"
                },
                "Final Pose Diff Dist (m)": f"{pose_diff_dist}"
            }
        }
        with open(out_yaml, 'a') as outfile:
            yaml.dump(data, outfile)


def compare(out_yaml, ts_incrmt, print_incrmt):
    """
    Compare the final pose between two input in orientation and translation
    Compare two trajectories by small steps (ts_incrmt) and printout results in each
    small period in translation only
    See terminal printout and saved yaml file for more details

    """
    ns_ts_incrmt = int(ts_incrmt * s_to_ns)
    array_shape = (6,)
    zero_array = np.zeros(array_shape)
    sum_diff_pose = zero_array
    sum_pose = zero_array
    sum_diff_pose_part = zero_array
    sum_pose_part = zero_array
    last_truth = truth_TrajEval.pose_at(start_ts + 1)
    part_start_percnt = 0
    part_end_percnt = print_incrmt
    dur_ts = end_ts - start_ts
    total_count = 0
    partial_count = 0

    if out_yaml:
        data = {"Partially Comparison": {}}

    for ts in range(start_ts, end_ts, ns_ts_incrmt):
        total_count += 1
        partial_count += 1
        truth_pose = truth_TrajEval.pose_at(ts)
        cmpar_pose = cmpar_TrajEval.pose_at(ts)

        pose_diff = pu.log_pose(np.matmul(np.linalg.inv(truth_pose), cmpar_pose))
        sum_diff_pose = np.add(sum_diff_pose, np.abs(pose_diff))
        sum_diff_pose_part = np.add(sum_diff_pose_part, np.abs(pose_diff))

        pose_travelled = pu.log_pose(np.matmul(np.linalg.inv(last_truth), truth_pose))
        sum_pose = np.add(sum_pose, np.abs(pose_travelled))
        sum_pose_part = np.add(sum_pose_part, np.abs(pose_travelled))

        last_truth = truth_pose
        if ts >= part_end_percnt * dur_ts + start_ts:
            dist_part = cal_dist(sum_pose_part)
            diff_part = cal_dist(sum_diff_pose_part) / partial_count
            diff_part_percnt = cal_diff_percnt(diff_part, dist_part)

            key = f"{part_start_percnt * 100:.1f}% - {part_end_percnt * 100:.1f}%"
            value = (
                f"travelled {dist_part:.3f} m and average diff"
                f" {diff_part:.3f} m. Around {diff_part_percnt:.3f}% off"
            )
            logger.info(f"{key}: {value}")
            if out_yaml:
                data["Partially Comparison"][key] = value

            part_start_percnt += print_incrmt
            part_end_percnt += print_incrmt
            sum_diff_pose_part = zero_array
            sum_pose_part = zero_array
            partial_count = 0

    # Handle the last part. In default param, it handles 80% - 100% part
    dist_part = cal_dist(sum_pose_part)
    diff_part = cal_dist(sum_diff_pose_part) / partial_count
    diff_part_percnt = cal_diff_percnt(diff_part, dist_part)

    key = f"{part_start_percnt * 100:.1f}% - {part_end_percnt * 100:.1f}%"
    value = (
        f"travelled {dist_part:.3f} m and average diff"
        f" {diff_part:.3f} m. Around {diff_part_percnt:.3f}% off"
    )
    logger.info(f"{key}: {value}")
    if out_yaml:
        data["Partially Comparison"][key] = value

    # Total travelled and diff
    dist_total = cal_dist(sum_pose)
    diff_total = cal_dist(sum_diff_pose) / total_count
    diff_total_percnt = cal_diff_percnt(diff_total, dist_total)

    logger.info(f"Total travelled {dist_total:.3f} m and average diff {diff_total:.3f} m. "
                f"Off around {diff_total_percnt:.3f}%")

    if out_yaml:
        key = "Overall Stat"
        sub_key1 = "Travelled (m)"
        sub_key2 = "Avg_diff (m)"
        sub_key3 = "Off_percnt (%)"
        value1 = f"{dist_total:.3f}"
        value2 = f"{diff_total:.3f}"
        value3 = f"{diff_part_percnt:.3f}"
        with open(out_yaml) as infile:
            old_data = yaml.safe_load(infile)
        old_data["Global Comparison"][key] = {}
        old_data["Global Comparison"][key][sub_key1] = value1
        old_data["Global Comparison"][key][sub_key2] = value2
        old_data["Global Comparison"][key][sub_key3] = value3
        with open(out_yaml, 'w') as outfile:
            yaml.dump(old_data, outfile)
            yaml.dump(data, outfile)


def plotTraj(out_dir, plot, print_incrmt):
    """
    Plot and save the two trajectories

    """
    ns_ts_incrmt = int(print_incrmt * s_to_ns)

    ts_values = []
    for ts in range(start_ts, end_ts, ns_ts_incrmt):
        ts_values.append(ts)

    truth_poses = truth_TrajEval.poses_at(ts_values)
    cmpar_poses = cmpar_TrajEval.poses_at(ts_values)

    truth_x_values = truth_poses[:, 0, 3]
    truth_y_values = truth_poses[:, 1, 3]
    truth_z_values = truth_poses[:, 2, 3]

    cmpar_x_values = cmpar_poses[:, 0, 3]
    cmpar_y_values = cmpar_poses[:, 1, 3]
    cmpar_z_values = cmpar_poses[:, 2, 3]

    # Create a 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.plot(truth_x_values, truth_y_values, truth_z_values, color='red', linestyle='-', label='Tray_Truth')
    ax.plot(cmpar_x_values, cmpar_y_values, cmpar_z_values, color='blue', linestyle='-', label='Traj_Cmpar')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('3D Plot with Top-Down View')

    # Set the view angle for top-down view
    ax.view_init(elev=90, azim=0)  # 90 degrees elevation (top-down view)
    ax.legend()

    # Save the plot as a PNG image
    if out_dir:
        out_file = out_dir + '/Eval_Traj.png'
        logger.info(f"out png saved at {out_file} ")
        plt.savefig(out_dir + '/Eval_Traj_topdown.png', bbox_inches='tight')

    # Display the plot
    if plot:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Evaluate SLAM results.")
    parser.add_argument("truth_file", help="Path to grouth_truth SLAM output file A.osf")
    parser.add_argument("cmpar_file", help="Path to to_compare SLAM output file B.osf")
    parser.add_argument("--plot", action="store_true", help="Enable plotting")
    parser.add_argument("--save", action="store_true", help="Enable saving evaluation results with auto naming")
    parser.add_argument("--out_dir", type=str, help="Enable saving evaluation results and given the output dir")
    parser.add_argument("--print_incrmt", type=float, default=0.2,
                        help="Printout percnt increment value default: 0.2, printout every 20 percnt. "
                             "Suggest choose between 0.05 - 0.2")
    parser.add_argument("--cmpar_incrmt", type=float, default=0.1,
                        help="Compare ts increment value default: 0.1, unit is second. "
                             "Smaller value gives more precise results but takes a bit longer")

    args = parser.parse_args()

    if not args.save and not args.plot and not args.out_dir:
        logger.warning("Warning: Plotting and saving results are disabled. Try --plot and --save options")

    if args.print_incrmt <= 0 or args.print_incrmt > 0.5:
        logger.error("Error: --print_incrmt range is (0, 0.5]")
        exit(1)

    if args.cmpar_incrmt <= 0:
        logger.error("Error: --cmpar_incrmt needs to be greater than 0")
        exit(1)

    out_dir = None
    out_yaml = None

    if args.save or args.out_dir:
        if args.out_dir:
            out_dir = args.out_dir
            args.save = True
        else:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            out_dir = f"TrajEval_{current_time}"

        if not os.path.exists(out_dir):
            os.mkdir(out_dir)
        out_yaml = out_dir + '/results.yaml'

        logger.info(f"Evaluation results saved at '{out_dir}'.")
        evaluation_info = {
            "Evaluation SLAM results": {
                "Ground Truth file": f"{args.truth_file}",
                "To Compare file": f"{args.cmpar_file}"
            }
        }
        with open(out_yaml, 'w') as outfile:
            yaml.dump(evaluation_info, outfile)

    loadOSFPoses(args.truth_file, args.cmpar_file, out_yaml)

    compare(out_yaml, args.cmpar_incrmt, args.print_incrmt)

    if args.plot and _no_plotlib:
        logger.error("Error: Missing matplotlib. Run 'pip install matplotlib' to install")
        exit(1)

    if args.save and _no_yaml:
        logger.error("Error: Missing PyYAML. Run 'pip install PyYAML' to install")
        exit(1)

    plotTraj(out_dir, args.plot, args.print_incrmt)


if __name__ == "__main__":
    main()
