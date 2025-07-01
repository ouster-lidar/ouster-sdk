#include <gtest/gtest.h>
#include <ouster/pose_conversion.h>
#include <ouster/pose_util.h>

bool isMatrixEqual(const Eigen::Matrix<double, 4, 4>& mat1,
                   const Eigen::Matrix<double, 4, 4>& mat2, double tol = 1e-4) {
    return (mat1 - mat2).norm() < tol;
}

TEST(transformation, euler_pose_to_matrix_test) {
    Eigen::Matrix<double, 6, 1> pose;
    pose << 0.1, 0.2, 0.3, 1, 2, 3;

    // Compute the transformation matrix using Euler angles.
    Eigen::Matrix4d transform_euler = euler_to_matrix(pose);

    // Expected transformation matrix for Euler pose.
    Eigen::Matrix4d expected;
    expected << 0.936293, -0.275096, 0.218351, 1, 0.289629, 0.956425, -0.036957,
        2, -0.198669, 0.0978434, 0.97517, 3, 0, 0, 0, 1;

    double tol = 1e-5;
    for (int i = 0; i < transform_euler.rows(); ++i) {
        for (int j = 0; j < transform_euler.cols(); ++j) {
            EXPECT_NEAR(transform_euler(i, j), expected(i, j), tol)
                << "Mismatch at (" << i << "," << j << ")";
        }
    }
}

TEST(transformation, quaternion_pose_to_matrix) {
    // Example 7-element pose: translation (x, y, z) and quaternion (qw, qx, qy,
    // qz) Expected order: qw, qx, qy, qz, x, y, z.
    Eigen::Matrix<double, 7, 1> pose2;
    pose2 << 0.8446, 0.1913, 0.4619, 0.1913, 1, 2, 3;

    // Compute the transformation matrix using the quaternion.
    Eigen::Matrix4d transform = quaternion_to_matrix(pose2);

    // Expected transformation matrix for quaternion pose.
    Eigen::Matrix4d expected;
    expected << 0.500051, -0.146437, 0.853525, 1, 0.499921, 0.853601, -0.146437,
        2, -0.707126, 0.499921, 0.500051, 3, 0, 0, 0, 1;

    double tol = 1e-5;
    for (int i = 0; i < transform.rows(); ++i) {
        for (int j = 0; j < transform.cols(); ++j) {
            EXPECT_NEAR(transform(i, j), expected(i, j), tol)
                << "Mismatch at (" << i << "," << j << ")";
        }
    }
}

TEST(TransformTest, TransforN3) {
    // Define known input points of shape (10, 3)
    ouster::core::Points points(10, 3);
    points << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
        25.0, 26.0, 27.0, 28.0, 29.0, 30.0;

    // Define a non-identity transformation matrix (4x4)
    ouster::core::Pose transformation_matrix;
    transformation_matrix << 0.866, -0.5, 0.0, 1.0, 0.5, 0.866, 0.0, 2.0, 0.0,
        0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0;

    // Expected transformed points (manually calculated)
    ouster::core::Points expected_points(10, 3);
    expected_points << 0.866, 4.232, 2, 1.964, 8.33, 5, 3.062, 12.428, 8, 4.16,
        16.526, 11, 5.258, 20.624, 14, 6.356, 24.722, 17, 7.454, 28.82, 20,
        8.552, 32.918, 23, 9.65, 37.016, 26, 10.748, 41.114, 29;

    // Call the transform function
    ouster::core::Points result =
        ouster::core::transform(points, transformation_matrix);

    // Compare the results
    for (int i = 0; i < result.rows(); ++i) {
        for (int j = 0; j < result.cols(); ++j) {
            EXPECT_NEAR(result(i, j), expected_points(i, j),
                        1e-3);  // Small tolerance for floating-point comparison
        }
    }
}

TEST(pose_util, interp_pose) {
    // Create three sample poses.
    Eigen::Matrix<double, 4, 4> prev_pose =
        Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 4, 4> curr_pose =
        Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 4, 4> next_pose =
        Eigen::Matrix<double, 4, 4>::Identity();

    // Set a translation in curr_pose.
    curr_pose(0, 3) = 5.0;  // x translation
    curr_pose(1, 3) = 3.0;  // y translation
    curr_pose(2, 3) = 2.0;  // z translation

    uint64_t prev_ts = 101000;
    uint64_t curr_ts = 102000;
    uint64_t next_ts = 103000;

    // Set yaw rotation (rotation about the Z-axis) to 0.3 radians for
    // curr_pose.
    double yaw = 0.3;
    Eigen::Matrix3d yawRotation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Set pitch rotation (rotation about the Y-axis) to -0.1 radians for
    // curr_pose.
    double pitch = -0.1;
    Eigen::Matrix3d pitchRotation =
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // Combine the yaw and pitch rotations.
    Eigen::Matrix3d combinedRotation = yawRotation * pitchRotation;

    // Update the rotation portion of curr_pose.
    curr_pose.block<3, 3>(0, 0) = combinedRotation;

    std::cout << " curr_pose = \n" << curr_pose << std::endl;

    // For next_pose, set up a different rotation.
    double yaw2 = 0.4;
    Eigen::Matrix3d yawRotation2 =
        Eigen::AngleAxisd(yaw2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Set pitch rotation (rotation about the Y-axis) to -0.3 radians for
    // next_pose.
    double pitch2 = -0.3;
    Eigen::Matrix3d pitchRotation2 =
        Eigen::AngleAxisd(pitch2, Eigen::Vector3d::UnitY()).toRotationMatrix();

    Eigen::Matrix3d combinedRotation2 = yawRotation2 * pitchRotation2;

    // Update the rotation portion of next_pose.
    next_pose.block<3, 3>(0, 0) = combinedRotation2;

    std::cout << " next_pose = \n" << next_pose << std::endl;

    // Generate inquiry timestamps.
    std::vector<uint64_t> inquire_ts = {100000, 100500, 101000, 101500,
                                        102000, 102500, 103000, 103500};

    std::cout << " size = " << inquire_ts.size() << std::endl;

    // Combine the poses and timestamps into vectors.
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> poses = {
        prev_pose, curr_pose, next_pose};
    std::vector<uint64_t> timestamps = {prev_ts, curr_ts, next_ts};

    // Execute the interpolation function.
    auto results = ouster::interp_pose(inquire_ts, timestamps, poses);

    // Define expected results.
    std::vector<Eigen::Matrix<double, 4, 4>> expected_results = {
        // Pose 0:
        (Eigen::Matrix<double, 4, 4>() << 9.50563566e-01, 2.94044472e-01,
         9.98336350e-02, -5.83461852e+00, -2.95520849e-01, 9.55336290e-01,
         -2.79214613e-08, -1.38840457e+00, -9.53747027e-02, -2.95028941e-02,
         9.95004143e-01, -1.42462609e+00, 0.00000000e+00, 0.00000000e+00,
         0.00000000e+00, 1.00000000e+00)
            .finished(),
        // Pose 1:
        (Eigen::Matrix<double, 4, 4>() << 0.98756338, 0.149066, 0.04997893,
         -2.84746798, -0.14943741, 0.98876405, 0.00375782, -0.91059756,
         -0.0488572, -0.01117981, 0.9987432, -0.7874577, 0, 0, 0, 1)
            .finished(),
        // Pose 2:
        (Eigen::Matrix<double, 4, 4>() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 1)
            .finished(),
        // Pose 3:
        (Eigen::Matrix<double, 4, 4>() << 0.98756338, -0.14943741, -0.0488572,
         2.63750479, 0.149066, 0.98876405, -0.01117981, 1.31602317, 0.04997893,
         0.00375782, 0.9987432, 0.93220328, 0, 0, 0, 1)
            .finished(),
        // Pose 4:
        (Eigen::Matrix<double, 4, 4>() << 9.50563566e-01, -2.95520849e-01,
         -9.53747027e-02, 5.00000000e+00, 2.94044472e-01, 9.55336290e-01,
         -2.95028941e-02, 3.00000000e+00, 9.98336350e-02, -2.79214613e-08,
         9.95004143e-01, 2.00000000e+00, 0.00000000e+00, 0.00000000e+00,
         0.00000000e+00, 1.00000000e+00)
            .finished(),
        // Pose 5:
        (Eigen::Matrix<double, 4, 4>() << 0.9208184, -0.34289665, -0.18578365,
         2.41554879, 0.33559594, 0.93936924, -0.07042739, 1.5451861, 0.1986689,
         0.00250261, 0.98006331, 1.14334982, 0, 0, 0, 1)
            .finished(),
        // Pose 6:
        (Eigen::Matrix<double, 4, 4>() << 8.79923149e-01, -3.89418384e-01,
         -2.72192394e-01, -2.23569899e-07, 3.72025983e-01, 9.21060403e-01,
         -1.15081184e-01, -1.19328240e-06, 2.95520591e-01, -1.60280748e-08,
         9.55336219e-01, 3.73615875e-06, 0.00000000e+00, 0.00000000e+00,
         0.00000000e+00, 1.00000000e+00)
            .finished(),
        // Pose 7:
        (Eigen::Matrix<double, 4, 4>() << 0.82838856, -0.43450466, -0.35352245,
         -2.21632003, 0.40287952, 0.90063797, -0.16290697, -1.61678417,
         0.38917989, -0.00747664, 0.9211313, -1.41550338, 0, 0, 0, 1)
            .finished()};

    // Check that the number of results matches the expected number.
    ASSERT_EQ(results.size(), expected_results.size());
    for (size_t i = 0; i < results.size(); ++i) {
        ASSERT_TRUE(isMatrixEqual(results[i], expected_results[i]))
            << "Matrix at index " << i << " is not equal.";
    }
}
