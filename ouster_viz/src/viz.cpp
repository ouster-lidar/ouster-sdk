#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <vector>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkDoubleArray.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkImageMapToColors.h>
#include <vtkImageMapper.h>
#include <vtkImageMapper3D.h>
#include <vtkImageProperty.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle)

#include "ouster/colormaps.h"
#include "ouster/lidar_scan.h"
#include "ouster/viz.h"

namespace ouster {
namespace viz {

/**
 * Helper function to create vtk lookup tables for the specific color palettes
 **/
vtkSmartPointer<vtkLookupTable> palette_gen(const float c[palette_n][3],
                                            const int n) {
    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetNumberOfTableValues(n);
    lut->Build();
    for (int i = 0; i < n; i++) {
        lut->SetTableValue(i, c[i][0], c[i][1], c[i][2]);
    }
    return lut;
}

const vtkSmartPointer<vtkLookupTable> parula_lut =
    palette_gen(parula, parula_n);
const vtkSmartPointer<vtkLookupTable> viridis_lut =
    palette_gen(viridis, viridis_n);
const vtkSmartPointer<vtkLookupTable> magma_lut = palette_gen(magma, magma_n);
const vtkSmartPointer<vtkLookupTable> autumn_lut =
    palette_gen(autumn, autumn_n);
const vtkSmartPointer<vtkLookupTable> rainbow_lut =
    palette_gen(rainbow, rainbow_n);
const vtkSmartPointer<vtkLookupTable> grey_lut = palette_gen(grey, grey_n);

std::vector<std::pair<std::string, vtkSmartPointer<vtkLookupTable>>> palettes =
    {{"Parula", parula_lut},   {"Viridis", viridis_lut}, {"Magma", magma_lut},
     {"Rainbow", rainbow_lut}, {"Autumn", autumn_lut},   {"Grey", grey_lut}};

/**
 * Enum for specifiying rendering mode in which to visualize
 **/
enum RenderMode { COLOR_Z, COLOR_INTENSITY, COLOR_ZINTENSITY, COLOR_RANGE };

/**
 * Configuration struct used for the visualizing the 2d iamge and the point
 * cloud
 **/
struct VisualizerConfig {
    int palette;
    int c_palette;
    int point_size;
    Eigen::Matrix4d camera_rel_pose;
    RenderMode render_mode;
    bool cycle_range;      // for rendering range image
    double fraction_3d;    // what fraction of the viewport is the 3d viz
    int which_lidar_scan;  // 0, 1, 2, ... n
    bool parallel;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

/**
 * Configuration struct used for maintaining information about camera pose
 **/
struct VisualizerState {
    vtkSmartPointer<vtkCamera> camera;
    Eigen::Matrix4d camera_pose;
    std::deque<vtkSmartPointer<vtkActor>> actors;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

/**
 * Helper struct for keeping track of the range image visualization
 **/
struct LidarScanState {
    vtkSmartPointer<vtkImageData> lidar_scan;
    vtkSmartPointer<vtkImageMapToColors> lidar_scan_color;
    double pix_w;
    double pix_h;
    int W;
    int H;
    std::vector<int> w_offset;
};

/**
 * Struct of vectors to store point cloud from lidar packets
 **/
struct PointOS1Cloud {
    uint32_t ind = 0;
    uint32_t max_size = 0;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    Points xyz;
    std::vector<double> intensity;

    void clear() {
        ind = 0;
        x.clear();
        y.clear();
        z.clear();
    }
};

/**
 * Struct containing two buffer of lidar scans which are rendered by the
 * visualizer. A third lidar scan is polled separately by the driver program to
 * institute a triple-buffer system.
 **/
struct LidarScanBuffer {
    std::mutex ls_mtx;
    bool ls_dirty = true;  // bool: false if the ls_buf is new, and can we
                           // swapped in to render
    std::unique_ptr<ouster::LidarScan> ls_buf;
    std::unique_ptr<ouster::LidarScan> ls_render;
};

/**
 * struct containing everything that is passed from init_viz to run_viz
 **/
struct VizHandle {
    LidarScanState lss;
    std::shared_ptr<VisualizerState> visualizer_state;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderer> renderer_2d;
    vtkSmartPointer<vtkRenderWindow> render_window;
    VisualizerConfig config;
    viz::LidarScanBuffer lsb;
    viz::UserConfig uc;
    viz::SensorSpecifics ss;
    std::vector<double> xyz_lut;
};

/**
 * Method used in the visualizer to abstract the triple buffering process inside
 * the driver program. Swaps lidar_scan with ls_buf inside of the
 * LidarScanBuffer struct
 **/
void update_poll(viz::VizHandle& vh,
                 std::unique_ptr<ouster::LidarScan>& lidar_scan) {
    std::unique_lock<std::mutex> ls_guard(vh.lsb.ls_mtx);
    (lidar_scan).swap(vh.lsb.ls_buf);
    vh.lsb.ls_dirty = false;
    ls_guard.unlock();
    (lidar_scan)->reset();
}

/**
 * Applies color to points based on palette
 **/
void color_points_by_scalar(vtkSmartPointer<vtkActor>& actor,
                            const VisualizerConfig& config) {
    auto mapper = actor->GetMapper();
    mapper->SetLookupTable(palettes[config.palette].second);
    actor->GetProperty()->SetPointSize(config.point_size);
    actor->Modified();
}

/**
 * Creates call back function for event loop, intialized in main
 **/
class vtkTimerCallback : public vtkCommand {
   public:
    static vtkTimerCallback* New() { return new vtkTimerCallback; }

    virtual void Execute(vtkObject* vtkNotUsed(caller), unsigned long eventId,
                         void* vtkNotUsed(callData)) override {
        if (vtkCommand::TimerEvent == eventId) {
            ls_render();
            pc_render();
            render();
        }
    }
    std::function<void()> ls_render;
    std::function<void()> pc_render;
    std::function<void()> render;
};

/**
 * Updates the camera to the next state
 **/
void update_camera(const VisualizerConfig& config,
                   std::shared_ptr<VisualizerState>& state) {
    Eigen::Matrix4d true_camera_pose =
        state->camera_pose * config.camera_rel_pose;
    auto camera = state->camera;
    camera->SetFocalPoint(
        Eigen::Vector3d(state->camera_pose.topRightCorner<3, 1>()).data());
    camera->GetViewPlaneNormal()[0] = true_camera_pose(0, 2);
    camera->GetViewPlaneNormal()[1] = true_camera_pose(1, 2);
    camera->GetViewPlaneNormal()[2] = true_camera_pose(2, 2);
    camera->SetPosition(true_camera_pose.topRightCorner<3, 1>().data());
    camera->SetViewUp(true_camera_pose(0, 1), true_camera_pose(1, 1),
                      true_camera_pose(2, 1));
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> true_camera_pose_inv =
        true_camera_pose.inverse();
    camera->GetModelViewTransformObject()->SetMatrix(
        true_camera_pose_inv.data());
    camera->SetClippingRange(1.0, 2000);
}

/**
 * Applies filter for scaling the intensity scaling factor based on their
 * intensity
 **/
void color_intensity(Eigen::Ref<Eigen::ArrayXd> key_eigen) {
    key_eigen /= 500.0;
    key_eigen = key_eigen.max(0.0).sqrt();
}

/**
 * Applies filter for scaling the intensity scaling factor based on their range
 **/
void color_range(Eigen::Ref<Eigen::ArrayXd> range_eigen,
                 const VisualizerConfig& config) {
    if (config.cycle_range) {
        if (config.palette != 3) {
            range_eigen *= 1.0 / 2;
            range_eigen = range_eigen * 0.01 + 0.4 * (1 - range_eigen.cos());
        } else {
            range_eigen *= 1.0 / 12;
            range_eigen = range_eigen * 1.01 - range_eigen.floor();
        }
    } else {
        range_eigen = (range_eigen / 50.0).min(1.0).max(0.0);
    }
}

/**
 * Generates a point cloud from a lidar scan, by multiplying each pixel in the
 * lidar scan by a vector pointing radially outward
 **/
void lidar_scan_to_point_cloud(std::unique_ptr<ouster::LidarScan>& ls,
                               std::unique_ptr<viz::PointOS1Cloud>& pc,
                               const double* xyz_lut) {
    int n = ls->W * ls->H;
    pc->intensity.clear();
    for (int i = 0; i < n; i++) {
        pc->intensity.push_back(ls->intensity.at(i));
    }
    pc->xyz =
        Eigen::Map<const Eigen::Array<double, -1, 3>>(xyz_lut, n, 3).colwise() *
        Eigen::Map<const Eigen::Array<double, -1, 1>>(ls->range.data() + 0,
                                                      ls->W * ls->H, 1) /
        1000.0;  // divide by 1000, by point cloud is in meters, lidar scan in
                 // mm
}

/**
 * Called from draw, uses VTK to draws the point cloud
 **/
vtkSmartPointer<vtkActor> points_to_actor(const Points& cloud,
                                          const std::vector<double>& key,
                                          const VisualizerConfig& config) {
    const int n = cloud.rows();
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto vertices = vtkSmartPointer<vtkCellArray>::New();
    std::vector<vtkIdType> pid(n);
    auto color = vtkSmartPointer<vtkDoubleArray>::New();
    color->SetName("DepthArray");

    for (int i = 0; i < n; i++) {
        pid[i] = points->InsertNextPoint(
            (double)cloud(i, 0), (double)cloud(i, 1), (double)cloud(i, 2));
        color->InsertNextValue(key[i]);
    }
    vertices->InsertNextCell(n, pid.data());

    auto vtk_cloud = vtkSmartPointer<vtkPolyData>::New();
    vtk_cloud->SetPoints(points);
    vtk_cloud->SetVerts(vertices);
    vtk_cloud->GetPointData()->SetScalars(color);
    vtk_cloud->GetPointData()->SetActiveScalars("DepthArray");

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(vtk_cloud);
    mapper->SetColorModeToDefault();
    mapper->SetScalarRange(0.0, 1.0);
    mapper->SetLookupTable(palettes[config.c_palette].second);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    actor->GetProperty()->SetPointSize(2 * config.point_size);
    return actor;
}

/**
 * Computes color and intensity for the 3D point cloud visualization, then
 * calls points_to_actor to visualize point cloud
 **/
void draw_points(vtkSmartPointer<vtkRenderer>& renderer,
                 const VisualizerConfig& config,
                 std::shared_ptr<VisualizerState>& state_ptr,
                 std::unique_ptr<PointOS1Cloud>& point_cloud) {
    const int n = point_cloud->xyz.rows();
    std::vector<double> key(n);
    Eigen::Map<Eigen::ArrayXd> key_eigen(key.data(), n);

    switch (config.render_mode) {
        case COLOR_Z:
            key_eigen = (point_cloud->xyz.col(2) / 20.0).max(0.0).sqrt();
            break;
        case COLOR_INTENSITY:

            key_eigen = Eigen::Map<const Eigen::ArrayXd>(
                point_cloud->intensity.data(), n);
            color_intensity(key_eigen);
            break;
        case COLOR_ZINTENSITY:
            key_eigen = Eigen::Map<const Eigen::ArrayXd>(
                point_cloud->intensity.data(), n);
            color_intensity(key_eigen);
            key_eigen *= 0.7;
            key_eigen += (point_cloud->xyz.col(2) / 60.0).max(0.0).sqrt();
            break;

        case COLOR_RANGE:
            key_eigen = (point_cloud->xyz.col(0) * point_cloud->xyz.col(0) +
                         point_cloud->xyz.col(1) * point_cloud->xyz.col(1) +
                         point_cloud->xyz.col(2) * point_cloud->xyz.col(2))
                            .sqrt();
            color_range(key_eigen, config);

            break;

        default:
            cerr << "Invalid render mode" << endl;
    }
    if (state_ptr->actors.empty()) {
        auto actor = points_to_actor(point_cloud->xyz, key, config);
        renderer->AddActor(actor);
        state_ptr->actors.push_back(actor);
    } else {
        renderer->RemoveActor(state_ptr->actors.back());
        auto actor = points_to_actor(point_cloud->xyz, key, config);
        renderer->AddActor(actor);
        state_ptr->actors.back() = actor;
    }
}

/**
 * Inserts lidar scan into frame so that it can be rendered.
 **/
void lidar_scan_handler(LidarScanState& lss, const VisualizerConfig& config,
                        viz::UserConfig& uc,
                        std::unique_ptr<ouster::LidarScan>& ls) {
    const int n = lss.W * lss.H;
    // setting up the images
    std::vector<double> intensity(n, 0);
    Eigen::Map<Eigen::ArrayXd> intensity_eigen(intensity.data(), n);
    std::vector<double> range(n, 0);
    Eigen::Map<Eigen::ArrayXd> range_eigen(range.data(), n);
    std::vector<double> noise(n, 0);
    Eigen::Map<Eigen::ArrayXd> noise_eigen(noise.data(), n);

    // setting up the Intensity image
    intensity_eigen = Eigen::Map<const Eigen::Array<double, -1, 1>>(
        (ls)->intensity.data(), n);
    intensity_eigen *= uc.intensity_scale;
    color_intensity(intensity_eigen);
    // setting up the range image
    range_eigen =
        Eigen::Map<const Eigen::Array<double, -1, 1>>((ls)->range.data(), n);
    range_eigen *= 0.005 * uc.range_scale;
    color_range(range_eigen, config);
    // setting up the noise image
    if (uc.image_noise)
        noise_eigen = Eigen::Map<const Eigen::Array<double, -1, 1>>(
            (ls)->noise.data(), n);
    else
        noise.clear();
    noise_eigen *= 0.020 * uc.noise_scale;
    color_range(noise_eigen, config);
    // prepare for viewing
    for (int u = 0; u < lss.H; u++) {
        for (int v = 0; v < lss.W; v++) {
            const int vv = (v + lss.w_offset[u]) % lss.W;

            const double rs = range[u * lss.W + vv];
            *static_cast<double*>(
                lss.lidar_scan->GetScalarPointer(v, lss.H - u - 1, 0)) = rs;

            const double is = intensity[u * lss.W + vv];
            *static_cast<double*>(
                lss.lidar_scan->GetScalarPointer(v, lss.H * 2 - u - 1, 0)) = is;

            const double ns = noise[u * lss.W + vv];
            *static_cast<double*>(
                lss.lidar_scan->GetScalarPointer(v, lss.H * 3 - u - 1, 0)) = ns;
        }
    }

    lss.lidar_scan->Modified();
    lss.lidar_scan_color->SetLookupTable(palettes[config.palette].second);
};

/**
 * Reorthogonalizes a matrix - specifically used for the camera_rel_pose, as it
 * may drift into non-orthogonality due rounding during transformations
 **/
void reorthogonalize(Eigen::Matrix4d& m) {
    double A[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            A[i][j] = m(j, i);  // Eigen matrices are column-major. C matrices
                                // are row-major
        }
    }
    double B[3][3];
    vtkMath::Orthogonalize3x3(A, B);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            m(j, i) = B[i][j];
        }
    }
    m.row(3) << 0, 0, 0, 1;
}

class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera {
   public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

    void OnChar() {
        // suppress existing key bindings on char
    }
    void OnKeyPress() {
        // Get the keypress
        vtkRenderWindowInteractor* rwi = this->Interactor;
        std::string key = rwi->GetKeySym();

        if (key == "p") {
            std::cout << "Point size decreased" << std::endl;
            if (config->point_size > 1) config->point_size--;
            color_points_by_scalar(visualizer_state->actors.back(), *config);
        }

        if (key == "o") {
            std::cout << "Point size increased" << std::endl;
            if (config->point_size < 100) config->point_size++;
            color_points_by_scalar(visualizer_state->actors.back(), *config);
        }

        if (key == "i") {
            std::cout << "Intensity color mode" << std::endl;
            config->render_mode = COLOR_INTENSITY;
        }

        if (key == "z") {
            std::cout << "Z color mode" << std::endl;
            config->render_mode = COLOR_Z;
        }

        if (key == "Z") {
            std::cout << "Zintensity color mode" << std::endl;
            config->render_mode = COLOR_ZINTENSITY;
        }

        if (key == "r") {
            std::cout << "Range color mode" << std::endl;
            config->render_mode = COLOR_RANGE;
        }

        if (key == "c") {
            config->palette = (config->palette + 1) % palettes.size();
            std::cout << "Cycling color mode: "
                      << palettes[config->palette].first << endl;
            color_points_by_scalar(visualizer_state->actors.back(), *config);
        }

        if (key == "C") {
            config->c_palette = (config->c_palette + 1) % palettes.size();
            std::cout << "Cycling color mode: "
                      << palettes[config->c_palette].first << std::endl;
            color_points_by_scalar(visualizer_state->actors.back(), *config);
        }

        if (key == "v") {
            config->cycle_range = !config->cycle_range;
            std::cout << "Cycle range: " << config->cycle_range << std::endl;
        }

        if (key == "a") {
            std::cout << "Top-down view" << endl;
            // clang-format off
            config->camera_rel_pose <<
                0,      1,          0,          0,
                -1,     0,          0,          0,
                0,      0,          1,          100,
                0,      0,          0,          1;
            // clang-format on
        }
        if (key == "f") {
            std::cout << "Front view" << std::endl;
            // clang-format off
            config->camera_rel_pose <<
                0,      -0.447215,  0.894427,   200,
                1,      0,          0,          0,
                0,      0.894427,   0.447215,   100,
                0,      0,          0,          1;
            // clang-format on
        }
        if (key == "d") {
            config->fraction_3d += 0.1;
            if (config->fraction_3d > 1.0) {
                config->fraction_3d = 0;
            }
            std::cout << "Fraction 3d: " << config->fraction_3d << std::endl;
        }
        if (key == "Left") {
            std::cout << "Rotate camera left" << std::endl;
            Eigen::Matrix4d r;
            // clang-format off
            r <<
                0.980785,   0.19509,    0,      0,
                -0.19509,   0.980785,   0,      0,
                0.,         0.,         1,      0,
                0.,         0.,         0.,     1.;
            // clang-format on
            Eigen::Matrix4d delta(r);
            config->camera_rel_pose = delta * config->camera_rel_pose;
            reorthogonalize(config->camera_rel_pose);
        }
        if (key == "Right") {
            std::cout << "Rotate camera right" << std::endl;
            Eigen::Matrix4d r;
            // clang-format off
            r <<
                0.980785,   -0.19509,   0,      0,
                0.19509,    0.980785,   0,      0,
                0.,         0.,         1,      0,
                0.,         0.,         0.,     1.;
            // clang-format on
            Eigen::Matrix4d delta(r);

            config->camera_rel_pose = delta * config->camera_rel_pose;
            reorthogonalize(config->camera_rel_pose);
        }
        if (key == "Up") {
            std::cout << "Rotate camera up" << std::endl;
            Eigen::Matrix4d r;
            // clang-format off
            r <<
                0.980785,   0.,     0.19509,    0,
                0.,         1.,     0.,         0,
                -0.19509,   0.,     0.980785,   0,
                0.,         0.,     0.,         1.;
            // clang-format on
            Eigen::Matrix4d delta(r);

            config->camera_rel_pose = delta * config->camera_rel_pose;
            reorthogonalize(config->camera_rel_pose);
        }
        if (key == "Down") {
            std::cout << "Rotate camera down" << std::endl;
            Eigen::Matrix4d r;
            // clang-format off
            r <<
                0.980785,   0.,     -0.19509,   0,
                0.,         1.,     0.,         0,
                0.19509,    0.,     0.980785,   0,
                0.,         0.,     0.,         1.;
            // clang-format on
            Eigen::Matrix4d delta(r);
            config->camera_rel_pose = delta * config->camera_rel_pose;
            reorthogonalize(config->camera_rel_pose);
        }
        if (key == "plus") {
            config->camera_rel_pose.block<3, 1>(0, 3) /= 1.2;
            if (config->parallel) {
                visualizer_state->camera->Zoom(1.2);
            }
        }
        if (key == "minus") {
            config->camera_rel_pose.block<3, 1>(0, 3) *= 1.2;
            if (config->parallel) {
                visualizer_state->camera->Zoom(1.0 / 1.2);
            }
        }
        if (key == "0") {
            auto& state = visualizer_state;
            if (!config->parallel) {
                std::cout << "Parallel projection on" << std::endl;
                config->parallel = true;
                state->camera->ParallelProjectionOn();
                config->camera_rel_pose.block<3, 1>(0, 3).normalize();
            } else {
                std::cout << "Parallel projection off" << std::endl;
                config->parallel = false;
                state->camera->ParallelProjectionOff();
            }
        }
        if (key == "n") {
            if (uc->image_noise) {
                uc->image_noise = false;
                std::cout << "Imaging noise off " << std::endl;
            } else {
                uc->image_noise = true;
                std::cout << "Imaging noise on " << std::endl;
            }
        }
    }
    void OnLeftButtonUp() {
        update_camera();
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
    }
    void OnRightButtonUp() {
        update_camera();
        vtkInteractorStyleTrackballCamera::OnRightButtonUp();
    }
    void OnMiddleButtonUp() {
        update_camera();
        vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
    }
    void OnMouseWheelForward() {
        vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
        update_camera();
    }
    void OnMouseWheelBackward() {
        vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
        update_camera();
    }
    void update_camera() {
        auto& state = visualizer_state;
        auto camera = state->camera;
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> m =
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
                (double*)(camera->GetModelViewTransformMatrix()->Element));
        config->camera_rel_pose = state->camera_pose.inverse() * m.inverse();
    }
    UserConfig* uc;
    vtkSmartPointer<vtkRenderer> renderer;
    VisualizerConfig* config;
    std::shared_ptr<VisualizerState> visualizer_state;
};
vtkStandardNewMacro(KeyPressInteractorStyle);

/**
 * Initializes the LidarScanState
 **/
LidarScanState load_lidar_scan_config(const viz::SensorSpecifics& ss) {
    int lidar_scan_w = ss.col_per_rev;
    int lidar_scan_h = ss.H;

    const auto& w_angles = ss.beam_azimuth_angles;
    const auto& h_angles = ss.beam_altitude_angles;
    const double pitch_w = 360.0 / lidar_scan_w;
    const double pitch_h =
        std::abs(h_angles.back() - h_angles.front()) / lidar_scan_h;

    std::vector<int> w_offset(lidar_scan_h);
    for (int i = 0; i < lidar_scan_h; i++) {
        w_offset[i] =
            std::round((w_angles[0] - w_angles[i]) / (360.0 / lidar_scan_w));
    }
    LidarScanState lss{vtkSmartPointer<vtkImageData>::New(),
                       vtkSmartPointer<vtkImageMapToColors>::New(),
                       1.0 / pitch_h,
                       1.0 / pitch_w,
                       lidar_scan_w,
                       lidar_scan_h,
                       w_offset};

    lss.lidar_scan->SetDimensions(lidar_scan_w, 3 * lidar_scan_h, 1024);
    lss.lidar_scan->AllocateScalars(VTK_DOUBLE, 1);
    lss.lidar_scan_color->SetLookupTable(parula_lut);
    lss.lidar_scan_color->SetInputData(lss.lidar_scan);
    return lss;
}

/**
 * Run visualizer renering loop
 **/
void run_viz(VizHandle& vh) {
    auto cb = vtkSmartPointer<vtkTimerCallback>::New();

    auto pc_render =
        std::unique_ptr<viz::PointOS1Cloud>(new viz::PointOS1Cloud());
    bool ls_new = false;

    /**
     * Loads the lidar scan into the renderer
     **/
    cb->ls_render = [&]() {
        if (!vh.lsb.ls_dirty) {
            std::unique_lock<std::mutex> ls_guard(vh.lsb.ls_mtx);
            (vh.lsb.ls_render).swap(vh.lsb.ls_buf);
            ls_guard.unlock();
            vh.lsb.ls_dirty = true;
            ls_new = true;
        }
        lidar_scan_handler(vh.lss, vh.config, vh.uc, vh.lsb.ls_render);
    };

    /**
     * Loads the point cloud into the renderer.
     * If there is a new Lidar Scan, generates new point cloud from lidar scan
     **/
    cb->pc_render = [&]() {
        if (ls_new) {
            lidar_scan_to_point_cloud(vh.lsb.ls_render, pc_render,
                                      vh.xyz_lut.data());
        }
        draw_points(vh.renderer, vh.config, vh.visualizer_state, pc_render);
    };

    /**
     * Renders the point cloud and the lidar scan
     **/
    cb->render = [&]() {
        update_camera(vh.config, vh.visualizer_state);
        vh.render_window->Render();

        auto camera_2d = vh.renderer_2d->GetActiveCamera();
        const double* z = camera_2d->GetFocalPoint();
        const double r = camera_2d->GetDistance();

        camera_2d->SetPosition(z[0], z[1], r);
        camera_2d->SetFocalPoint(z[0], z[1], 0);
        camera_2d->SetViewUp(0, 1, 0);
        camera_2d->SetClippingRange(8.0, 32000);

        vh.lss.lidar_scan_color->SetLookupTable(
            palettes[vh.config.palette].second);
        vh.renderer_2d->SetViewport(0, vh.config.fraction_3d, 1, 1);
        vh.renderer->SetViewport(0, 0, 1, vh.config.fraction_3d);
    };

    /**
     * Creates a VTK rendering event loop which runs infinitely, and creates a
     * timed callback function which every 30 milliseconds calls pc_load and
     * ls_load to draw and then it render the images.
     **/
    auto render_window_interactor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    render_window_interactor->SetRenderWindow(vh.render_window);
    auto style = vtkSmartPointer<KeyPressInteractorStyle>::New();
    style->renderer = vh.renderer;
    style->config = &vh.config;
    style->visualizer_state = vh.visualizer_state;
    style->uc = &vh.uc;
    render_window_interactor->SetInteractorStyle(style);
    style->SetCurrentRenderer(vh.renderer);
    render_window_interactor->Initialize();
    render_window_interactor->AddObserver(vtkCommand::TimerEvent, cb);
    render_window_interactor->CreateRepeatingTimer(30);
    vh.render_window->Render();
    render_window_interactor->Start();
}

/**
 * Initializes the visualizer based on user-given parameters
 **/
std::shared_ptr<VizHandle> init_viz(const std::vector<double>& xyz_lut,
                                    const viz::UserConfig& uc,
                                    const viz::SensorSpecifics& ss) {
    std::shared_ptr<ouster::viz::VisualizerState> visualizer_state;
    visualizer_state = std::allocate_shared<ouster::viz::VisualizerState>(
        Eigen::aligned_allocator<ouster::viz::VisualizerState>());
    visualizer_state->camera_pose =
        Eigen::Matrix4d(Eigen::Matrix4d::Identity());

    ouster::viz::VisualizerConfig config{
        1,
        4,
        1,  // palette, c_palette, point_size,
        Eigen::Matrix4d(Eigen::Matrix4d::Identity()),
        ouster::viz::COLOR_ZINTENSITY,
        false,  // camera_rel_pose render_mode, cycle range
        0.65,
        0,
        false};  // fraction_3d, which_lidar_scan, bool parallel

    config.camera_rel_pose << 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 100, 0, 0, 0, 1;

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetViewport(0, 0, 1, config.fraction_3d);
    auto render_window = vtkSmartPointer<vtkRenderWindow>::New();
    render_window->SetSize(3840, 2160);

    render_window->AddRenderer(renderer);
    renderer->SetBackground(0, 0, 0);

    auto camera = vtkSmartPointer<vtkCamera>::New();
    camera->SetPosition(1, 2, 403);
    camera->SetFocalPoint(1, 2, 3);
    camera->SetViewUp(1, 0, 0);
    camera->SetViewAngle(50);
    renderer->SetActiveCamera(camera);
    visualizer_state->camera = camera;

    ouster::viz::LidarScanState lss = ouster::viz::load_lidar_scan_config(ss);

    auto image_actor = vtkSmartPointer<vtkImageActor>::New();
    image_actor->GetMapper()->SetInputConnection(
        lss.lidar_scan_color->GetOutputPort());
    image_actor->GetProperty()->SetInterpolationTypeToNearest();
    image_actor->SetScale(lss.pix_w, lss.pix_h, 1);
    auto renderer_2d = vtkSmartPointer<vtkRenderer>::New();

    render_window->AddRenderer(renderer_2d);
    renderer_2d->SetViewport(0, config.fraction_3d, 1, 1);

    renderer_2d->AddActor2D(image_actor);

    auto camera_2d = renderer_2d->GetActiveCamera();

    camera_2d->SetFocalPoint(lss.W, 64 * 3 / 2 * lss.pix_h, 0);
    camera_2d->SetPosition(camera_2d->GetFocalPoint()[0],
                           camera_2d->GetFocalPoint()[1],
                           camera_2d->GetDistance());
    camera_2d->SetViewUp(0, 1, 0);
    camera_2d->SetClippingRange(8.0, 32000);

    std::shared_ptr<VizHandle> vh = std::make_shared<VizHandle>();

    vh->lss = lss;
    vh->visualizer_state = visualizer_state;
    vh->renderer = renderer;
    vh->renderer_2d = renderer_2d;
    vh->render_window = render_window;
    vh->config = config;
    vh->lsb.ls_buf = std::unique_ptr<ouster::LidarScan>(
        new ouster::LidarScan(ss.col_per_rev, ss.H));
    vh->lsb.ls_render = std::unique_ptr<ouster::LidarScan>(
        new ouster::LidarScan(ss.col_per_rev, ss.H));
    vh->uc = uc;
    vh->ss = ss;
    vh->xyz_lut = xyz_lut;
    return vh;
}
}
}
