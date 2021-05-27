#include "ouster/point_viz.h"

#include <GLFW/glfw3.h>

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "cloud.h"
#include "common.h"
#include "image.h"
#include "misc.h"
#include "ouster/colormaps.h"

namespace ouster {
namespace viz {

namespace impl {

// needed for GLFW callbacks to access PointViz member data
static std::unordered_map<GLFWwindow*, PointViz*> window_to_viz{};

constexpr int default_window_width = 640;
constexpr int default_window_height = 480;
int window_width;
int window_height;

// Adapted from WTFPL-licensed code:
// https://github.com/opengl-tutorials/ogl/blob/2.1_branch/common/shader.cpp
GLuint load_shaders(const std::string& vertex_shader_code,
                    const std::string& fragment_shader_code) {
    // Create the shaders
    GLuint vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);

    GLint result = GL_FALSE;
    int info_log_length;

    // Compile Vertex Shader
    char const* vertex_source_pointer = vertex_shader_code.c_str();
    glShaderSource(vertex_shader_id, 1, &vertex_source_pointer, NULL);
    glCompileShader(vertex_shader_id);

    // Check Vertex Shader
    glGetShaderiv(vertex_shader_id, GL_COMPILE_STATUS, &result);
    glGetShaderiv(vertex_shader_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
        std::vector<char> vertex_shader_error_message(info_log_length + 1);
        glGetShaderInfoLog(vertex_shader_id, info_log_length, NULL,
                           &vertex_shader_error_message[0]);
        printf("%s\n", &vertex_shader_error_message[0]);
    }

    // Compile Fragment Shader
    char const* fragment_source_pointer = fragment_shader_code.c_str();
    glShaderSource(fragment_shader_id, 1, &fragment_source_pointer, NULL);
    glCompileShader(fragment_shader_id);

    // Check Fragment Shader
    glGetShaderiv(fragment_shader_id, GL_COMPILE_STATUS, &result);
    glGetShaderiv(fragment_shader_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
        std::vector<char> fragment_shader_error_message(info_log_length + 1);
        glGetShaderInfoLog(fragment_shader_id, info_log_length, NULL,
                           &fragment_shader_error_message[0]);
        printf("%s\n", &fragment_shader_error_message[0]);
    }

    // Link the program
    GLuint program_id = glCreateProgram();
    glAttachShader(program_id, vertex_shader_id);
    glAttachShader(program_id, fragment_shader_id);
    glLinkProgram(program_id);

    // Check the program
    glGetProgramiv(program_id, GL_LINK_STATUS, &result);
    glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
        std::vector<char> program_error_message(info_log_length + 1);
        glGetProgramInfoLog(program_id, info_log_length, NULL,
                            &program_error_message[0]);
        printf("%s\n", &program_error_message[0]);
    }

    glDetachShader(program_id, vertex_shader_id);
    glDetachShader(program_id, fragment_shader_id);

    glDeleteShader(vertex_shader_id);
    glDeleteShader(fragment_shader_id);

    return program_id;
}

}  // namespace impl

using namespace viz::impl;

struct PointViz::impl {
    std::vector<MultiCloud> clouds;
    DoubleBuffer<Image> image;
    DoubleBuffer<Cuboids> cuboids;
    GLFWwindow* window;
    Camera camera;
    Rings rings;

    GLuint palette_texture_id;
    GLfloat point_program_id;
    GLfloat point_size;
    bool lbutton_down;
    bool mbutton_down;
    double mouse_x;
    double mouse_y;

    CloudIds cloud_ids;

    std::unordered_multimap<int, std::function<void()>> key_handlers_;
    std::thread window_thread;
    std::mutex init_mutex;
    bool initialized;
    std::condition_variable init_condition;

    impl()
        : image(0, 0),
          palette_texture_id((GLuint)-1),
          point_size(3),
          lbutton_down(false),
          mbutton_down(false),
          mouse_x(0),
          mouse_y(0),
          initialized(false) {}
};

static void draw(PointViz::impl& pimpl) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // draw images
    if (pimpl.image.enabled) {
        pimpl.image.draw(pimpl.camera);
    } else {
        pimpl.camera.setOffset(0, 0);
    }

    pimpl.camera.update();

    // draw point clouds
    glUseProgram(pimpl.point_program_id);
    for (auto& cloud : pimpl.clouds) {
        if (!cloud.enabled) continue;
        cloud.draw(pimpl.camera, pimpl.cloud_ids, pimpl.palette_texture_id);
    }

    // draw rings
    if (pimpl.rings.enabled) {
        pimpl.rings.draw(pimpl.camera);
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    // enable culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    // draw cuboids
    if (pimpl.cuboids.enabled) {
        pimpl.cuboids.draw(pimpl.camera);
    }
    glDisable(GL_BLEND);
    // enable culling
    glDisable(GL_CULL_FACE);

    // Swap buffers
    glfwSwapBuffers(pimpl.window);
}

/**
 * callback for resizing the window. Should be called automatically by GLFW.
 *
 * @param window pointer to GLFW window
 * @param width  window width in pixels
 * @param height window height in pixels
 */
static void updateFBSize(GLFWwindow* window, int fb_width, int fb_height) {
    impl::window_width = fb_width;
    impl::window_height = fb_height;
    impl::window_to_viz[window]->pimpl->camera.update();
    glViewport(0, 0, fb_width, fb_height);
#ifdef __APPLE__
    // glfwPollEvents blocks during resize. Keep rending to avoid artifacts
    draw(*impl::window_to_viz[window]->pimpl);
#endif
}

/**
 * callback for keypress. Polled after each drawing.
 * Called automatically by GLFW.
 *
 * @param window pointer to GLFW window
 * @param key    which key was pressed, e.g. GLFW_KEY_A
 * @param action whether key was released or pressed, e.g. GLFW_PRESS
 */
static void handleKeyPress(GLFWwindow* window, int key, int /*scancode*/,
                           int action, int mods) {
    if (action != GLFW_PRESS && action != GLFW_REPEAT) {
        return;
    }
    auto pthis = impl::window_to_viz[window];
    auto& pimpl = pthis->pimpl;
    if (mods == 0) {
        if (key == GLFW_KEY_R) {
            pimpl->camera.toggleAutoRotate();
        }
        if (key == GLFW_KEY_W) {
            pimpl->camera.up();
        }
        if (key == GLFW_KEY_S) {
            pimpl->camera.down();
        }
        if (key == GLFW_KEY_A) {
            pimpl->camera.left();
        }
        if (key == GLFW_KEY_D) {
            pimpl->camera.right();
        }
        if (key == GLFW_KEY_E) {
            pimpl->image.write->changeSizeFraction(1);
            pimpl->image.read->changeSizeFraction(1);
        }
        if (key == GLFW_KEY_O) {
            pimpl->point_size =
                std::max(static_cast<GLfloat>(1), pimpl->point_size - 1);
            glPointSize(pimpl->point_size);
            std::cerr << "point size decreased: " << pimpl->point_size
                      << std::endl;
        }
        if (key == GLFW_KEY_P) {
            pimpl->point_size =
                std::min(static_cast<GLfloat>(10), pimpl->point_size + 1);
            glPointSize(pimpl->point_size);
            std::cerr << "point size increased: " << pimpl->point_size
                      << std::endl;
        }
        if (key == GLFW_KEY_SEMICOLON) {
            pimpl->rings.ring_size = std::min(2, pimpl->rings.ring_size + 1);
            std::cerr << "ring size radius increased: 10^"
                      << pimpl->rings.ring_size << std::endl;
        }
        if (key == GLFW_KEY_APOSTROPHE) {
            pimpl->rings.ring_size = std::max(-2, pimpl->rings.ring_size - 1);
            std::cerr << "ring size radius decreased: 10^"
                      << pimpl->rings.ring_size << std::endl;
        }
        if (key == GLFW_KEY_EQUAL) {
            pimpl->camera.zoomIn();
        }
        if (key == GLFW_KEY_MINUS) {
            pimpl->camera.zoomOut();
        }
        if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9) {
            size_t cloud_id = static_cast<size_t>(key - GLFW_KEY_1);
            if (cloud_id < pimpl->clouds.size()) {
                pimpl->clouds[cloud_id].enabled =
                    !pimpl->clouds[cloud_id].enabled;
            }
            for (size_t i = 0; i < pimpl->clouds.size(); i++) {
                std::cerr << i + 1;
            }
            std::cerr << std::endl;
            for (const auto& cloud : pimpl->clouds) {
                if (cloud.enabled) {
                    std::cerr << "*";
                } else {
                    std::cerr << " ";
                }
            }
            std::cerr << std::endl;
        }
        if (key == GLFW_KEY_0) {
            pimpl->camera.toggleOrthographic();
        }
    } else if (mods == GLFW_MOD_SHIFT) {
        if (key == GLFW_KEY_R) {
            // reset camera
            pimpl->camera.reset();
        }
    }

    // process custom key handlers from users
    pthis->callKeyHandlers(key);
}

/**
 * callback for mouse press. Polled after each drawing.
 * Keeps track of whether mouse is held down with lbutton_down member
 * variable. Called automatically by GLFW.
 *
 * shift + left click is the same as middle click to support people with
 * very few mouse buttons
 *
 * @param window pointer to GLFW window
 * @param button which button was pressed, e.g. GLFW_MOUSE_BUTTON_LEFT
 * @param action whether key was released or pressed, e.g. GLFW_PRESS
 */
static void handleMouseButton(GLFWwindow* window, int button, int action,
                              int mods) {
    auto& pimpl = impl::window_to_viz[window]->pimpl;
    if (GLFW_PRESS == action) {
        if (button == GLFW_MOUSE_BUTTON_LEFT && mods == 0) {
            pimpl->lbutton_down = true;
        } else if (button == GLFW_MOUSE_BUTTON_MIDDLE ||
                   (button == GLFW_MOUSE_BUTTON_LEFT &&
                    mods == GLFW_MOD_SHIFT)) {
            pimpl->mbutton_down = true;
        }
    }

    if (GLFW_RELEASE == action) {
        pimpl->lbutton_down = false;
        pimpl->mbutton_down = false;
    }
}

/**
 * callback for cursor movement.
 * If mouse is held down, this is used for dragging, and updates camera.
 * Called automatically by GLFW.
 *
 * @param window pointer to GLFW window
 * @param xpos   subpixel x position of mouse
 * @param ypos   subpixel y position of mouse
 */
static void handleCursorPos(GLFWwindow* window, double xpos, double ypos) {
    auto& pimpl = impl::window_to_viz[window]->pimpl;
    const double dx = (xpos - pimpl->mouse_x);
    const double dy = (ypos - pimpl->mouse_y);
    if (pimpl->lbutton_down) {
        constexpr double sensitivity = 3;
        if (dx > 0) {
            pimpl->camera.left(sensitivity * dx);
        } else {
            pimpl->camera.right(-sensitivity * dx);
        }
        if (dy > 0) {
            pimpl->camera.up(sensitivity * dy);
        } else {
            pimpl->camera.down(-sensitivity * dy);
        }
    } else if (pimpl->mbutton_down) {
        pimpl->camera.changeOffset3d(dx, dy);
    }
    pimpl->mouse_x = xpos;
    pimpl->mouse_y = ypos;
}

/**
 * callback for mouse scroll
 * Used for dollying the camera.
 * Called automatically by GLFW.
 *
 * @param window pointer to GLFW window
 * @param xoff   horizontal scroll amount (unused)
 * @param yoff   vertical scroll amount
 */
static void handleScroll(GLFWwindow* window, double xoff, double yoff) {
    (void)xoff;
    auto& pimpl = impl::window_to_viz[window]->pimpl;
    if (yoff > 0) {
        pimpl->camera.dollyIn(yoff * 5);
    } else {
        pimpl->camera.dollyOut(-yoff * 5);
    }
}

/**
 * callback for mouse entering or leaving the window
 * Used to avoid the "sticky" situation where one clicks and drags but
 * releases the mouse button outside of the window, and then the next time
 * when the mouse re-enters the window, it still thinks the mouse is pressed
 * causing the camera to freak out when the user moves the mouse around.
 * Called automatically by GLFW.
 *
 * @param window  pointer to GLFW window
 * @param entered whether it entered or left the window
 */
static void handleCursorEnter(GLFWwindow* window, int entered) {
    auto& pimpl = impl::window_to_viz[window]->pimpl;
    if (entered) {
        // do something? idk
    } else {
        pimpl->lbutton_down = false;
        pimpl->mbutton_down = false;
    }
}

PointViz::PointViz(const std::vector<CloudSetup>& viz_setups,
                   const std::string& name, const bool fork)
    : viz_setups(viz_setups),
      name(name),
      quit(false),
      pimpl(std::unique_ptr<impl>(new impl{})) {
    pimpl->image.enabled = false;
    if (fork) {
        pimpl->window_thread = std::thread([this]() {
            bool success = false;
            {
                std::unique_lock<std::mutex> init_lock(pimpl->init_mutex);
                success = this->initialize();
            }
            this->pimpl->initialized = true;
            this->pimpl->init_condition.notify_one();
            if (!success) return;
            this->drawLoop();
        });
        std::unique_lock<std::mutex> init_lock(pimpl->init_mutex);
        pimpl->init_condition.wait(
            init_lock, [this]() { return this->pimpl->initialized; });
    } else {
        // initialize on the main thread
        initialize();
    }
}

void PointViz::setRange(const idx cloud_id, const uint32_t* x) {
    auto& c = pimpl->clouds[cloud_id].write();
    c->setRange(x);
}

void PointViz::setKey(const idx cloud_id, const double* x) {
    auto& c = pimpl->clouds[cloud_id].write();
    c->setKey(x);
}

void PointViz::setXYZ(const idx cloud_id, const double* x) {
    auto& c = pimpl->clouds[cloud_id].write();
    c->setXYZ(x);
}

void PointViz::setOffset(const idx cloud_id, const double* x) {
    auto& c = pimpl->clouds[cloud_id].write();
    c->setOffset(x);
}

void PointViz::setMapPose(const idx cloud_id, const mat4d& map_pose) {
    auto& c = pimpl->clouds[cloud_id].write();
    c->setMapPose(map_pose);
}

void PointViz::setColumnPoses(const idx cloud_id, const double* rotation,
                              const double* translation) {
    auto& c = pimpl->clouds[cloud_id].write();
    c->setColumnPoses(rotation, translation);
}

void PointViz::setCloudMask(const idx cloud_id, const GLfloat* msk_data) {
    auto& c = pimpl->clouds[cloud_id].write();
    c->setMask(msk_data);
}

void PointViz::cloudSwap(size_t i) { pimpl->clouds[i].swap(); }

void PointViz::setImage(const GLfloat* image_data) {
    pimpl->image.enabled = true;
    pimpl->image.write->setImage(image_data);
}

void PointViz::setImageMask(const GLfloat* msk_data) {
    pimpl->image.write->setMask(msk_data);
}

void PointViz::resizeImage(size_t w, size_t h) {
    if (w == 0 || h == 0) {
        pimpl->image.enabled = false;
    } else {
        pimpl->image.enabled = true;
        pimpl->image.write->resize(w, h);
    }
}

void PointViz::setImageAspectRatio(GLfloat a) {
    pimpl->image.write->setAspectRatio(a);
}

void PointViz::disableImage() { pimpl->image.enabled = false; }

void PointViz::imageSwap() { pimpl->image.swap(); }

void PointViz::addCuboid(Cuboid&& cuboid) {
    pimpl->cuboids.write->push(std::move(cuboid));
}

void PointViz::cuboidSwap() {
    pimpl->cuboids.read->clear();
    pimpl->cuboids.swap();
}

void PointViz::setCameraTarget(const mat4d& target) {
    pimpl->camera.setTarget(target);
}

void PointViz::setPointCloudPalette(const float palette[][3],
                                    size_t palette_size) {
    if (pimpl->palette_texture_id != (GLuint)-1) {
        load_texture(palette, palette_size, 1, pimpl->palette_texture_id);
    } else {
        std::cerr << "Cannot set custom palette before initialization"
                  << std::endl;
    }
}

void PointViz::attachKeyHandler(int key, std::function<void()>&& f) {
    pimpl->key_handlers_.insert(std::make_pair<int, std::function<void()>>(
        std::move(key), std::move(f)));
}

void PointViz::callKeyHandlers(const int key) {
    auto p = pimpl->key_handlers_.equal_range(key);
    for (auto& it = p.first; it != p.second; ++it) {
        it->second();
    }
}

PointViz::~PointViz() {
    quit = true;
    if (pimpl->window_thread.joinable()) {
        pimpl->window_thread.join();
    }
}

bool PointViz::initialize() {
    glfwSetErrorCallback(error_callback);

    // avoid chdir to resources dir on macos
#ifdef __APPLE__
    glfwInitHint(GLFW_COCOA_CHDIR_RESOURCES, GLFW_FALSE);
#endif
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }
#ifdef __APPLE__
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
#endif
    glfwWindowHint(GLFW_SAMPLES, GLFW_DONT_CARE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // Open a window and create its OpenGL context
    pimpl->window = glfwCreateWindow(default_window_width,
                                     default_window_height, name.c_str(),
                                     /*glfwGetPrimaryMonitor()*/ NULL, NULL);
    window_width = default_window_width;
    window_height = default_window_height;
    if (pimpl->window == NULL) {
        std::cerr << "Failed to open GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwSetFramebufferSizeCallback(pimpl->window, updateFBSize);
    glfwMakeContextCurrent(pimpl->window);

    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        glfwTerminate();
        return false;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(pimpl->window, GLFW_STICKY_KEYS, GL_TRUE);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glPointSize(pimpl->point_size);  // for the points
    glLineWidth(1);                  // for the rings

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Create and compile our GLSL program from the shaders
    pimpl->point_program_id =
        load_shaders(point_vertex_shader_code, point_fragment_shader_code);

    // handles for buffers
    pimpl->cloud_ids = CloudIds(pimpl->point_program_id);

    glGenTextures(1, &pimpl->palette_texture_id);

    // we load the palette as a 1D texture
    load_texture(spezia, spezia_n, 1, pimpl->palette_texture_id);

    pimpl->clouds.reserve(viz_setups.size());
    for (const auto& vs : viz_setups) {
        pimpl->clouds.emplace_back(vs.accumulation, vs.xyz, vs.off, vs.n, vs.w,
                                   vs.extrinsic);
    }

    glfwSetKeyCallback(pimpl->window, handleKeyPress);
    glfwSetMouseButtonCallback(pimpl->window, handleMouseButton);
    glfwSetCursorPosCallback(pimpl->window, handleCursorPos);
    glfwSetCursorEnterCallback(pimpl->window, handleCursorEnter);
    glfwSetScrollCallback(pimpl->window, handleScroll);
    window_to_viz[pimpl->window] = this;
    pimpl->rings.initialize();
    pimpl->image.read->initialize();
    pimpl->image.write->initialize();
    pimpl->cuboids.read->initialize();
    pimpl->cuboids.write->initialize();
    return true;
}

void PointViz::drawLoop() {
    // since drawLoop may be called from a different thread,
    // we should set the opengl context to be current
    glfwMakeContextCurrent(pimpl->window);
    do {
        draw(*pimpl);
        glfwPollEvents();
    } while (!quit && glfwWindowShouldClose(pimpl->window) == 0);
    glDeleteProgram(pimpl->point_program_id);
    glfwTerminate();
    quit = true;
}

}  // namespace viz
}  // namespace ouster
