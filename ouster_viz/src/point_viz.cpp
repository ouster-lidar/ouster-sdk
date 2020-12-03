#include "ouster/point_viz.h"

namespace ouster {
namespace viz {
// needed for GLFW callbacks to access PointViz member data
std::unordered_map<GLFWwindow*, PointViz*> PointViz::window_to_viz{};
namespace impl {
int window_width, window_height;

// shamelessly copied from
// https://github.com/opengl-tutorials/ogl/blob/2.1_branch/common/shader.cpp
// which is licensed under WTFPL:
// http://www.opengl-tutorial.org/download/
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

using namespace impl;

bool PointViz::initialize() {
    glfwSetErrorCallback(impl::error_callback);
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
    window = glfwCreateWindow(impl::default_window_width,
                              impl::default_window_height, name.c_str(),
                              /*glfwGetPrimaryMonitor()*/ NULL, NULL);
    impl::window_width = impl::default_window_width;
    impl::window_height = impl::default_window_height;
    if (window == NULL) {
        std::cerr << "Failed to open GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwSetWindowSizeCallback(window, updateWindowSize);
    glfwMakeContextCurrent(window);

    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        glfwTerminate();
        return false;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glPointSize(point_size);  // for the points
    glLineWidth(1);           // for the rings

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Create and compile our GLSL program from the shaders
    point_program_id =
        load_shaders(point_vertex_shader_code, point_fragment_shader_code);

    // handles for buffers
    cloud_ids = CloudIds(point_program_id);

    glGenTextures(1, &palette_texture_id);

    // we load the palette as a 1D texture
    load_texture(spezia, spezia_n, 1, palette_texture_id);

    clouds.reserve(viz_setups.size());
    for (const auto& vs : viz_setups) {
        clouds.emplace_back(vs);
    }

    glfwSetKeyCallback(window, handleKeyPress);
    glfwSetMouseButtonCallback(window, handleMouseButton);
    glfwSetCursorPosCallback(window, handleCursorPos);
    glfwSetCursorEnterCallback(window, handleCursorEnter);
    glfwSetScrollCallback(window, handleScroll);
    window_to_viz[window] = this;
    rings.initialize();
    image.read->initialize();
    image.write->initialize();
    cuboids.read->initialize();
    cuboids.write->initialize();
    return true;
}

void PointViz::drawLoop() {
    // since drawLoop may be called from a different thread,
    // we should set the opengl context to be current
    glfwMakeContextCurrent(window);
    do {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // draw images
        if (image.enabled) {
            image.draw(camera);
        } else {
            camera.setOffset(0, 0);
        }

        camera.update();

        // draw point clouds
        glUseProgram(point_program_id);
        for (auto& cloud : clouds) {
            if (!cloud.enabled) continue;
            cloud.draw(camera, cloud_ids, palette_texture_id);
        }

        // draw rings
        if (rings.enabled) {
            rings.draw(camera);
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
        glBlendEquation(GL_FUNC_ADD);

        // enable culling
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);

        // draw cuboids
        if (cuboids.enabled) {
            cuboids.draw(camera);
        }
        glDisable(GL_BLEND);
        // enable culling
        glDisable(GL_CULL_FACE);

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    } while (!quit && glfwWindowShouldClose(window) == 0);
    glDeleteProgram(point_program_id);
    glfwTerminate();
    quit = true;
}

}  // namespace viz
}  // namespace ouster
