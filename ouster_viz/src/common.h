/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "glfw.h"

namespace {
inline void print_with_line_numbers(const std::string& data) {
    std::istringstream stream(data);
    std::string line;
    int number = 1;
    while (std::getline(stream, line)) {
        printf("%03i%s\n", number, line.c_str());
        number++;
    }
}

inline bool replace(std::string& str, const std::string& from,
                    const std::string& to) {
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos) {
        return false;
    }
    str.replace(start_pos, from.length(), to);
    return true;
}
}  // namespace

namespace ouster {
namespace sdk {
namespace viz {
namespace impl {

/**
 * Compile a shader
 */
inline GLuint compile_shader(std::string shader_code, GLuint shader_type) {
    // Adapted from WTFPL-licensed code:
    // https://github.com/opengl-tutorials/ogl/blob/2.1_branch/common/shader.cpp
    GLuint shader_id = glCreateShader(shader_type);
    GLint result = GL_FALSE;
    int info_log_length = 0;

    // Change shader version to work for opengl es
    if (GLFWContext::is_opengl_es()) {
        replace(shader_code, "#version 330 core", "#version 310 es");
    }

    // Compile shader
    char const* source_pointer = shader_code.c_str();
    glShaderSource(shader_id, 1, &source_pointer, nullptr);
    glCompileShader(shader_id);

    // Check shader
    glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
    glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
        std::vector<char> shader_error_message(info_log_length + 1);
        glGetShaderInfoLog(shader_id, info_log_length, nullptr,
                           shader_error_message.data());
        print_with_line_numbers(shader_code);
        printf("Shader compile info:\n%s\n", shader_error_message.data());
    }
    if (!result) {
        throw std::runtime_error("Could not compile shader.");
    }
    return shader_id;
}

/**
 * Create a GLSL program from a list of shaders
 */
inline GLuint link_program(std::initializer_list<GLuint> shader_ids) {
    GLuint program_id = glCreateProgram();
    for (GLuint shader_id : shader_ids) {
        glAttachShader(program_id, shader_id);
    }
    glLinkProgram(program_id);

    GLint result = GL_FALSE;
    int info_log_length = 0;
    glGetProgramiv(program_id, GL_LINK_STATUS, &result);
    glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
        std::vector<char> program_error_message(info_log_length + 1);
        glGetProgramInfoLog(program_id, info_log_length, nullptr,
                            program_error_message.data());
        printf("Shader program link info:\n%s\n", program_error_message.data());
    }
    if (!result) {
        throw std::runtime_error("Could not link shader program.");
    }

    for (GLuint shader_id : shader_ids) {
        glDetachShader(program_id, shader_id);
        glDeleteShader(shader_id);
    }
    return program_id;
}

/**
 * Create a GLSL program from shader code
 *
 * @param[in] vertex_shader_code code of vertex shader
 * @param[in] fragment_shader_code code of fragment shader
 * @param[in] geometry_shader_code code of geometry shader
 * @return handle to program_id
 */
inline GLuint load_shaders(const std::string& vertex_shader_code,
                           const std::string& fragment_shader_code,
                           const std::string& geometry_shader_code = "") {
    GLuint vertex_shader_id =
        compile_shader(vertex_shader_code, GL_VERTEX_SHADER);
    GLuint fragment_shader_id =
        compile_shader(fragment_shader_code, GL_FRAGMENT_SHADER);
    if (!geometry_shader_code.empty()) {
        GLuint geometry_shader_id =
            compile_shader(geometry_shader_code, GL_GEOMETRY_SHADER);
        return link_program(
            {vertex_shader_id, fragment_shader_id, geometry_shader_id});
    }
    return link_program({vertex_shader_id, fragment_shader_id});
}

static std::string read_file(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.good()) {
        throw std::runtime_error("Unable to read file " + file_path + ".");
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    if (!file.good()) {
        throw std::runtime_error("Unable to read file " + file_path + ".");
    }
    return buffer.str();
}

inline GLuint load_shaders_from_file(
    const std::string& vertex_shader_path,
    const std::string& fragment_shader_path,
    const std::string& geometry_shader_path = "") {
    if (!geometry_shader_path.empty()) {
        return load_shaders(read_file(vertex_shader_path),
                            read_file(fragment_shader_path),
                            read_file(geometry_shader_path));
    }
    return load_shaders(read_file(vertex_shader_path),
                        read_file(fragment_shader_path));
}
/**
 * load a texture from an array of GLfloat or equivalent
 * such as float[n][3]
 *
 * @tparam F The type for the texture object.
 *
 * @param[in] texture array of at least size
 *                width * height * elements_per_texel where elements
 *                per texel is 3 for GL_RGB format and 1 for
 *                GL_RED format
 * @param[in] width   width of texture in texels
 * @param[in] height  height of texture in texels
 * @param[in] texture_id handle generated by glGenTextures
 * @param[in] internal_format internal format, e.g. GL_RGB or GL_RGB32F
 * @param[in] format  format, e.g. GL_RGB or GL_RED
 * @param[in] type    texture element type
 */
template <class F>
void load_texture(const F& texture, size_t width, size_t height,
                  GLuint texture_id, GLenum internal_format = GL_RGB,
                  GLenum format = GL_RGB, GLenum type = GL_FLOAT) {
    glBindTexture(GL_TEXTURE_2D, texture_id);

    // we have only 1 level, so we override base/max levels
    // https://www.khronos.org/opengl/wiki/Common_Mistakes#Creating_a_complete_texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, 0, format,
                 type, texture);
}

/**
 * The point vertex shader supports transforming the point cloud by an array of
 * transformations.
 *
 * @param[in] xyz            XYZ point before it was multiplied by range.
 *                           Corresponds to the "xyzlut" used by LidarScan.
 *
 * @param[in] range          Range of each point.
 *
 * @param[in] key            Key for colouring each point for aesthetic reasons.
 *
 * @param[in] trans_index    Index of which of the transformations to use for
 * this point. Normalized between 0 and 1. (0 being the first 1 being the last).
 *
 * @param[in] model          Extrinsic calibration of the lidar.
 *
 * @param[in] transformation The w transformations are stored as a w x 4
 *                            texture. Each column of the texture corresponds
 *                            one 4 x 4 transformation matrix, where the
 *                            four pixels' rgb values correspond to
 *                            four columns (3 rotation 1 translation)
 * @param[in] proj_view      Camera view matrix controlled by the visualizer.
 */
static const std::string POINT_VERTEX_SHADER_CODE =
    R"SHADER(
            #version 330 core

            in vec3 xyz;
            in vec3 offset;
            in float range;
            in float trans_index;

            uniform sampler2D transformation;
            uniform mat4 model;
            uniform mat4 proj_view;

            in vec4 vkey;
            in vec4 vmask;

            out vec4 key;
            out vec4 mask;
            void main() {
                key = vkey;
                mask = vmask;

                // Discard any points with range == 0 by putting them at inf
                if (range <= 0.0) {
                    const float inf = intBitsToFloat(int(0x7F800000u));
                    gl_Position = vec4(inf, inf, inf, inf);
                    return;
                }

                vec4 local_point = model * vec4(xyz * range + offset, 1.0);
                // Here, we get the four columns of the transformation.
                // Since this version of GLSL doesn't have texel fetch,
                // we use texture2D instead. Numbers are chosen to index
                // the middle of each pixel.
                // |     r0     |     r1     |     r2     |     t     |
                // 0   0.125  0.25  0.375   0.5  0.625  0.75  0.875   1
                vec4 r0 = texture(transformation, vec2(trans_index, 0.125));
                vec4 r1 = texture(transformation, vec2(trans_index, 0.375));
                vec4 r2 = texture(transformation, vec2(trans_index, 0.625));
                vec4 t = texture(transformation, vec2(trans_index, 0.875));
                mat4 car_pose = mat4(
                    r0.x, r0.y, r0.z, 0,
                    r1.x, r1.y, r1.z, 0,
                    r2.x, r2.y, r2.z, 0,
                     t.x,  t.y,  t.z, 1
                );

                gl_Position = proj_view * car_pose * local_point;
            })SHADER";
static const std::string POINT_FRAGMENT_SHADER_CODE =
    R"SHADER(
            #version 330 core
            in lowp vec4 key;
            in lowp vec4 mask;
            uniform bool mono;
            uniform sampler2D palette;
            out lowp vec4 color;
            void main() {
                // getting color from palette or as it set in the rgb data
                // the full resolved color will be in vec4(c, key.a)
                lowp vec3 c = mono ? texture(palette, vec2(key.r, 1)).rgb : key.rgb;
                // compositing the mask RGBA value on top of the resolved point color c
                // using "over" operator https://en.wikipedia.org/wiki/Alpha_compositing
                lowp float color_a = mask.a + key.a * (1.0 - mask.a);
                lowp vec3 color_rgb = mask.rgb * mask.a + c * key.a * (1.0 - mask.a);
                color = vec4(color_rgb / color_a, color_a);
            })SHADER";
static const std::string RING_VERTEX_SHADER_CODE =
    R"SHADER(
            #version 330 core
            in vec3 ring_xyz;
            uniform mat4 proj_view;
            out vec2 ring_xy;
            void main(){
                gl_Position = proj_view * vec4(ring_xyz, 1.0);
                gl_Position.z = gl_Position.w;
                ring_xy = ring_xyz.xy;
            })SHADER";
static const std::string RING_FRAGMENT_SHADER_CODE =
    R"SHADER(
            #version 330 core
            out lowp vec4 color;
            in mediump vec2 ring_xy;
            uniform mediump float ring_range;
            uniform mediump float ring_thickness;
            void main() {
                // Compute this fragment's distance from the center of the rings
                mediump float radius = length(ring_xy);

                // Convert to a signed distance from the nearest ring
                mediump float signedDistance = radius - round(radius/ring_range)*ring_range;

                // Compute how quickly distance changes per pixel at our location
                // Make sure to do this using radius since it is mostly continuous
                mediump vec2 gradient = vec2(dFdx(radius), dFdy(radius));
                mediump float len = length(gradient);// meters/pixel

                // Get far we are from the line in pixel coordinates
                //  meters/(meters/pixels) = pixels
                mediump float rangeFromLine = abs(signedDistance/len);

                // Draw a line within the thickness
                mediump float lineWeight = clamp(ring_thickness - rangeFromLine, 0.0f, 1.0f);
                
                // Don't draw anything outside our max radius or at the center
                if (radius > 1000.0 || radius < ring_range*0.1) { lineWeight = 0.0; }
                color = vec4(vec3(0.15), lineWeight);
            })SHADER";
static const std::string CUBOID_VERTEX_SHADER_CODE =
    R"SHADER(
            #version 330 core
            in vec3 cuboid_xyz;
            uniform vec4 cuboid_rgba;
            uniform mat4 proj_view;
            out vec4 rgba;
            void main(){
                gl_Position = proj_view * vec4(cuboid_xyz, 1.0);
                rgba = cuboid_rgba;
            })SHADER";
static const std::string CUBOID_FRAGMENT_SHADER_CODE =
    R"SHADER(
            #version 330 core
            in lowp vec4 rgba;
            out lowp vec4 color;
            void main() {
                color = rgba;
            })SHADER";
static const std::string LINES_VERTEX_SHADER_CODE =
    R"SHADER(
            #version 330 core
            in vec3 lines_xyz;
            uniform vec4 lines_rgba;
            uniform mat4 proj_view;
            out vec4 rgba;
            void main(){
                gl_Position = proj_view * vec4(lines_xyz, 1.0);
                rgba = lines_rgba;
            })SHADER";
static const std::string LINES_FRAGMENT_SHADER_CODE =
    R"SHADER(
            #version 330 core
            in lowp vec4 rgba;
            out lowp vec4 color;
            void main() {
                color = rgba;
            })SHADER";
static const std::string IMAGE_VERTEX_SHADER_CODE =
    R"SHADER(
            #version 330 core
            in vec2 vertex;
            in vec2 vertex_uv;
            out vec2 uv;
            void main() {
                gl_Position = vec4(vertex, 0, 1);
                uv = vertex_uv;
            })SHADER";
static const std::string IMAGE_FRAGMENT_SHADER_CODE =
    R"SHADER(
            #version 330 core
            in mediump vec2 uv;
            uniform bool mono;
            uniform bool use_palette;
            uniform sampler2D image;
            uniform sampler2D mask;
            uniform sampler2D palette;
            out lowp vec4 color;
            void main() {
                lowp vec4 m = texture(mask, uv);
                lowp vec4 itex = texture(image, uv);
                lowp vec3 key_color = use_palette ? texture(palette, vec2(itex.r, 1)).rgb : vec3(itex.r);
                lowp vec3 img_color = mono ? key_color : itex.rgb;
                lowp float color_a = m.a + itex.a * (1.0 - m.a);
                color = vec4((m.rgb * m.a + img_color * (1.0 - m.a)) / color_a, color_a);
            })SHADER";
static const std::string mesh_vertex_shader_code =
    R"SHADER(
#version 330 core
in vec3 mesh_xyz;
in vec3 mesh_normal;
uniform vec4 mesh_face_rgba;
uniform vec4 mesh_edge_rgba;
uniform mat4 proj_view;
out vec4 edge_rgba;
void main(){
    gl_Position = proj_view * vec4(mesh_xyz, 1.0);
    edge_rgba = mesh_edge_rgba;
})SHADER";
static const std::string mesh_fragment_shader_code =
    R"SHADER(
#version 330 core
in vec4 edge_rgba;
out vec4 color;

void main() {
    color = edge_rgba;
})SHADER";
}  // namespace impl
}  // namespace viz
}  // namespace sdk
}  // namespace ouster
