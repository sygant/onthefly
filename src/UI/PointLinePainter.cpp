#include "PointLinePainter.h"
#include "Common.h"

PointPainter::PointPainter() : num_geoms_(0) {}

PointPainter::~PointPainter() {
    vao_.destroy();
    vbo_.destroy();
}

void PointPainter::Setup() {
    vao_.destroy();
    vbo_.destroy();
    if (shader_program_.isLinked()) {
        shader_program_.release();
        shader_program_.removeAllShaders();
    }

    shader_program_.addShaderFromSourceFile(QOpenGLShader::Vertex,
        ":/shaders/points.v.glsl");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Fragment,
        ":/shaders/points.f.glsl");
    shader_program_.link();
    shader_program_.bind();

    vao_.create();
    vbo_.create();

#if _DEBUG
    glDebugLog();
#endif
}

void PointPainter::Upload(const std::vector<PointPainter::Data>& data) {
    num_geoms_ = data.size();
    if (num_geoms_ == 0) {
        return;
    }

    vao_.bind();
    vbo_.bind();

    // Upload data array to GPU
    vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vbo_.allocate(data.data(),
        static_cast<int>(data.size() * sizeof(PointPainter::Data)));

    // in_position
    shader_program_.enableAttributeArray("a_position");
    shader_program_.setAttributeBuffer(
        "a_position", GL_FLOAT, 0, 3, sizeof(PointPainter::Data));

    // in_color
    shader_program_.enableAttributeArray("a_color");
    shader_program_.setAttributeBuffer(
        "a_color", GL_FLOAT, 3 * sizeof(GLfloat), 4, sizeof(PointPainter::Data));

    // Make sure they are not changed from the outside
    vbo_.release();
    vao_.release();

#if _DEBUG
    glDebugLog();
#endif
}

void PointPainter::Render(const QMatrix4x4& pmv_matrix,
    const float point_size) {
    if (num_geoms_ == 0) {
        return;
    }

    shader_program_.bind();
    vao_.bind();

    shader_program_.setUniformValue("u_pmv_matrix", pmv_matrix);
    shader_program_.setUniformValue("u_point_size", point_size);

    QOpenGLFunctions* gl_funcs = QOpenGLContext::currentContext()->functions();
    gl_funcs->glDrawArrays(GL_POINTS, 0, (GLsizei)num_geoms_);

    // Make sure the VAO is not changed from the outside
    vao_.release();

#if _DEBUG
    glDebugLog();
#endif
}

LinePainter::LinePainter() : num_geoms_(0) {}

LinePainter::~LinePainter() {
    vao_.destroy();
    vbo_.destroy();
}

void LinePainter::Setup() {
    vao_.destroy();
    vbo_.destroy();
    if (shader_program_.isLinked()) {
        shader_program_.release();
        shader_program_.removeAllShaders();
    }

    shader_program_.addShaderFromSourceFile(QOpenGLShader::Vertex,
        ":/shaders/lines.v.glsl");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Geometry,
        ":/shaders/lines.g.glsl");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Fragment,
        ":/shaders/lines.f.glsl");
    shader_program_.link();
    shader_program_.bind();

    vao_.create();
    vbo_.create();

#if _DEBUG
    glDebugLog();
#endif
}

void LinePainter::Upload(const std::vector<LinePainter::Data>& data) {
    num_geoms_ = data.size();
    if (num_geoms_ == 0) {
        return;
    }

    vao_.bind();
    vbo_.bind();

    // Upload data array to GPU
    vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vbo_.allocate(data.data(),
        static_cast<int>(data.size() * sizeof(LinePainter::Data)));

    // in_position
    shader_program_.enableAttributeArray("a_pos");
    shader_program_.setAttributeBuffer(
        "a_pos", GL_FLOAT, 0, 3, sizeof(PointPainter::Data));

    // in_color
    shader_program_.enableAttributeArray("a_color");
    shader_program_.setAttributeBuffer(
        "a_color", GL_FLOAT, 3 * sizeof(GLfloat), 4, sizeof(PointPainter::Data));

    // Make sure they are not changed from the outside
    vbo_.release();
    vao_.release();

#if _DEBUG
    glDebugLog();
#endif
}

void LinePainter::Render(const QMatrix4x4& pmv_matrix,
    const int width,
    const int height,
    const float line_width) {
    if (num_geoms_ == 0) {
        return;
    }

    shader_program_.bind();
    vao_.bind();

    shader_program_.setUniformValue("u_pmv_matrix", pmv_matrix);
    shader_program_.setUniformValue("u_inv_viewport",
        QVector2D(1.0f / width, 1.0f / height));
    shader_program_.setUniformValue("u_line_width", line_width);

    QOpenGLFunctions* gl_funcs = QOpenGLContext::currentContext()->functions();
    gl_funcs->glDrawArrays(GL_LINES, 0, (GLsizei)(2 * num_geoms_));

    // Make sure the VAO is not changed from the outside
    vao_.release();

#if _DEBUG
    glDebugLog();
#endif
}

TrianglePainter::TrianglePainter() : num_geoms_(0) {}

TrianglePainter::~TrianglePainter() {
    vao_.destroy();
    vbo_.destroy();
}

void TrianglePainter::Setup() {
    vao_.destroy();
    vbo_.destroy();
    if (shader_program_.isLinked()) {
        shader_program_.release();
        shader_program_.removeAllShaders();
    }

    shader_program_.addShaderFromSourceFile(QOpenGLShader::Vertex,
        ":/shaders/triangles.v.glsl");
    shader_program_.addShaderFromSourceFile(QOpenGLShader::Fragment,
        ":/shaders/triangles.f.glsl");
    shader_program_.link();
    shader_program_.bind();

    vao_.create();
    vbo_.create();

#if _DEBUG
    glDebugLog();
#endif
}

void TrianglePainter::Upload(const std::vector<TrianglePainter::Data>& data) {
    num_geoms_ = data.size();
    if (num_geoms_ == 0) {
        return;
    }

    vao_.bind();
    vbo_.bind();

    // Upload data array to GPU
    vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vbo_.allocate(data.data(),
        static_cast<int>(data.size() * sizeof(TrianglePainter::Data)));

    // in_position
    shader_program_.enableAttributeArray("a_position");
    shader_program_.setAttributeBuffer(
        "a_position", GL_FLOAT, 0, 3, sizeof(PointPainter::Data));

    // in_color
    shader_program_.enableAttributeArray("a_color");
    shader_program_.setAttributeBuffer(
        "a_color", GL_FLOAT, 3 * sizeof(GLfloat), 4, sizeof(PointPainter::Data));

    // Make sure they are not changed from the outside
    vbo_.release();
    vao_.release();

#if _DEBUG
    glDebugLog();
#endif
}

void TrianglePainter::Render(const QMatrix4x4& pmv_matrix) {
    if (num_geoms_ == 0) {
        return;
    }

    shader_program_.bind();
    vao_.bind();

    shader_program_.setUniformValue("u_pmv_matrix", pmv_matrix);

    QOpenGLFunctions* gl_funcs = QOpenGLContext::currentContext()->functions();
    gl_funcs->glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(3 * num_geoms_));

    // Make sure the VAO is not changed from the outside
    vao_.release();

#if _DEBUG
    glDebugLog();
#endif
}
