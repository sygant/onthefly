#pragma once
#include <QtCore>
#include <QtOpenGL>

class PointPainter {
public:
    PointPainter();
    ~PointPainter();

    struct Data {
        Data() : x(0), y(0), z(0), r(0), g(0), b(0), a(0) {}
        Data(const float x_,
            const float y_,
            const float z_,
            const float r_,
            const float g_,
            const float b_,
            const float a_)
            : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), a(a_) {}

        float x, y, z;
        float r, g, b, a;
    };

    void Setup();
    void Upload(const std::vector<PointPainter::Data>& data);
    void Render(const QMatrix4x4& pmv_matrix, float point_size);

private:
    QOpenGLShaderProgram shader_program_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;

    size_t num_geoms_;
};

class LinePainter {
public:
    LinePainter();
    ~LinePainter();

    struct Data {
        Data() {}
        Data(const PointPainter::Data& p1, const PointPainter::Data& p2)
            : point1(p1), point2(p2) {}

        PointPainter::Data point1;
        PointPainter::Data point2;
    };

    void Setup();
    void Upload(const std::vector<LinePainter::Data>& data);
    void Render(const QMatrix4x4& pmv_matrix,
        int width,
        int height,
        float line_width);

private:
    QOpenGLShaderProgram shader_program_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;

    size_t num_geoms_;
};



class TrianglePainter {
public:
    TrianglePainter();
    ~TrianglePainter();

    struct Data {
        Data() {}
        Data(const PointPainter::Data& p1,
            const PointPainter::Data& p2,
            const PointPainter::Data& p3)
            : point1(p1), point2(p2), point3(p3) {}

        PointPainter::Data point1;
        PointPainter::Data point2;
        PointPainter::Data point3;
    };

    void Setup();
    void Upload(const std::vector<TrianglePainter::Data>& data);
    void Render(const QMatrix4x4& pmv_matrix);

private:
    QOpenGLShaderProgram shader_program_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;

    size_t num_geoms_;
};


