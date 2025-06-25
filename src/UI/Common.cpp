#include "Common.h"
#include "../Scene/CameraModel.h"
#include "../Base/misc.h"

OpenGLContextManager::OpenGLContextManager(int opengl_major_version,
    int opengl_minor_version)
    : parent_thread_(QThread::currentThread()),
    current_thread_(nullptr),
    make_current_action_(new QAction(this)) {
    CHECK_NOTNULL(QCoreApplication::instance());
    CHECK_EQ(QCoreApplication::instance()->thread(), QThread::currentThread());

    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setMajorVersion(opengl_major_version);
    format.setMinorVersion(opengl_minor_version);
    format.setSamples(4);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    context_.setFormat(format);

    surface_.create();
    CHECK(context_.create());
    context_.makeCurrent(&surface_);
    CHECK(context_.isValid()) << "Could not create valid OpenGL context";

    connect(
        make_current_action_,
        &QAction::triggered,
        this,
        [this]() {
            CHECK_NOTNULL(current_thread_);
            context_.doneCurrent();
            context_.moveToThread(current_thread_);
        },
        Qt::BlockingQueuedConnection);
}

bool OpenGLContextManager::MakeCurrent() {
    current_thread_ = QThread::currentThread();
    make_current_action_->trigger();
    context_.makeCurrent(&surface_);
    return context_.isValid();
}

void RunThreadWithOpenGLContext(Thread* thread) {
    std::thread opengl_thread([thread]() {
        thread->Start();
        thread->Wait();
        CHECK_NOTNULL(QCoreApplication::instance())->exit();
        });
    CHECK_NOTNULL(QCoreApplication::instance())->exec();
    opengl_thread.join();
    // Make sure that all triggered OpenGLContextManager events are processed in
    // case the application exits before the contexts were made current.
    QCoreApplication::processEvents();
}

void GLError(const char* file, const int line) {
    GLenum error_code(glGetError());
    while (error_code != GL_NO_ERROR) {
        std::string error_name;
        switch (error_code) {
        case GL_INVALID_OPERATION:
            error_name = "INVALID_OPERATION";
            break;
        case GL_INVALID_ENUM:
            error_name = "INVALID_ENUM";
            break;
        case GL_INVALID_VALUE:
            error_name = "INVALID_VALUE";
            break;
        case GL_OUT_OF_MEMORY:
            error_name = "OUT_OF_MEMORY";
            break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            error_name = "INVALID_FRAMEBUFFER_OPERATION";
            break;
        default:
            error_name = "UNKNOWN_ERROR";
            break;
        }
        fprintf(stderr,
            "OpenGL error [%s, line %i]: GL_%s",
            file,
            line,
            error_name.c_str());
        error_code = glGetError();
    }
}


Eigen::Matrix4f QMatrixToEigen(const QMatrix4x4& matrix) {
    Eigen::Matrix4f eigen;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            eigen(r, c) = matrix(r, c);
        }
    }
    return eigen;
}

QMatrix4x4 EigenToQMatrix(const Eigen::Matrix4f& matrix) {
    QMatrix4x4 qt;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            qt(r, c) = matrix(r, c);
        }
    }
    return qt;
}

QImage BitmapToQImageRGB(const Bitmap& bitmap) {
    QImage image(bitmap.Width(), bitmap.Height(), QImage::Format_RGB32);
    for (int y = 0; y < image.height(); ++y) {
        QRgb* image_line = (QRgb*)image.scanLine(y);
        for (int x = 0; x < image.width(); ++x) {
            BitmapColor<uint8_t> color;
            if (bitmap.GetPixel(x, y, &color)) {
                image_line[x] = qRgba(color.r, color.g, color.b, 255);
            }
        }
    }
    return image;
}

QPixmap ShowImagesSideBySide(const QPixmap& image1, const QPixmap& image2) {
    QPixmap image = QPixmap(QSize(image1.width() + image2.width(),
        std::max(image1.height(), image2.height())));

    image.fill(Qt::black);

    QPainter painter(&image);
    painter.drawImage(0, 0, image1.toImage());
    painter.drawImage(image1.width(), 0, image2.toImage());

    return image;
}

void DrawKeypoints(QPixmap* pixmap,
    const FeatureKeypoints& points,
    const QColor& color) {
    if (pixmap->isNull()) {
        return;
    }

    const int pen_width = std::max(pixmap->width(), pixmap->height()) / 2048 + 1;
    const int radius = 3 * pen_width + (3 * pen_width) % 2;
    const float radius2 = radius / 2.0f;

    QPainter painter(pixmap);
    painter.setRenderHint(QPainter::Antialiasing);

    QPen pen;
    pen.setWidth(pen_width);
    pen.setColor(color);
    painter.setPen(pen);

    for (const auto& point : points) {
        painter.drawEllipse(point.x - radius2, point.y - radius2, radius, radius);
    }
}

QPixmap DrawMatches(const QPixmap& image1,
    const QPixmap& image2,
    const FeatureKeypoints& points1,
    const FeatureKeypoints& points2,
    const FeatureMatches& matches,
    const QColor& keypoints_color) {
    QPixmap image = ShowImagesSideBySide(image1, image2);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);

    // Draw keypoints

    const int pen_width = std::max(image.width(), image.height()) / 2048 + 1;
    const int radius = 3 * pen_width + (3 * pen_width) % 2;
    const float radius2 = radius / 2.0f;

    QPen pen;
    pen.setWidth(pen_width);
    pen.setColor(keypoints_color);
    painter.setPen(pen);

    for (const auto& point : points1) {
        painter.drawEllipse(point.x - radius2, point.y - radius2, radius, radius);
    }
    for (const auto& point : points2) {
        painter.drawEllipse(
            image1.width() + point.x - radius2, point.y - radius2, radius, radius);
    }

    // Draw matches

    pen.setWidth(std::max(pen_width / 2, 1));

    for (const auto& match : matches) {
        const point2D_t idx1 = match.point2D_idx1;
        const point2D_t idx2 = match.point2D_idx2;
        pen.setColor(QColor(0, 255, 0));
        painter.setPen(pen);
        painter.drawLine(QPoint(points1[idx1].x, points1[idx1].y),
            QPoint(image1.width() + points2[idx2].x, points2[idx2].y));
    }

    return image;
}