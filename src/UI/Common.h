#pragma once
#include "../Base/types.h"
#include "../Feature/Bitmap.h"
#include "../Base/threading.h"
#include <QtCore>
#include <QtOpenGL>
#include <QAction>
#include <QApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QThread>
#include <QWaitCondition>

#include <Eigen/Core>

#ifdef _DEBUG
#define glDebugLog() GLError(__FILE__, __LINE__)
#else
#define glDebugLog()
#endif

// This class manages a thread-safe OpenGL context. Note that this class must be
// instantiated in the main Qt thread, since an OpenGL context must be created
// in it. The context can then be made current in any other thread.
class OpenGLContextManager : public QObject {
public:
    explicit OpenGLContextManager(int opengl_major_version = 2,
        int opengl_minor_version = 1);

    // Make the OpenGL context available by moving it from the thread where it was
    // created to the current thread and making it current.
    bool MakeCurrent();

private:
    QOffscreenSurface surface_;
    QOpenGLContext context_;
    QThread* parent_thread_;
    QThread* current_thread_;
    QAction* make_current_action_;
};

// Run and wait for the thread, that uses the OpenGLContextManager, e.g.:
//
//    class TestThread : public Thread {
//     private:
//      void Run() { opengl_context_.MakeCurrent(); }
//      OpenGLContextManager opengl_context_;
//    };
//    QApplication app(argc, argv);
//    TestThread thread;
//    RunThreadWithOpenGLContext(&thread);
//
void RunThreadWithOpenGLContext(Thread* thread);

// Get the OpenGL errors and print them to stderr.
void GLError(const char* file, int line);

Eigen::Matrix4f QMatrixToEigen(const QMatrix4x4& matrix);

QMatrix4x4 EigenToQMatrix(const Eigen::Matrix4f& matrix);

QImage BitmapToQImageRGB(const Bitmap& bitmap);

void DrawKeypoints(QPixmap* image,
    const FeatureKeypoints& points,
    const QColor& color = Qt::red);

QPixmap ShowImagesSideBySide(const QPixmap& image1, const QPixmap& image2);

QPixmap DrawMatches(const QPixmap& image1,
    const QPixmap& image2,
    const FeatureKeypoints& points1,
    const FeatureKeypoints& points2,
    const FeatureMatches& matches,
    const QColor& keypoints_color = Qt::red);