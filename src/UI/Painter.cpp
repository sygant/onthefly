#include "Painter.h"
#include "Common.h"

using namespace std;

CPointPainter::CPointPainter()
{
	numGeo = 0;
}
CPointPainter::~CPointPainter()
{
	vertexArrayObject.destroy();
	buffer.destroy();
}
void CPointPainter::Setup()
{
	vertexArrayObject.destroy();
	buffer.destroy();
	if (shaderProgram.isLinked()) 
	{
		shaderProgram.release();
		shaderProgram.removeAllShaders();
	}

	shaderProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/points.v.glsl");
	shaderProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/points.f.glsl");
	shaderProgram.link();
	shaderProgram.bind();

	vertexArrayObject.create();
	buffer.create();

#if _DEBUG
	glDebugLog();
#endif
}
void CPointPainter::Upload(const vector<CPointPainterData>& data)
{
	if (data.empty()) return;

	numGeo = data.size();
	vertexArrayObject.bind();
	buffer.bind();

	// 将数据数组上传到GPU
	buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
	buffer.allocate(data.data(), data.size() * sizeof(CPointPainterData));

	shaderProgram.enableAttributeArray("a_position");
	shaderProgram.setAttributeBuffer("a_position", GL_FLOAT, 0, 3, sizeof(CPointPainterData));

	shaderProgram.enableAttributeArray("a_color");
	shaderProgram.setAttributeBuffer("a_color", GL_FLOAT, 3 * sizeof(GLfloat), 4, sizeof(CPointPainterData));

	// 确保它们不会从外部被修改
	buffer.release();
	vertexArrayObject.release();

#if _DEBUG
	glDebugLog();
#endif
}
void CPointPainter::Render(const QMatrix4x4& pmvMatrix, float pointSize)
{
	if (numGeo == 0)return;

	shaderProgram.bind();
	vertexArrayObject.bind();

	shaderProgram.setUniformValue("u_pmv_matrix", pmvMatrix);
	shaderProgram.setUniformValue("u_point_size", pointSize);

	QOpenGLFunctions* glFunctions = QOpenGLContext::currentContext()->functions();
	glFunctions->glDrawArrays(GL_POINTS, 0, (GLsizei)numGeo);

	// Make sure the VAO is not changed from the outside
	vertexArrayObject.release();

#if _DEBUG
	glDebugLog();
#endif




}

CLinePainter::CLinePainter()
{
	numGeo = 0;
}
CLinePainter::~CLinePainter()
{
	vertexArrayObject.destroy();
	buffer.destroy();
}
void CLinePainter::Setup()
{
	vertexArrayObject.destroy();
	buffer.destroy();
	if (shaderProgram.isLinked())
	{
		shaderProgram.release();
		shaderProgram.removeAllShaders();
	}

	shaderProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/lines.v.glsl");
	shaderProgram.addShaderFromSourceFile(QOpenGLShader::Geometry, ":/shaders/lines.g.glsl");
	shaderProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/lines.f.glsl");
	shaderProgram.link();
	shaderProgram.bind();

	vertexArrayObject.create();
	buffer.create();

#if _DEBUG
	glDebugLog();
#endif
}
void CLinePainter::Upload(const vector<CLinePainterData>& data)
{
	if (data.empty()) return;
	numGeo = data.size();

	vertexArrayObject.bind();
	buffer.bind();

	// 将数据数组上传到GPU
	buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
	buffer.allocate(data.data(), data.size() * sizeof(CLinePainterData));

	shaderProgram.enableAttributeArray("a_pos");
	shaderProgram.setAttributeBuffer("a_pos", GL_FLOAT, 0, 3, sizeof(CPointPainterData));

	shaderProgram.enableAttributeArray("a_color");
	shaderProgram.setAttributeBuffer("a_color", GL_FLOAT, 3 * sizeof(GLfloat), 4, sizeof(CPointPainterData));

	// 确保它们不会从外部被修改
	buffer.release();
	vertexArrayObject.release();

#if _DEBUG
	glDebugLog();
#endif
}
void CLinePainter::Render(const QMatrix4x4& pmvMatrix, int width, int height, float lineWidth)
{
	if (numGeo == 0)return;

	shaderProgram.bind();
	vertexArrayObject.bind();

	shaderProgram.setUniformValue("u_pmv_matrix", pmvMatrix);
	shaderProgram.setUniformValue("u_inv_viewport", QVector2D(1.0f / width, 1.0f / height));
	shaderProgram.setUniformValue("u_line_width", lineWidth);

	QOpenGLFunctions* gl_funcs = QOpenGLContext::currentContext()->functions();
	gl_funcs->glDrawArrays(GL_LINES, 0, (GLsizei)(2 * numGeo));

	// 确保vertexArrayObject不会从外部被修改
	vertexArrayObject.release();

#if _DEBUG
	glDebugLog();
#endif
}

CTrianglePainter::CTrianglePainter()
{
	numGeo = 0;
}
CTrianglePainter::~CTrianglePainter()
{
	vertexArrayObject.destroy();
	buffer.destroy();
}
void CTrianglePainter::Setup()
{
	vertexArrayObject.destroy();
	buffer.destroy();
	if (shaderProgram.isLinked())
	{
		shaderProgram.release();
		shaderProgram.removeAllShaders();
	}

	shaderProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/triangles.v.glsl");
	shaderProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/triangles.f.glsl");
	shaderProgram.link();
	shaderProgram.bind();

	vertexArrayObject.create();
	buffer.create();

#if _DEBUG
	glDebugLog();
#endif
}
void CTrianglePainter::Upload(const vector<CTrianglePainterData>& data)
{
	if (data.empty()) return;

	numGeo = data.size();

	vertexArrayObject.bind();
	buffer.bind();

	buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
	buffer.allocate(data.data(), data.size() * sizeof(CTrianglePainterData));

	shaderProgram.enableAttributeArray("a_position");
	shaderProgram.setAttributeBuffer("a_position", GL_FLOAT, 0, 3, sizeof(CPointPainterData));

	shaderProgram.enableAttributeArray("a_color");
	shaderProgram.setAttributeBuffer("a_color", GL_FLOAT, 3 * sizeof(GLfloat), 4, sizeof(CPointPainterData));

	buffer.release();
	vertexArrayObject.release();

#if _DEBUG
	glDebugLog();
#endif
}
void CTrianglePainter::Render(const QMatrix4x4& pmvMatrix)
{
	if (numGeo == 0) 
	{
		return;
	}

	shaderProgram.bind();
	vertexArrayObject.bind();

	shaderProgram.setUniformValue("u_pmv_matrix", pmvMatrix);

	QOpenGLFunctions* glFunctions = QOpenGLContext::currentContext()->functions();
	glFunctions->glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(3 * numGeo));

	vertexArrayObject.release();

#if _DEBUG
	glDebugLog();
#endif
}
