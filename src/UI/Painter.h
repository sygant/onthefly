#pragma once
#include <QtCore>
#include <QtOpenGL>


struct CPointPainterData final
{
	float X, Y, Z;
	float R, G, B, A;

	CPointPainterData()
	{
		X = Y = Z = 0;
		R = G = B = A = 0;
	}
	CPointPainterData(float x, float y, float z, float r, float g, float b, float a)
	{
		X = x;
		Y = y;
		Z = z;
		R = r;
		G = g;
		B = b;
		A = a;
	}
};
class CPointPainter final
{
public:
	CPointPainter();
	~CPointPainter();
	void Setup();
	void Upload(const std::vector<CPointPainterData>& data);
	void Render(const QMatrix4x4& pmvMatrix, float pointSize);


private:
	QOpenGLShaderProgram shaderProgram;
	QOpenGLVertexArrayObject vertexArrayObject;
	QOpenGLBuffer buffer;
	size_t numGeo;
};

struct CLinePainterData final
{
	CPointPainterData point1;
	CPointPainterData point2;

	CLinePainterData() {};
	CLinePainterData(const CPointPainterData& point1, const CPointPainterData& point2)
	{
		this->point1 = point1;
		this->point2 = point2;
	}
};
class CLinePainter final
{
public:
	CLinePainter();
	~CLinePainter();
	void Setup();
	void Upload(const std::vector<CLinePainterData>& data);
	void Render(const QMatrix4x4& pmvMatrix, int width, int height, float lineWidth);

private:
	QOpenGLShaderProgram shaderProgram;
	QOpenGLVertexArrayObject vertexArrayObject;
	QOpenGLBuffer buffer;
	size_t numGeo;
};

struct CTrianglePainterData final
{
	CPointPainterData point1;
	CPointPainterData point2;
	CPointPainterData point3;

	CTrianglePainterData() {}
	CTrianglePainterData(const CPointPainterData& point1, const CPointPainterData& point2, const CPointPainterData& point3)
	{
		this->point1 = point1;
		this->point2 = point2;
		this->point3 = point3;
	}
};
class CTrianglePainter final
{
public:
	CTrianglePainter();
	~CTrianglePainter();
	void Setup();
	void Upload(const std::vector<CTrianglePainterData>& data);
	void Render(const QMatrix4x4& pmvMatrix);


private:
	QOpenGLShaderProgram shaderProgram;
	QOpenGLVertexArrayObject vertexArrayObject;
	QOpenGLBuffer buffer;
	size_t numGeo;
};








