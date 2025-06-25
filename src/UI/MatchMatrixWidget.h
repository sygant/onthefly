#pragma once
#include <QWidget>
#include <QTimer>
#include <QPushButton>
#include <QGridLayout>
#include "../Scene/Database.h"

class MatchMatrixWidget :public QWidget
{
	Q_OBJECT
public:
	MatchMatrixWidget(QWidget* parent, Database* database);
	static void SaveMatchMatrix(const std::string& filePath, Database* database);
private:
	QWidget* parent = nullptr;
	Database* database = nullptr;
	QTimer updateTimer;
	static std::unordered_map<image_pair_t, size_t> matchMatrix;
	static size_t maxNumMatches;
	std::mutex drawingMutex;

	void showEvent(QShowEvent* event) override;
	void paintEvent(QPaintEvent* event) override;
	void closeEvent(QCloseEvent* event) override;
	static size_t GetValue(image_t i, image_t j);
	static QColor GetColor(size_t value);
};
class ShowMatchMatrixWidget :public QWidget
{
	Q_OBJECT
public:
	ShowMatchMatrixWidget(QWidget* parent, Database* database);

private:
	QWidget* parent = nullptr;
	Database* database = nullptr;
	MatchMatrixWidget* matchMatrix = nullptr;
	QPushButton* saveButton = nullptr;

	void showEvent(QShowEvent* event) override;
	void closeEvent(QCloseEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;
	void Save();
};
