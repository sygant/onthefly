#pragma once
#include <QListWidget>
#include <QDockWidget>
#include <QScrollbar>
#include <QImageReader>
#include "../Scene/Database.h"


class ImageListWidget :public QListWidget
{
	Q_OBJECT
public:
	std::vector<std::string> imagePaths;

	ImageListWidget(QDockWidget* parent, Database* database);
	~ImageListWidget();
	void AddImage(const std::string& imagePath);
	void ClearImage();
public slots:
	void itemDoubleClicked_SLOT(QListWidgetItem* item);

private:
	QDockWidget* parent;
	std::vector<QListWidgetItem*> items;
	Database* database;
	std::mutex imageListMutex;
	std::atomic_size_t changeCount = 0;

	QSize CalculateImageSize(QSize& OriginSize);
};


























