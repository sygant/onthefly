#pragma once
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QLayout>
#include <QTabWidget>
#include <QTableWidget>
#include <QPainter>
#include "../Scene/Database.h"


class CShowMatchWidget :public QWidget
{
	Q_OBJECT
public:
	CShowMatchWidget(QWidget* parent, Database* database, size_t CurrentImgID, size_t MatchedImgID, bool IsTwoViewGeometry);
private:
	QWidget* parent = nullptr;
	Database* database = nullptr;
	size_t CurrentImgID;
	size_t MatchedImgID;
	std::string ImageSaveDir;
	QLabel* ImageLabel;
	QImage Image;              //不带匹配线的两张原始影像的拼接影像
	QImage Image_Matches;      //带匹配线的拼接影像
	QPixmap Pixmap;
	QPushButton* HideMatchesButton;
	QPushButton* SaveButton;
	QPainter* ImagePainter;
	QPainter* Painter;

	QList<QColor> GenerateRandomColor(int Num);
	void Save();
	void HideMatches();
	void closeEvent(QCloseEvent* event);
};
class CMatchesTab :public QWidget
{
	Q_OBJECT
public:
	CMatchesTab(QWidget* parent, Database* database, std::string ImagePath);
private:
	CShowMatchWidget* ShowMatchWidget = nullptr;
	QWidget* parent = nullptr;
	Database* database = nullptr;
	std::string ImagePath;
	size_t MatchedImageNum;
	QTableWidget* Table;
	std::vector<std::pair<size_t, size_t>> MatchInfo;

	void GetMatchInfo();
	void ShowMatches();
};
class CTwoViewGeometriesTab :public QWidget
{
	Q_OBJECT
public:
	CTwoViewGeometriesTab(QWidget* parent, Database* database, std::string ImagePath);
private:
	CShowMatchWidget* ShowMatchWidget;
	QWidget* parent;
	Database* database = nullptr;
	std::string ImagePath;
	std::vector<std::pair<size_t, size_t>> TwoviewGeometryInfo;
	std::vector<int> Config;
	size_t MatchedImageNum;
	QTableWidget* Table;

	void GetTwoViewGeometriesInfo();
	void ShowMatches();
};
class COverlappingInfoWidget :public QWidget
{
	Q_OBJECT
public:
	COverlappingInfoWidget(QWidget* parent, Database* database, std::string ImagePath);

private:
	QWidget* parent;
	QTabWidget* Tab;
	QGridLayout* grid;
	Database* database = nullptr;
	std::string ImagePath;

	void closeEvent(QCloseEvent* event);
};





class CImageViewer :public QWidget
{
	Q_OBJECT
public:
	explicit CImageViewer(QWidget* parent, Database* database, const std::string& ImagePath);
	explicit CImageViewer(QWidget* parent, Database* database, image_t imageID);

public slots:
	void EnableButton(); //使"显示特征点"、"显示重叠影像"按钮可用

private:
	QWidget* parent = nullptr;
	Database* database = nullptr;
	std::string ImagePath;
	QLabel ImageLabel;
	QPixmap Pixmap;
	QImage* image = nullptr;
	QImage* imageWithKeypoints = nullptr;
	bool StopFlag;

	QPushButton* KeyPointsButton;
	QPushButton* SaveButton;
	QPushButton* MatchButton;
	QVBoxLayout* Layout;
	QHBoxLayout* SubLayout;
	QHBoxLayout* ButtonLayout;
	COverlappingInfoWidget* OverlappingInfoWidget;


	void Daemon();
	void ShowKeyPoints();
	void Save();
	void ShowOverlappingWidget();
	void closeEvent(QCloseEvent* event);
	void resizeEvent(QResizeEvent* event) override;


signals:
	void EnableButton_SIGNAL();
};