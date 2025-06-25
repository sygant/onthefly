#include "ImageViewer.h"
#include <QtWidgets>
#include "../Base/misc.h"

using namespace std;


//QString转string
std::string QString2StdString(QString QString)
{
	return std::string(QString.toLocal8Bit());
}

//string转QString
QString StdString2QString(std::string string)
{
	return QString::fromLocal8Bit(string.data());
}



CShowMatchWidget::CShowMatchWidget(QWidget* parent, Database* database, size_t CurrentImgID, size_t MatchedImgID, bool IsTwoViewGeometry) :QWidget(parent)
{
	setWindowFlags(Qt::Window);
	setWindowModality(Qt::ApplicationModal);
	this->parent = parent;
	this->CurrentImgID = CurrentImgID;
	this->MatchedImgID = MatchedImgID;
	this->ImageSaveDir = Database::imageDir;
	this->database = database;


	string CurrentImgPath = ImageSaveDir + "/" + database->ReadImage(CurrentImgID).Name();
	string MatchedImgPath = ImageSaveDir + "/" + database->ReadImage(MatchedImgID).Name();

	QImage CurrentImg(StdString2QString(CurrentImgPath));
	QImage MatchedImg(StdString2QString(MatchedImgPath));

	//横向拼接两张原始影像
	Image = QImage(CurrentImg.width() + MatchedImg.width(), max(CurrentImg.height(), MatchedImg.height()), QImage::Format_RGB888);
	ImagePainter = new QPainter(&Image);
	ImagePainter->drawPixmap(0, 0, CurrentImg.width(), CurrentImg.height(), QPixmap::fromImage(CurrentImg));
	ImagePainter->drawPixmap(CurrentImg.width(), 0, MatchedImg.width(), MatchedImg.height(), QPixmap::fromImage(MatchedImg));
	Image_Matches = Image; //拼接后的影像拷贝到Image_Matches

	FeatureKeypoints CurrentImg_Keypoints = database->ReadKeypoints(CurrentImgID);
	FeatureKeypoints MatchedImg_Keypoints = database->ReadKeypoints(MatchedImgID);

	Painter = new QPainter(&Image_Matches);
	Painter->setRenderHint(QPainter::Antialiasing, true);
	int PenSize = 2, CircleSize = 3;
	if (Image_Matches.width() < 1000 || Image_Matches.height() < 1000) //影像太小, 特征点也画小一些
	{
		PenSize = 1;
		CircleSize = 1;
	}
	Painter->setPen(QPen(Qt::red, PenSize, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	for (FeatureKeypoint& Keypoint : CurrentImg_Keypoints)
	{
		Painter->drawEllipse(QPoint(Keypoint.x, Keypoint.y), CircleSize, CircleSize);
	}
	for (FeatureKeypoint& Keypoint : MatchedImg_Keypoints)
	{
		Painter->drawEllipse(QPoint(Keypoint.x + CurrentImg.width(), Keypoint.y), CircleSize, CircleSize);
	}
	Painter->end();
	Painter->~QPainter();

	FeatureMatches Matches;
	if (IsTwoViewGeometry)
	{
		TwoViewGeometry TwoViewGeometries = database->ReadTwoViewGeometry(CurrentImgID, MatchedImgID);
		Matches = TwoViewGeometries.inlier_matches;
		
		setWindowTitle(tr("Two-view Matches between [ ") + StdString2QString(database->ReadImage(CurrentImgID).Name()) + tr(" ] and [ ") + StdString2QString(database->ReadImage(MatchedImgID).Name()) + tr(" ]"));
	}
	else
	{
		Matches = database->ReadMatches(CurrentImgID, MatchedImgID);
		setWindowTitle(tr("Matches between [ ") + StdString2QString(database->ReadImage(CurrentImgID).Name()) + tr(" ] and [ ") + StdString2QString(database->ReadImage(MatchedImgID).Name()) + tr(" ]"));
	}
	QList<QColor> Colors = GenerateRandomColor(Matches.size());

	//画匹配线
	//为了提高QPainter的绘制速度, 所以先把Image_Matches(QImage)转成Pixmap(QPixmap), 在Pixmap上画
	Pixmap = QPixmap::fromImage(Image_Matches);
	QPainter DrawLinePainter(&Pixmap);
	for (size_t i = 0; i < Matches.size(); i++)
	{
		DrawLinePainter.setPen(QPen(Colors[i], PenSize, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		FeatureKeypoint Point1 = CurrentImg_Keypoints[Matches[i].point2D_idx1];
		FeatureKeypoint Point2 = MatchedImg_Keypoints[Matches[i].point2D_idx2];
		DrawLinePainter.drawEllipse(QPoint(Point1.x, Point1.y), CircleSize, CircleSize);
		DrawLinePainter.drawEllipse(QPoint(Point2.x + CurrentImg.width(), Point2.y), CircleSize, CircleSize);
		DrawLinePainter.drawLine(QPoint(Point1.x, Point1.y), QPoint(Point2.x + CurrentImg.width(), Point2.y));
	}
	DrawLinePainter.end();
	Image_Matches = Pixmap.toImage();

	ImageLabel = new QLabel();
	ImageLabel->setScaledContents(true);
	ImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	Pixmap.scaled(this->width(), this->height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
	ImageLabel->setPixmap(Pixmap);

	SaveButton = new QPushButton(tr("Save"), this);
	HideMatchesButton = new QPushButton(tr("Hide matches"), this);
	connect(HideMatchesButton, &QPushButton::released, this, &CShowMatchWidget::HideMatches);
	connect(SaveButton, &QPushButton::released, this, &CShowMatchWidget::Save);


	QVBoxLayout* Layout = new QVBoxLayout();
	Layout->addWidget(ImageLabel);
	QHBoxLayout* SubLayout = new QHBoxLayout();
	QHBoxLayout* ButtonLayout = new QHBoxLayout();
	ButtonLayout->addSpacerItem(new QSpacerItem(20, 20, QSizePolicy::Expanding));
	ButtonLayout->addWidget(HideMatchesButton);
	ButtonLayout->addWidget(SaveButton);
	SubLayout->addLayout(ButtonLayout);
	Layout->addLayout(SubLayout);
	this->setLayout(Layout);

	resize(900, 700);
	setWindowIcon(QIcon(":/media/undistort.png"));
}
QList<QColor> CShowMatchWidget::GenerateRandomColor(int Num)
{
	QList<QColor> ColorList;
	int ValidNum = min(Num, 280); //最大只能生成280个不重复且差异尽可能大的颜色
	int Length = 280 / ValidNum;
	int h = rand() % Length + 10;
	for (int i = 0; i < ValidNum; i++)
	{
		int s = rand() % 200 + 55;
		int v = rand() % 155 + 100;
		QColor randColor = QColor::fromHsv(h, s, v);
		h = h + Length;
		ColorList << randColor;
	}
	while (ColorList.size() < Num) //如果需要的颜色超过280个, 那么把前面生成的差异尽可能大的280个颜色加进来, 后面不足的颜色就开始摆烂, 随机生成
	{
		ColorList << QColor(rand() % 256, rand() % 256, rand() % 256);
	}
	return ColorList;
}
void CShowMatchWidget::Save()
{
	QString filter("PNG (*.png)");
	const QString SavePath = QFileDialog::getSaveFileName(this, tr("Select destination..."), "", "PNG (*.png);;JPEG (*.jpg);;BMP (*.bmp)", &filter).toUtf8().constData();
	if (SavePath == "")
	{
		return;
	}
	if (HideMatchesButton->text() == tr("Hide matches")) //当前显示的影像是带匹配线的
	{
		Image_Matches.save(SavePath, nullptr, 100);
	}
	else //当前显示的影像是不带匹配线的
	{
		Image.save(SavePath, nullptr, 100);
	}
}
void CShowMatchWidget::HideMatches()
{
	if (HideMatchesButton->text() == tr("Hide matches")) //当前显示的影像是带匹配线的, 则显示不带匹配线的
	{
		Pixmap = QPixmap::fromImage(Image).scaled(this->width(), this->height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
		ImageLabel->setScaledContents(true);
		ImageLabel->setPixmap(Pixmap);
		HideMatchesButton->setText(tr("Show matches"));
	}
	else //当前显示的影像是不带匹配线的, 则显示带匹配线的
	{
		Pixmap = QPixmap::fromImage(Image_Matches).scaled(this->width(), this->height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
		ImageLabel->setScaledContents(true);
		ImageLabel->setPixmap(Pixmap);
		HideMatchesButton->setText(tr("Hide matches"));
	}
}
void CShowMatchWidget::closeEvent(QCloseEvent* event)
{
	Image.~QImage();
	Image_Matches.~QImage();
	deleteLater();
}


CMatchesTab::CMatchesTab(QWidget* parent, Database* database, string ImagePath)
{
	this->parent = parent;
	this->ImagePath = ImagePath;
	this->database = database;

	GetMatchInfo();
	MatchedImageNum = MatchInfo.size();

	QGridLayout* Layout = new QGridLayout(this);
	QLabel* MatchedImgNum_Label = new QLabel(tr("Matched images: ") + QString::number(MatchedImageNum), this);
	Layout->addWidget(MatchedImgNum_Label, 0, 0);
	QPushButton* ShowMatchedButton = new QPushButton(tr("Show matches"), this);
	connect(ShowMatchedButton, &QPushButton::released, this, &CMatchesTab::ShowMatches);
	Layout->addWidget(ShowMatchedButton, 0, 1, Qt::AlignRight);

	QStringList TableHeader({ tr("Image ID"),tr("Image Name"),tr("Matches Num") });
	Table = new QTableWidget(this);
	Table->setSortingEnabled(false); //为了使表格可以自动/手动排序, 在表格完成之前需要先关闭排序功能, 全部填完之后再打开
	Table->setColumnCount(3);
	Table->setHorizontalHeaderLabels(TableHeader);
	Table->setShowGrid(true);
	Table->setSelectionBehavior(QAbstractItemView::SelectRows);
	Table->setSelectionMode(QAbstractItemView::SingleSelection);
	Table->setEditTriggers(QAbstractItemView::NoEditTriggers);
	Table->horizontalHeader()->setStretchLastSection(true);
	Table->verticalHeader()->setVisible(false);
	Table->verticalHeader()->setDefaultSectionSize(20);
	Table->clearContents();
	Table->setRowCount(MatchedImageNum);

	for (size_t i = 0; i < MatchedImageNum; i++)
	{
		size_t MatchedImageID = MatchInfo[i].first;
		size_t MatchesNum = MatchInfo[i].second;

		QTableWidgetItem* item1 = new QTableWidgetItem;
		item1->setData(Qt::DisplayRole, MatchedImageID);
		Table->setItem(i, 0, item1);

		QTableWidgetItem* item2 = new QTableWidgetItem;
		item2->setData(Qt::DisplayRole, StdString2QString(database->ReadImage(MatchedImageID).Name()));
		Table->setItem(i, 1, item2);

		QTableWidgetItem* item3 = new QTableWidgetItem;
		item3->setData(Qt::DisplayRole, MatchesNum);
		Table->setItem(i, 2, item3);
	}
	Table->setSortingEnabled(true);
	Table->setCurrentCell(0, 0);
	Layout->addWidget(Table, 1, 0, 1, 2);
}
void CMatchesTab::GetMatchInfo()
{
	MatchInfo.resize(0);
	string ImageName = GetFileName(ImagePath);
	size_t CurrentImgID = database->ReadImageWithName(ImageName).CameraId();
	size_t ImagesNum = database->NumImages();
	MatchInfo.reserve(ImagesNum);
	for (size_t i = 1; i <= ImagesNum; i++)
	{
		if (i == CurrentImgID)continue;
		if (database->ExistsMatches(i, CurrentImgID))
		{
			size_t MatchesNum = database->ReadMatches(i, CurrentImgID).size();
			MatchInfo.emplace_back(i, MatchesNum);
		}
	}
}
void CMatchesTab::ShowMatches()
{
	QList<QTableWidgetItem*> items = Table->selectedItems();
	if (items.count() == 0)
	{
		QMessageBox::critical(this, "Error", tr("No image pair selected!"));
		return;
	}
	size_t MatchedImgID = Table->item(Table->row(items[0]), 0)->text().toInt();
	size_t CurrentImgID = database->ReadImageWithName(GetFileName(ImagePath)).CameraId();
	ShowMatchWidget = new CShowMatchWidget(this, database, CurrentImgID, MatchedImgID, false);
	ShowMatchWidget->show();
}

CTwoViewGeometriesTab::CTwoViewGeometriesTab(QWidget* parent, Database* database, string ImagePath)
{
	this->parent = parent;
	this->ImagePath = ImagePath;
	this->database = database;
	GetTwoViewGeometriesInfo();
	MatchedImageNum = TwoviewGeometryInfo.size();

	QGridLayout* Layout = new QGridLayout(this);
	QLabel* MatchedImgNum_Label = new QLabel(tr("Matched images: ") + QString::number(MatchedImageNum), this);
	Layout->addWidget(MatchedImgNum_Label, 0, 0);
	QPushButton* ShowMatchedButton = new QPushButton(tr("Show matches"), this);
	connect(ShowMatchedButton, &QPushButton::released, this, &CTwoViewGeometriesTab::ShowMatches);
	Layout->addWidget(ShowMatchedButton, 0, 1, Qt::AlignRight);

	QStringList TableHeader({ tr("Image ID"),tr("Image Name"),tr("Two-view Matches Num"),tr("Config") });
	Table = new QTableWidget(this);
	Table->setSortingEnabled(false); //为了使表格可以自动/手动排序, 在表格完成之前需要先关闭排序功能, 全部填完之后再打开
	Table->setColumnCount(4);
	Table->setHorizontalHeaderLabels(TableHeader);
	Table->setShowGrid(true);
	Table->setSelectionBehavior(QAbstractItemView::SelectRows);
	Table->setSelectionMode(QAbstractItemView::SingleSelection);
	Table->setEditTriggers(QAbstractItemView::NoEditTriggers);
	Table->horizontalHeader()->setStretchLastSection(true);
	Table->verticalHeader()->setVisible(false);
	Table->verticalHeader()->setDefaultSectionSize(20);
	Table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
	Table->clearContents();
	Table->setRowCount(MatchedImageNum);
	for (size_t i = 0; i < MatchedImageNum; i++)
	{
		size_t MatchedImgID = TwoviewGeometryInfo[i].first;
		size_t MatchesNum = TwoviewGeometryInfo[i].second;

		QTableWidgetItem* item1 = new QTableWidgetItem;
		item1->setData(Qt::DisplayRole, MatchedImgID);
		Table->setItem(i, 0, item1);

		QTableWidgetItem* item2 = new QTableWidgetItem;

		
		item2->setData(Qt::DisplayRole, StdString2QString(database->ReadImage(MatchedImgID).Name()));
		Table->setItem(i, 1, item2);

		QTableWidgetItem* item3 = new QTableWidgetItem;
		item3->setData(Qt::DisplayRole, MatchesNum);
		Table->setItem(i, 2, item3);

		QTableWidgetItem* item4 = new QTableWidgetItem;
		item4->setData(Qt::DisplayRole, QString::number(Config[i]));
		Table->setItem(i, 3, item4);
	}
	Table->setSortingEnabled(true);
	Table->setCurrentCell(0, 0);
	Layout->addWidget(Table, 1, 0, 1, 2);
}
void CTwoViewGeometriesTab::GetTwoViewGeometriesInfo()
{
	TwoviewGeometryInfo.resize(0);
	Config.resize(0);
	
	size_t CurrentImgID = database->ReadImageWithName(GetFileName(ImagePath)).ImageId();
	size_t ImagesNum = database->NumImages();
	TwoviewGeometryInfo.reserve(ImagesNum);
	Config.reserve(ImagesNum);
	for (size_t i = 1; i <= ImagesNum; i++)
	{
		if (i == CurrentImgID)continue;
		if (database->ExistsInlierMatches(i, CurrentImgID))
		{
			TwoViewGeometry Geometry = database->ReadTwoViewGeometry(i, CurrentImgID);
			TwoviewGeometryInfo.emplace_back(i, Geometry.inlier_matches.size());
			Config.push_back(Geometry.config);
		}
	}
}
void CTwoViewGeometriesTab::ShowMatches()
{
	QList<QTableWidgetItem*> items = Table->selectedItems();
	if (items.count() == 0)
	{
		QMessageBox::critical(this, "Error", tr("No image pair selected!"));
		return;
	}
	int MatchedImgID = Table->item(Table->row(items[0]), 0)->text().toInt();
	
	int CurrentImgID = database->ReadImageWithName(GetFileName(ImagePath)).ImageId();
	ShowMatchWidget = new CShowMatchWidget(nullptr, database, CurrentImgID, MatchedImgID, true);
	ShowMatchWidget->show();
}

COverlappingInfoWidget::COverlappingInfoWidget(QWidget* parent, Database* database, string ImagePath) :QWidget(parent)
{
	this->parent = parent;
	this->ImagePath = ImagePath;
	this->database = database;
	setWindowFlags(Qt::Window);
	grid = new QGridLayout(this);
	Tab = new QTabWidget(this);
	Tab->addTab(new CMatchesTab(this, database, ImagePath), tr("Matches"));
	Tab->addTab(new CTwoViewGeometriesTab(this, database, ImagePath), tr("Two-view Geometries"));
	grid->addWidget(Tab, 0, 0);
	
	size_t ImageID = database->ReadImageWithName(GetFileName(ImagePath)).ImageId();
	setWindowTitle(tr("Matches for image [ ") + StdString2QString(GetFileName(ImagePath)) + tr(" ] ID=") + QString::number(ImageID));
	setMinimumWidth(600);
	setWindowIcon(QIcon(":/media/database-management.png"));
}
void COverlappingInfoWidget::closeEvent(QCloseEvent* event)
{
	grid->~QGridLayout();
	deleteLater();
}

CImageViewer::CImageViewer(QWidget* parent, Database* database, const std::string& ImagePath) : parent(parent), database(database), ImagePath(ImagePath)
{
	if (!ExistsFile(ImagePath))
	{
		throw ImagePath + " does not exists!";
	}
	StopFlag = false;

	image = new QImage(StdString2QString(ImagePath));
	Pixmap = QPixmap::fromImage(*image).scaled(this->width(), this->height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
	ImageLabel.setScaledContents(true);
	ImageLabel.setPixmap(Pixmap);
	ImageLabel.setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

	KeyPointsButton = new QPushButton(tr("Show Keypoints"), this);
	KeyPointsButton->setEnabled(false);
	KeyPointsButton->setFixedWidth(100);

	SaveButton = new QPushButton(tr("Save"), this);
	SaveButton->setFixedWidth(60);

	MatchButton = new QPushButton(tr("Overlapping Images"), this);
	MatchButton->setEnabled(false);
	MatchButton->setFixedWidth(120);

	connect(KeyPointsButton, &QPushButton::released, this, &CImageViewer::ShowKeyPoints);
	connect(SaveButton, &QPushButton::released, this, &CImageViewer::Save);
	connect(MatchButton, &QPushButton::released, this, &CImageViewer::ShowOverlappingWidget);

	Layout = new QVBoxLayout();
	Layout->addWidget(&ImageLabel);
	SubLayout = new QHBoxLayout();
	ButtonLayout = new QHBoxLayout();
	ButtonLayout->addSpacerItem(new QSpacerItem(20, 60, QSizePolicy::Expanding));
	ButtonLayout->addWidget(KeyPointsButton);
	ButtonLayout->addWidget(MatchButton);
	ButtonLayout->addWidget(SaveButton);
	SubLayout->addLayout(ButtonLayout);
	Layout->addLayout(SubLayout);
	setLayout(Layout);

	QFileInfo fileInfo(StdString2QString(ImagePath));
	setWindowTitle(fileInfo.fileName());
	connect(this, SIGNAL(EnableButton_SIGNAL()), this, SLOT(EnableButton()));

	std::thread DaemonThread(&CImageViewer::Daemon, this); //检测当前影像是否已完成"特征提取"和"特征匹配"
	DaemonThread.detach();

	resize(900, 700);
	setWindowIcon(QIcon(":/media/undistort.png"));
	show();
}
CImageViewer::CImageViewer(QWidget* parent, Database* database, image_t imageID) : parent(parent), database(database)
{
	const std::string imageDir = Database::imageDir;
	const std::string ImagePath = EnsureTrailingSlash(StringReplace(imageDir, "\\", "/")) + database->ReadImage(imageID).Name();
	this->ImagePath = ImagePath;

	if (!ExistsFile(ImagePath))
	{
		throw ImagePath + " does not exists!";
	}
	StopFlag = false;

	image = new QImage(StdString2QString(ImagePath));
	Pixmap = QPixmap::fromImage(*image).scaled(this->width(), this->height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
	ImageLabel.setScaledContents(true);
	ImageLabel.setPixmap(Pixmap);
	ImageLabel.setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

	KeyPointsButton = new QPushButton(tr("Show Keypoints"), this);
	KeyPointsButton->setEnabled(false);
	KeyPointsButton->setFixedWidth(100);

	SaveButton = new QPushButton(tr("Save"), this);
	SaveButton->setFixedWidth(60);

	MatchButton = new QPushButton(tr("Overlapping Images"), this);
	MatchButton->setEnabled(false);
	MatchButton->setFixedWidth(120);

	connect(KeyPointsButton, &QPushButton::released, this, &CImageViewer::ShowKeyPoints);
	connect(SaveButton, &QPushButton::released, this, &CImageViewer::Save);
	connect(MatchButton, &QPushButton::released, this, &CImageViewer::ShowOverlappingWidget);

	Layout = new QVBoxLayout();
	Layout->addWidget(&ImageLabel);
	SubLayout = new QHBoxLayout();
	ButtonLayout = new QHBoxLayout();
	ButtonLayout->addSpacerItem(new QSpacerItem(20, 60, QSizePolicy::Expanding));
	ButtonLayout->addWidget(KeyPointsButton);
	ButtonLayout->addWidget(MatchButton);
	ButtonLayout->addWidget(SaveButton);
	SubLayout->addLayout(ButtonLayout);
	Layout->addLayout(SubLayout);
	setLayout(Layout);

	QFileInfo fileInfo(StdString2QString(ImagePath));
	setWindowTitle(fileInfo.fileName());
	connect(this, SIGNAL(EnableButton_SIGNAL()), this, SLOT(EnableButton()));

	std::thread DaemonThread(&CImageViewer::Daemon, this); //检测当前影像是否已完成"特征提取"和"特征匹配"
	DaemonThread.detach();

	resize(900, 700);
	setWindowIcon(QIcon(":/media/undistort.png"));
	show();
}
void CImageViewer::EnableButton()
{
	if (KeyPointsButton && MatchButton && !StopFlag)
	{
		KeyPointsButton->setEnabled(true);
		MatchButton->setEnabled(true);
		OverlappingInfoWidget = new COverlappingInfoWidget(this, database, ImagePath);
	}
}
void CImageViewer::Daemon()
{
	const string ImgName = GetFileName(ImagePath);
	while (!StopFlag && (!database->ExistsImageWithName(ImgName) || !database->ExistsKeypoints(database->ReadImageWithName(ImgName).ImageId()) || !image))
	{
		this_thread::sleep_for(chrono::milliseconds(500));
	}
	if (!StopFlag)
	{
		imageWithKeypoints = new QImage(StdString2QString(ImagePath));
		size_t ImageID = database->ReadImageWithName(ImgName).ImageId();
		FeatureKeypoints Keypoints = database->ReadKeypoints(ImageID);

		QPainter Painter(imageWithKeypoints);
		Painter.setRenderHint(QPainter::Antialiasing, true);
		Painter.setPen(QPen(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		for (FeatureKeypoint& Keypoint : Keypoints)
		{
			Painter.drawEllipse(QPoint(Keypoint.x, Keypoint.y), 3, 3);
		}
	}
	//while (!StopFlag && CDatabase::MatchedImagesNum(ImageID) == 0) //只有当前影像的状态是Matched, Reconstructing, Finished的时候才退出循环
	//{
	//	this_thread::sleep_for(chrono::milliseconds(500));
	//}
	if (!StopFlag)
	{
		emit EnableButton_SIGNAL();
	}
}
void CImageViewer::ShowKeyPoints()
{
	if (KeyPointsButton->text() == tr("Show Keypoints")) //当前显示的影像是"原影像", 需要显示"带特征点的影像"
	{
		if (imageWithKeypoints)
		{
			Pixmap = QPixmap::fromImage(*imageWithKeypoints).scaled(width(), height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
			ImageLabel.setScaledContents(true);
			ImageLabel.setPixmap(Pixmap);
			KeyPointsButton->setText(tr("Hide Keypoints"));
		}
	}
	else //当前显示的影像是"带特征点的影像", 需要显示"原影像"
	{
		if (image)
		{
			Pixmap = QPixmap::fromImage(*image).scaled(width(), height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
			ImageLabel.setScaledContents(true);
			ImageLabel.setPixmap(Pixmap);
			KeyPointsButton->setText(tr("Show Keypoints"));
		}
	}
}
void CImageViewer::Save()
{
	QString filter("BMP (*.bmp)");
	const QString SavePath = QFileDialog::getSaveFileName(this, tr("Select destination..."), "", "PNG (*.png);;JPEG (*.jpg);;BMP (*.bmp)", &filter).toUtf8().constData();
	if (SavePath == "")
	{
		return;
	}
	if (KeyPointsButton->text() == tr("Show Keypoints"))
	{
		if (image)
		{
			image->save(SavePath, nullptr, 100);
		}
	}
	else
	{
		if (imageWithKeypoints)
		{
			imageWithKeypoints->save(SavePath, nullptr, 100);
		}
	}
}
void CImageViewer::ShowOverlappingWidget()
{
	OverlappingInfoWidget = new COverlappingInfoWidget(this, database, ImagePath);
	OverlappingInfoWidget->show();
}
void CImageViewer::resizeEvent(QResizeEvent* event)
{
	if (KeyPointsButton->text() == tr("Show Keypoints"))
	{
		if (image)
		{
			Pixmap = QPixmap::fromImage(*image).scaled(ImageLabel.width(), ImageLabel.height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
			ImageLabel.setScaledContents(true);
			ImageLabel.setPixmap(Pixmap);
		}
	}
	else
	{
		if (imageWithKeypoints)
		{
			Pixmap = QPixmap::fromImage(*imageWithKeypoints).scaled(ImageLabel.width(), ImageLabel.height(), Qt::KeepAspectRatioByExpanding, Qt::FastTransformation);
			ImageLabel.setScaledContents(true);
			ImageLabel.setPixmap(Pixmap);
		}
	}
}
void CImageViewer::closeEvent(QCloseEvent* event)
{
	StopFlag = true;
	delete image;
	delete imageWithKeypoints;
	Layout->deleteLater();
}















