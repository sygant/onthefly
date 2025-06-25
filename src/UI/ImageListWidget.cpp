#include "ImageListWidget.h"
#include <iostream>
#include "../Base/misc.h"
#include "../Base/macros.h"
#include "ImageViewer.h"
using namespace std;

//QString转string
std::string QString2StdString(const QString& QString)
{
	return std::string(QString.toLocal8Bit());
}

//string转QString
QString StdString2QString(const std::string& string)
{
	return QString::fromLocal8Bit(string.data());
}

ImageListWidget::ImageListWidget(QDockWidget* parent, Database* database) : QListWidget(parent)
{
	this->parent = parent;
	this->database = database;
	items.clear();
	imagePaths.clear();
	parent->setWindowTitle(tr("Image list"));

	setSelectionMode(QAbstractItemView::SingleSelection);
	setViewMode(QListWidget::IconMode);
	setIconSize(QSize(180, 180));
	setSpacing(2);
	setResizeMode(QListView::Adjust);
	setMovement(QListView::Static);
	connect(this, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(itemDoubleClicked_SLOT(QListWidgetItem*)));

	changeCount = 0;
}
ImageListWidget::~ImageListWidget()
{
	ClearImage();
}
void ImageListWidget::AddImage(const std::string& imagePath)
{
	lock_guard<mutex> lock(imageListMutex);
	__try
	{
		QScrollBar* scrollbar = verticalScrollBar();
		int currentValue = scrollbar ? scrollbar->value() : 0;
		int maxValue = scrollbar ? scrollbar->maximum() : 0;

		QString QImgPath = QString::fromLocal8Bit(imagePath.data());

		//为了加快读取速度并且降低内存占用, 影像预览并不读取原图, 而是使用ImageReader以图标(Icon)的形式直接读取设定大小的影像图标
		QImageReader ImageReader(QImgPath);
		ImageReader.setAutoTransform(true);
		QSize OriginSize = ImageReader.size();
		QSize TargetSize = OriginSize.scaled(CalculateImageSize(OriginSize), Qt::KeepAspectRatio);
		ImageReader.setScaledSize(TargetSize);
		if (!ImageReader.canRead()) //影像读取失败, 影像可能是损坏的
		{
			cout << "[Warning] Image [ " + imagePath + " ] is damaged!" << endl;
			return;
		}
		QListWidgetItem* NewItem = new QListWidgetItem();
		NewItem->setIcon(QIcon(QPixmap::fromImageReader(&ImageReader)));
		NewItem->setText(StdString2QString(GetFileName(imagePath)));
		TargetSize.setHeight(TargetSize.height() + 10);
		NewItem->setSizeHint(TargetSize);
		NewItem->setData(Qt::UserRole, StdString2QString(GetFileName(imagePath)));
		addItem(NewItem);
		items.push_back(NewItem);
		imagePaths.push_back(imagePath);
		parent->setWindowTitle(tr("Image list ") + "(" + QString::number(items.size()) + ")");

#ifdef DEMO_MODE
		changeCount++;
		if (count() > 2 && verticalScrollBar())
		{
			if (changeCount >= 2)
			{
				setCurrentRow(count() - 1);
				changeCount = 0;
			}
		}
#endif
//#ifndef IMAGE_LIST_NOT_FOLLOW
//		changeCount++;
//		if (count() > 2 && verticalScrollBar())
//		{
//			if (changeCount >= 2)
//			{
//				setCurrentRow(count() - 1);
//				changeCount = 0;
//			}
//		}
//#endif
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		cout << "listwidget error!" << endl;
	}
}
void ImageListWidget::ClearImage()
{
	clear();
	items.clear();
}
void ImageListWidget::itemDoubleClicked_SLOT(QListWidgetItem* item)
{
	//弹出"影像浏览"窗口
	QListWidgetItem* currentItem = this->currentItem();
	if (currentItem)
	{
		size_t index = this->row(currentItem);
		CImageViewer* ImageViewer = new CImageViewer(this, database, imagePaths[index]);
	}
}
QSize ImageListWidget::CalculateImageSize(QSize& OriginSize)
{
	//计算影像图标合适的大小
	float dWidth = OriginSize.width() / 200.0;
	float dHeight = OriginSize.height() / 200.0;
	float Shrink = max(dHeight, dWidth);
	QSize dsize(OriginSize.width() / Shrink, OriginSize.height() / Shrink);
	return dsize;
}



