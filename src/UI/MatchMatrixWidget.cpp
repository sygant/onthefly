#include "MatchMatrixWidget.h"
#include <QPainter>
#include "../Feature/Bitmap.h"
#include <QtWidgets>

using namespace std;

unordered_map<image_pair_t, size_t> MatchMatrixWidget::matchMatrix;
size_t MatchMatrixWidget::maxNumMatches = 0;
MatchMatrixWidget::MatchMatrixWidget(QWidget* parent, Database* database)
{
	CHECK(database);
	this->parent = parent;
	this->database = database;
	connect(&updateTimer, SIGNAL(timeout()), this, SLOT(update()));
}
void MatchMatrixWidget::SaveMatchMatrix(const std::string& filePath, Database* database)
{
	CHECK(database);
	const size_t imageNum = database->NumImages();

	vector<pair<image_t, image_t>> imagePairs(0);
	vector<int> numInliers(0);
	database->ReadTwoViewGeometryNumInliers(&imagePairs, &numInliers);
	for (int i = 0; i < imagePairs.size(); i++)
	{
		const image_pair_t imagePairID = Database::ImagePairToPairId(imagePairs[i].first, imagePairs[i].second);
		const size_t numInlierMatches = numInliers[i];
		matchMatrix[imagePairID] = numInlierMatches;
		maxNumMatches = max(maxNumMatches, numInlierMatches);
	}

	const size_t gridSize = (imageNum < 200 ? 2000 / imageNum : 10);
	QPixmap pixmap(gridSize * imageNum, gridSize * imageNum);
	QPainter pixmapPainter(&pixmap);
	pixmap.fill(Qt::white);

	QColor lastColor;
	bool isColorSet = false;
	for (size_t i = 1; i <= imageNum; i++)
	{
		for (size_t j = 1; j <= imageNum; j++)
		{
			const size_t value = GetValue(i, j);
			const QColor color = GetColor(value);
			if (!isColorSet || color != lastColor)
			{
				pixmapPainter.setBrush(color);
				lastColor = color;
				isColorSet = true;
			}
			pixmapPainter.drawRect((i - 1) * gridSize, (j - 1) * gridSize, gridSize, gridSize);
		}
	}
	const QString saveImagePath = QString::fromStdString(filePath);
	pixmap.save(saveImagePath, 0, 100);
}
void MatchMatrixWidget::showEvent(QShowEvent* event)
{
	updateTimer.start(2000);
	show();
	paintEvent(nullptr);
}
void MatchMatrixWidget::paintEvent(QPaintEvent* event)
{
	const size_t imageNum = database->NumImages();
	if (imageNum == 0)
	{
		return;
	}
	lock_guard<mutex> lock(drawingMutex);

	vector<pair<image_t, image_t>> imagePairs(0);
	vector<int> numInliers(0);
	database->ReadTwoViewGeometryNumInliers(&imagePairs, &numInliers);
	for (int i = 0; i < imagePairs.size(); i++)
	{
		const image_pair_t imagePairID = Database::ImagePairToPairId(imagePairs[i].first, imagePairs[i].second);
		const size_t numInlierMatches = numInliers[i];
		matchMatrix[imagePairID] = numInlierMatches;
		maxNumMatches = max(maxNumMatches, numInlierMatches);
	}

	const size_t width = this->width();
	const size_t height = this->height();
	const size_t minLength = min(width, height);

	if (minLength / imageNum > 10)
	{
		const size_t gridSize = minLength / imageNum;
		QPixmap pixmap(minLength, minLength);
		QPainter pixmapPainter(&pixmap);
		pixmap.fill(Qt::white);

		QColor lastColor;
		bool isColorSet = false;

		for (size_t i = 1; i <= imageNum; i++)
		{
			for (size_t j = 1; j <= imageNum; j++)
			{
				const size_t value = GetValue(i, j);
				const QColor color = GetColor(value);
				if (!isColorSet || color != lastColor)
				{
					pixmapPainter.setBrush(color);
					lastColor = color;
					isColorSet = true;
				}
				pixmapPainter.drawRect((i - 1) * gridSize, (j - 1) * gridSize, gridSize, gridSize);
			}
		}
		QPainter widgetPainter(this);
		widgetPainter.drawPixmap(0, 0, pixmap);
	}
	else
	{
		QImage image(imageNum, imageNum, QImage::Format_RGB32);
		image.fill(Qt::white);
		for (int y = 0; y < image.height(); y++)
		{
			for (int x = 0; x < image.width(); x++)
			{
				const image_t imageID1 = y + 1;
				const image_t imageID2 = x + 1;
				const size_t value = GetValue(imageID1, imageID2);
				const QColor color = GetColor(value);
				image.setPixel(x, y, color.rgb());
			}
		}
		QPixmap pixmap = QPixmap::fromImage(image);
		QPainter widgetPainter(this);
		widgetPainter.drawPixmap(0, 0, pixmap);
	}
}
void MatchMatrixWidget::closeEvent(QCloseEvent* event)
{
	updateTimer.stop();
	hide();
}
size_t MatchMatrixWidget::GetValue(image_t i, image_t j)
{
	if (i == j)
	{
		return 0;
	}
	const image_pair_t imagePairID = Database::ImagePairToPairId(i, j);
	const auto it = matchMatrix.find(imagePairID);
	if (it == matchMatrix.end())
	{
		return 0;
	}
	return it->second;
}
QColor MatchMatrixWidget::GetColor(size_t value)
{
	if (value == 0)
	{
		return Qt::white;
	}
	QColor Color;
	double value2 = log1p(value) / log1p(maxNumMatches);
	Color.setRed(255 * JetColormap::Red(value2));
	Color.setGreen(255 * JetColormap::Green(value2));
	Color.setBlue(255 * JetColormap::Blue(value2));
	return Color;
}

ShowMatchMatrixWidget::ShowMatchMatrixWidget(QWidget* parent, Database* database) :QWidget(parent)
{
	CHECK(database);
	setWindowTitle(tr("match matrix"));
	setWindowFlags(Qt::Window);
	setWindowIcon(QIcon(":/media/match-matrix.png"));
	this->parent = parent;
	matchMatrix = new MatchMatrixWidget(parent, database);
	saveButton = new QPushButton(tr("Save"), this);
	connect(saveButton, &QPushButton::released, this, &ShowMatchMatrixWidget::Save);

	QGridLayout* layout = new QGridLayout(this);
	layout->addWidget(matchMatrix, 0, 0, 1, 2);
	layout->addWidget(saveButton, 1, 1, 1, 1);
	layout->setRowStretch(0, 1);
	layout->setColumnStretch(0, 1);
	setLayout(layout);
	setMinimumSize(500, 500);
}
void ShowMatchMatrixWidget::showEvent(QShowEvent* event)
{
	matchMatrix->show();
	this->show();
}
void ShowMatchMatrixWidget::closeEvent(QCloseEvent* event)
{
	matchMatrix->close();
	hide();
}
void ShowMatchMatrixWidget::resizeEvent(QResizeEvent* event)
{
	matchMatrix->repaint();
	QWidget::resizeEvent(event);
}
void ShowMatchMatrixWidget::Save()
{
	QString filename = QFileDialog::getSaveFileName(this, "Export Matching Relationship Matrix", "", "BMP Files (*.bmp);;JPEG Files (*.jpg *.jpeg);;All Files (*)");
	if (!filename.isEmpty())
	{
		MatchMatrixWidget::SaveMatchMatrix(filename.toStdString(), database);
	}
}

