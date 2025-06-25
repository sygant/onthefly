#include "MainWindow.h"
#include "../Base/misc.h"
#include "../Base/macros.h"

#include <QSettings>
#include <QString>
#include <filesystem>

#include <algorithm>
#include <random>
#include <utility>

using namespace std;
std::string ToLowerCase(const std::string& str) {
	std::string lowerCaseStr = str;
	std::transform(lowerCaseStr.begin(), lowerCaseStr.end(), lowerCaseStr.begin(),
		[](unsigned char c) { return std::tolower(c); });
	return lowerCaseStr;
}
std::vector<std::string> GetImageFileList(const std::string& path)
{
	std::vector<std::string> fileList;
	std::filesystem::path dirPath(path);
	const std::vector<std::string> imageExtensions = { ".jpg", ".jpeg", ".png", ".bmp", ".gif" };

	if (std::filesystem::exists(dirPath) && std::filesystem::is_directory(dirPath)) {
		for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
			if (entry.is_regular_file()) {
				std::filesystem::path filePath = entry.path();
				std::string extension = ToLowerCase(filePath.extension().string());
				if (std::find_if(imageExtensions.begin(), imageExtensions.end(), [&extension](const std::string& ext) {
					return extension == ext;
					}) != imageExtensions.end()) {
					fileList.push_back(filePath.string());
				}
			}
		}
	}

	return fileList;
}
//string转QString
QString StdString2QString2(std::string string)
{
	return QString::fromLocal8Bit(string.data());
}
int Add(int currentTime) 
{
	int hours = currentTime / 10000; // 提取小时部分
	int minutes = (currentTime / 100) % 100; // 提取分钟部分
	int seconds = currentTime % 100; // 提取秒部分

	seconds++; // 增加一秒

	// 检查秒是否达到60
	if (seconds == 60) {
		seconds = 0;
		minutes++; // 分钟进位
		// 检查分钟是否达到60
		if (minutes == 60) {
			minutes = 0;
			hours++; // 小时进位
			// 检查小时是否超过23
			if (hours == 24) {
				hours = 0; // 重新开始一个新的一天
			}
		}
	}

	// 重新组合小时、分钟和秒
	int newTime = hours * 10000 + minutes * 100 + seconds;
	return newTime;
}


void MainWindow::RunFunc()
{
	const std::string baseImagePath = Database::imageDir;
	const vector<std::string> images = GetImageFileList(baseImagePath);

	//ADD-24-07-14:随机打乱影像的输入顺序
	/*std::vector<std::string> shuffled_images = images;
	std::random_device rd;
	std::mt19937 generator(rd());
	std::shuffle(shuffled_images.begin(), shuffled_images.end(), generator);*/

	QSettings settings("HKEY_CURRENT_USER\\Software\\RTPS\\CamFiTransmit", QSettings::NativeFormat);
	for (int i = 0; i < images.size(); i++)
	{
		const QString imagePath = StdString2QString2(images[i]);
		settings.setValue("NewImage", imagePath);

		if (i <= 20)
		{
			this_thread::sleep_for(chrono::milliseconds(2000));
		}
		else
		{
			this_thread::sleep_for(chrono::milliseconds(4000));
		}
	}
}

MainWindow::MainWindow(QWidget* parent) :QMainWindow(parent)
{
	receiver = new Receiver([&](const std::string& value) {
		ProcessImage(value);
		});
	database = new Database();
	reconstructionManager = new ReconstructionManager();
	reconstructor = new Reconstructor(database, reconstructionManager);
	CWorkflow::Initialize(database, reconstructor);


	SetupWidgets();
	SetupActions();
	CreateMenus();
	CreateToolbar();
	CreateStatusbar();
}
void MainWindow::StartReceive()
{
	receiver->Start();
	startReceiveAction->setEnabled(false);
	stopReceiverAction->setEnabled(true);

#ifdef AUTO_RUN_IMAGES
	std::thread experimentThread(&MainWindow::RunFunc, this);
	experimentThread.detach();
#endif
}
void MainWindow::ProcessImage(const std::string& newImagePath)
{
	std::cout << "New image: " << newImagePath << std::endl;
	imageListWidget->AddImage(newImagePath);
	CWorkflow::AddImageProcess(newImagePath);
}
void MainWindow::StopReceive()
{
	receiver->Stop();
	startReceiveAction->setEnabled(true);
	stopReceiverAction->setEnabled(false);

	std::thread tryMergeThread([&]() {
		reconstructor->TryMergeModels();
		reconstructor->FinalGlobalBA();
		});
	tryMergeThread.detach();
}
void MainWindow::closeEvent(QCloseEvent* event)
{
	CWorkflow::Stop();
	receiver->Stop();
	event->accept();
}
void MainWindow::SetupWidgets()
{
	resize(600, 600);
	setWindowTitle(tr("On-the-fly SfM"));
	setWindowIcon(QIcon(":/media/WindowIcon.png"));

	modelViewerWidget = new ModelViewerWidget(this);
	modelViewerWidget->SetDatabase(database);
	setCentralWidget(modelViewerWidget);

	reconstructionManagerWidget = new ReconstructionManagerWidget(this, std::shared_ptr<const ReconstructionManager>(reconstructionManager));
	reconstructionManagerWidget->setFixedWidth(270);
	reconstructionManagerWidget->Update();
	connect(reconstructionManagerWidget, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainWindow::ChangeSelectModel);

	imageListWindowDock = new QDockWidget(tr("Image Preview"), this);
	imageListWindowDock->setMinimumHeight(100);
	imageListWindowDock->setMinimumWidth(250);
	imageListWidget = new ImageListWidget(imageListWindowDock, database);
	imageListWindowDock->setWidget(imageListWidget);
	addDockWidget(Qt::RightDockWidgetArea, imageListWindowDock);

	logWindowDock = new QDockWidget(tr("Log"), this);
	logWidget = new LogWidget(this);
	logWindowDock->setWidget(logWidget);
	addDockWidget(Qt::BottomDockWidgetArea, logWindowDock);
	logWindowDock->show();
	logWindowDock->raise();

	projectWidget = new ProjectWidget(this);
	connect(projectWidget, &ProjectWidget::NewProject_SIGNAL, this, &MainWindow::NewProject_SLOT);

	isTrackModelCheckBox = new QCheckBox(tr("Track Workflow"));
	isTrackModelCheckBox->setChecked(true);

	showMatchMatrixWidget = new ShowMatchMatrixWidget(this, database);
}
void MainWindow::SetupActions()
{
	newProjectAction = new QAction(QIcon(":/media/project-new.png"), tr("New Project"), this);
	connect(newProjectAction, &QAction::triggered, this, &MainWindow::NewProject);

	exportProjectAction = new QAction(QIcon(":/media/project-save-as.png"), tr("Export Project"), this);
	connect(exportProjectAction, &QAction::triggered, this, &MainWindow::ExportProject);
	exportProjectAction->setEnabled(false);

	startReceiveAction = new QAction(QIcon(":/media/reconstruction-start.png"), tr("Start Receive"), this);
	connect(startReceiveAction, &QAction::triggered, this, &MainWindow::StartReceive);

	stopReceiverAction = new QAction(QIcon(":/media/reconstruction-pause.png"), tr("Stop Receive"), this);
	connect(stopReceiverAction, &QAction::triggered, this, &MainWindow::StopReceive);
	stopReceiverAction->setEnabled(false);

	renderNowAction = new QAction(this);
	connect(renderNowAction, &QAction::triggered, this, &MainWindow::RenderNow);

	reconstructor->AddCallback(
		Reconstructor::INITIAL_IMAGE_PAIR_REG_CALLBACK, [this]() {
			renderNowAction->trigger();
		});
	reconstructor->AddCallback(
		Reconstructor::NEXT_IMAGE_REG_CALLBACK, [this]() {
			renderNowAction->trigger();
		});
	reconstructor->AddCallback(
		Reconstructor::NEXT_IMAGE_REG_CALLBACK, [this]() {
			renderNowAction->trigger();
		});
	reconstructor->AddCallback(
		Reconstructor::CHANGE_CURRENT_MODEL_CALLBACK, [this]() {
			ChangeCurrentModel();
		});

	showMatchMatrixAction = new QAction(QIcon(":/media/match-matrix.png"), tr("Show Match Matrix"), this);
	connect(showMatchMatrixAction, &QAction::triggered, this, &MainWindow::ShowMatchMatrix);

}
void MainWindow::CreateMenus()
{
	projectMenu = new QMenu(tr("Project"), this);
	projectMenu->addAction(newProjectAction);
	projectMenu->addAction(exportProjectAction);
	menuBar()->addAction(projectMenu->menuAction());

	transmitMenu = new QMenu(tr("Transmit"), this);
	transmitMenu->addAction(startReceiveAction);
	transmitMenu->addAction(stopReceiverAction);
	menuBar()->addAction(transmitMenu->menuAction());

}
void MainWindow::CreateToolbar()
{
	projectToolBar = addToolBar(tr("Project"));
	projectToolBar->addAction(newProjectAction);
	projectToolBar->addAction(exportProjectAction);
	projectToolBar->setIconSize(QSize(16, 16));
	projectToolBar->setMovable(false);

	transmitToolBar = addToolBar(tr("Transmit"));
	transmitToolBar->addAction(startReceiveAction);
	transmitToolBar->addAction(stopReceiverAction);
	transmitToolBar->setIconSize(QSize(16, 16));
	transmitToolBar->setMovable(false);

	reconstructToolBar = addToolBar(tr("Reconstruct"));
	reconstructToolBar->addAction(showMatchMatrixAction);
	reconstructToolBar->addWidget(reconstructionManagerWidget);
	reconstructToolBar->setIconSize(QSize(16, 16));
	reconstructToolBar->addSeparator();
	reconstructToolBar->addWidget(isTrackModelCheckBox);

	reconstructToolBar->setMovable(false);
}
void MainWindow::CreateStatusbar()
{
	QFont font;
	font.setPointSize(11);

	modelViewerWidget->statusbar_status_label = new QLabel("0 Images - 0 Points", this);
	modelViewerWidget->statusbar_status_label->setFont(font);
	modelViewerWidget->statusbar_status_label->setAlignment(Qt::AlignCenter);
	statusBar()->addWidget(modelViewerWidget->statusbar_status_label, 1);
}
void MainWindow::ChangeCurrentModel()
{
	if (isTrackModelCheckBox->isChecked())
	{
		lock_guard<mutex> lock(renderMutex);
		const int lastRegModelID = Reconstructor::lastRegModelID;
		if (lastRegModelID >= 0 && lastRegModelID < reconstructionManager->Size())
		{
			reconstructionManagerWidget->Update();
			reconstructionManagerWidget->SelectReconstruction(lastRegModelID);
			renderNowAction->trigger();
		}
	}
	else
	{
		reconstructionManagerWidget->Update();
		renderNowAction->trigger();
	}
}
void MainWindow::ShowMatchMatrix()
{
	if (showMatchMatrixWidget)
	{
		showMatchMatrixWidget->show();
	}
}
void MainWindow::NewProject()
{
	projectWidget->show();
}
void MainWindow::ExportProject()
{
	std::thread exportProjectThread([&]() {
		const std::string basePath = StringReplace(EnsureTrailingSlash(GetParentDir(Database::exportPath)), "\\", "/");
		if (database)
		{
			cout << "Exporting database..." << endl;
			database->Export(Database::exportPath);
			cout << "Export database completed!" << endl;
		}
		if (reconstructionManager)
		{
			cout << "Exporting models..." << endl;
			for (int i = 0; i < reconstructionManager->Size(); i++)
			{
				const std::string modelExportDir = basePath + "models/" + to_string(i + 1);
				CreateDirIfNotExists(modelExportDir + "/bin", true);
				CreateDirIfNotExists(modelExportDir + "/text", true);
				reconstructionManager->Get(i)->WriteBinary(modelExportDir + "/bin");
				reconstructionManager->Get(i)->WriteText(modelExportDir + "/text");
				reconstructionManager->Get(i)->OutputDebugResult(modelExportDir);
			}
			cout << "Export models completed!" << endl;
		}
		logWidget->SaveLogToFile(basePath + "Log.txt");
		CWorkflow::OutputTimeFiles(basePath);
		MatchMatrixWidget::SaveMatchMatrix(basePath + "match_matrix.bmp", database);
		});
	exportProjectThread.detach();
}
void MainWindow::ChangeSelectModel()
{
	isForcedRender = true;
	RenderNow();
	isForcedRender = false;
}
void MainWindow::RenderNow()
{
#ifdef DISABLE_MODELVIEWER
	return;
#endif
	lock_guard<mutex> lock(renderMutex);
	__try
	{
		if (reconstructionManager->Size() == 0)
		{
			reconstructionManagerWidget->SelectReconstruction(ReconstructionManagerWidget::kNewestReconstructionIdx);
			modelViewerWidget->ClearReconstruction();
			return;
		}
		reconstructionManagerWidget->Update();
		size_t reconstruction_idx = reconstructionManagerWidget->SelectedReconstructionIdx();
		if (reconstruction_idx == ReconstructionManagerWidget::kNewestReconstructionIdx)
		{
			if (reconstructionManager->Size() > 0)
			{
				reconstruction_idx = reconstructionManager->Size() - 1;
			}
		}
		if (isTrackModelCheckBox->isChecked() || reconstruction_idx == reconstructor->lastRegModelID || isForcedRender)
		{
			modelViewerWidget->reconstruction = reconstructionManager->Get(reconstruction_idx);
			modelViewerWidget->ReloadReconstruction();
		}
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		std::cout << "RenderNow unknown exception caught" << endl;
	}
}
void MainWindow::NewProject_SLOT()
{
	exportProjectAction->setEnabled(true);
}








