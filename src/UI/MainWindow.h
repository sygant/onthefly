#include <QMainWindow>
#include "ImageListWidget.h"
#include "LogWidget.h"
#include "ModelViewerWidget.h"
#include "ReconstructionManagerWidget.h"
#include "ProjectWidget.h"
#include "MatchMatrixWidget.h"
#include "../Workflow/Receiver.h"
#include "../Workflow/Workflow.h"

#include <QMenu>
#include <QToolBar>
#include <QAction>

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget* parent = nullptr);
	void RunFunc();

private:
	LogWidget* logWidget = nullptr;
	ReconstructionManagerWidget* reconstructionManagerWidget = nullptr;
	ImageListWidget* imageListWidget = nullptr;
	ProjectWidget* projectWidget = nullptr;
	ModelViewerWidget* modelViewerWidget = nullptr;
	ShowMatchMatrixWidget* showMatchMatrixWidget = nullptr;
	QLabel* statusLabel = nullptr;
	QDockWidget* logWindowDock = nullptr;
	QDockWidget* imageListWindowDock = nullptr;
	QCheckBox* isTrackModelCheckBox = nullptr;;

	QMenu* settingMenu = nullptr;
	QMenu* projectMenu = nullptr;
	QMenu* transmitMenu = nullptr;
	QMenu* toolsMenu = nullptr;
	QToolBar* settingToolBar = nullptr;
	QToolBar* projectToolBar = nullptr;
	QToolBar* transmitToolBar = nullptr;
	QToolBar* reconstructToolBar = nullptr;
	QToolBar* toolsToolBar = nullptr;

	Database* database = nullptr;
	Reconstructor* reconstructor = nullptr;
	ReconstructionManager* reconstructionManager = nullptr;
	Receiver* receiver = nullptr;


	QAction* newProjectAction = nullptr;                   // 弹出"创建新工程"对话框
	QAction* startReceiveAction = nullptr;                 // 开始传输
	QAction* stopReceiverAction = nullptr;                 // 停止传输
	QAction* renderNowAction = nullptr;                    // 渲染模型
	QAction* exportProjectAction = nullptr;                // 导出工程
	QAction* showMatchMatrixAction = nullptr;

	std::mutex renderMutex;
	bool isForcedRender = false;

	void StartReceive();
	void ProcessImage(const std::string& newImagePath);
	void StopReceive();

	void closeEvent(QCloseEvent* event) override;
	void SetupWidgets();
	void SetupActions();
	void CreateMenus();
	void CreateToolbar();
	void CreateStatusbar();
	void ChangeCurrentModel();
	void ShowMatchMatrix();

	void NewProject();
	void ExportProject();
	void ChangeSelectModel();
	void RenderNow();

	void NewProject_SLOT();


};









