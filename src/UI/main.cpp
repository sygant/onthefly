#include "MainWindow.h"


int main(int argc, char* argv[])
{
	QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

	QApplication App(argc, argv);
	QTranslator translator;
	translator.load("D:/CS/study/On_the_fly_SfME/src/UI/UITranslation_zh_CN.qm");
	App.installTranslator(&translator);

	MainWindow w;
	w.show();
	return App.exec();
}