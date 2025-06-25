#pragma once
#include <QtCore>
#include <QtWidgets>

class ProjectWidget : public QWidget
{
	Q_OBJECT
public:
	ProjectWidget(QWidget* parent);

	bool IsValid() const;
	void Reset();

	std::string GetDatabasePath() const;
	std::string GetImagePath() const;
	void SetDatabasePath(const std::string& path);
	void SetImagePath(const std::string& path);

signals:
	void NewProject_SIGNAL();

private:
	void Save();
	void SelectNewDatabasePath();
	void SelectExistingDatabasePath();
	void SelectImagePath();
	QString DefaultDirectory();

	// Whether file dialog was opened previously.
	bool prev_selected_;

	// Text boxes that hold the currently selected paths.
	QLineEdit* database_path_text_;
	QLineEdit* image_path_text_;
};