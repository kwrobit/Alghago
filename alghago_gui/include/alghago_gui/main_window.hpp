/**
 * @file /include/alghago_gui/main_window.hpp
 *
 * @brief Qt based gui for alghago_gui.
 *
 * @date November 2010
 **/
#ifndef alghago_gui_MAIN_WINDOW_H
#define alghago_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace alghago_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );
    void on_button_user_first_clicked();
    void on_button_robot_first_clicked();
    void on_button_think_done_clicked();
    void on_button_user_done_clicked();
    void on_button_shoot_done_clicked();
    void on_button_calibrate_video_clicked();
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateState();
    void updateBadukpanImage();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    cv::Mat badukpan_image;
};

}  // namespace alghago_gui

#endif // alghago_gui_MAIN_WINDOW_H
