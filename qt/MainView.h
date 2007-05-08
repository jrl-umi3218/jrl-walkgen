//
//    File: mview.h
//
//    (C) 2000 Helmut Cantzler
//
//    Licensed under the terms of the Lesser General Public License.
//

#ifndef _MVIEW_H
#define _MVIEW_H

#include <qmainwindow.h>
#include <qworkspace.h>
#include <qtoolbutton.h>
#include <qtimer.h>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include <PatternGeneratorInterface.h>
#include <HRP2Display.h>
#include <joystick.h>

class QMultiLineEdit;
class QToolBar;
class QPopupMenu; 
class QSlider;
class QLabel;

class MainView: public QMainWindow
{
  Q_OBJECT
 public:
    MainView(int argc, char **argv);
    ~MainView();

 signals:
    void sendMotors(double *);
    void sendWaistPosture(double *);
    void sendMotorsAndWaistPosture(double *,double *);

 public slots:
    void togglePick();
    void toggleTextureMapping();
    void timerEvent();
    void setTexture();

 private slots:
    void load_file();
    void InitialPosition();
    void InitialPosition(QString s);
    void ToZMPStack();
    void StopOnLine();
    void StartOnLine();

    void save_as(int item);
    void takeShot(int item);

    void removeDoublePoints();
    void negateNormals();

    void setSolid();
    void setWire();
    void setFrontlines();
    void setPoints();
    void setFeatures();
    void toggleLighting();
    void toggleShapeColors();
    void toggleSurfaceNormals();
    void toggleCutBackFaces();
    void toggleBilinearFiltering();
    void toggleFeatures();
    void toggleAspectRatio();
    void setBackgroundColor();

    void reset_view();
    void set_view();
    void save_view();
    void generate_view();
    void clear_views();
    void load_view(int id);
    void GenerateImageFromViews();
    void about();
    void aboutQt();

    void LoadCameraParameters();
    void SimulateProjection();
    void HRP2Model();
    void MotorValues();
    void ComputeAPostureFromEulerAngles(PatternGeneratorJRL::COMPosition &aPosition,
					double *Posture);

 protected:


    // Multiple document workspace widget 
    QWorkspace * m_Workspace;

    // Window for display the HRP2 robot.
    HRP2Display * m_HD;

 private:
    void set_status();

    QString name;
    QToolButton *pick, *text;
    QSlider *clipping;
    QPopupMenu *save, *screenshot, *modes, *viewpoint, *CameraParameters;
    int StartingIDForGeneratedViews;
    
    int *ids, displayMode;

    QTimer * m_timer;
    PatternGeneratorJRL::PatternGeneratorInterface * m_PGI;
    Joystick * m_Joystick;
    MAL_MATRIX(UpperBodyAngles,double);
};

#endif
