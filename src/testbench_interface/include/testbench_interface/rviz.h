#ifndef __RVIZ_H_
#define __RVIZ_H_

#include <QWidget>
// #include <QColor>
// #include <QSlider>
// #include <QLabel>
// #include <QGridLayout>
#include <QVBoxLayout>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/view_manager.h>

class Rvizviewer: public QVBoxLayout
{
Q_OBJECT
public:
    Rvizviewer(QVBoxLayout* parent);
    ~Rvizviewer();
  
private:
    rviz::RenderPanel* rvizPanel;// = new rviz::RenderPanel;
    rviz::VisualizationManager* rvizManager;// =  new rviz::VisualizationManager(pointCloud_panel);
};


#endif // __RVIZ_H_