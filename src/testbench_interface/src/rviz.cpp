#include "testbench_interface/mainwindow.h"

Rvizviewer::Rvizviewer(QVBoxLayout* parent){
    rvizPanel = new rviz::RenderPanel;
    rvizManager =  new rviz::VisualizationManager(rvizPanel);
//
//
    rvizPanel->initialize(rvizManager->getSceneManager(), rvizManager);
    rvizPanel->setBackgroundColor( Ogre::ColourValue(0, 0,0,0.3)); //no use
    parent->addWidget(rvizPanel);
//
//    rvizManager->initialize();
//    rvizManager->startUpdate();
//    rvizManager->setFixedFrame("imu_link");
//
//    rviz::Display *grid = rvizManager->createDisplay("rviz/Grid","Grid",true);
//
//    viewManager = rvizManager->getViewManager();
//    viewManager->setRenderPanel(rvizPanel);
//    viewManager->setCurrentViewControllerType("rviz/Orbit");
//    viewManager->getCurrent()->subProp("Target Frame")->setValue("/aft_mapped");
//
//    rviz::Display *pointCloud = rvizManager->createDisplay("rviz/PointCloud2","PointCloud2", true);
//    pointCloud->subProp("Topic")->setValue("/kitti/velo/pointcloud");
//    pointCloud->subProp("Style")->setValue("Points");
//    pointCloud->subProp("Size (Pixels)")->setValue("1");
//    pointCloud->subProp("Color Transformer")->setValue("FlatColor");
//    pointCloud->subProp("Invert Rainbow")->setValue("true");
}

Rvizviewer::~Rvizviewer(){
    delete rvizPanel;
    delete rvizManager;
}