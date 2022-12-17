#include <rw/invkin.hpp>
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/core/PropertyMap.hpp>
// #include <rwlibs/simulation/GLFrameGrabber.hpp>
// #include <rw/graphics/SceneViewer.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
// #include <rws/ImageUtil.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <numeric>

using namespace rw::kinematics;
using namespace rw::math;

int main(int argc, char** argv)
{
    // load workcell
    static const std::string WC_FILE = "../WorkCell/Scene.wc.xml";
    rw::models::WorkCell::Ptr wc =
        rw::loaders::WorkCellLoader::Factory::load (WC_FILE);
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR-6-85-5-A");
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }

    // find camera frames
    MovableFrame::Ptr camera_right = wc->findFrame< MovableFrame > ("Camera_Right");
    if (camera_right.isNull ()) {
        RW_THROW ("COULD not find movable frame Camera_Right ... check model");
    }
    const rw::core::PropertyMap& camera_right_properties = camera_right->getPropertyMap();
    if(!camera_right_properties.has("Camera"))
    {
        RW_THROW ("Camera frame does not have Camera property.");
    }

    const std::string camera_right_parameters = camera_right_properties.get<std::string> ("Camera");
    // std::cout << "\n" << camera_right_parameters;
    std::istringstream iss (camera_right_parameters, std::istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    std::cout << "Camera_Right properties: fov " << fovy << " width " << width << " height " << height
              << std::endl;


    // camera_right_properties.
    // // rw::graphics::SceneViewer::Ptr gldrawer = //rwstudio->getView ()->getSceneViewer ();
    // // SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
    // // rwlibs::simulation::GLFrameGrabbr::Ptr camR_FrameGrabber = 


    
    // rwlibs::simulation::GLFrameGrabber gl_framegrabber = rwlibs::simulation::GLFrameGrabber();

    // rw::sensor::Image img_r = 

    // RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
    // rwstudio->postOpenWorkCell (WC_FILE);
    // TimerUtil::sleepMs (5000);


    // const rw::graphics::SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();

    // // get the default state
    // State state = wc->getDefaultState ();
    // rwlibs::simulation::GLFrameGrabber::Ptr camR_FrameGrabber = rw::ownedPtr(new rwlibs::simulation::GLFrameGrabber(width, height, fovy));
    
    // camR_FrameGrabber->grab(camera_right,state);

    //wc->findSensor("Camera_Right"); //(wc,)

    

    // RobWorkStudioApp app ("");
    rws::RobWorkStudioApp app("");
    RWS_START(app)
    {
        std::cout << "\nINSIDE APP SCOPE\n";
        // rws::RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        // rwstudio->postOpenWorkCell (WC_FILE);
        // rw::common::TimerUtil::sleepMs (5000);
        app.close ();
    }
    RWS_END()

    std::cout << "\nPROGRAM DONE\n";

    return 0;
}
