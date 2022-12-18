//#include <rw/invkin.hpp>
#include <rw/rw.hpp>
//#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
//#include <rw/core/PropertyMap.hpp>
//#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

//#include <rwlibs/simulation/GLFrameGrabber.hpp>
//#include <rwlibs/simulation/SimulatedCamera.hpp>
//#include <rw/graphics/SceneViewer.hpp>
// #include <rws/ImageUtil.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <numeric>

#include <SamplePlugin.hpp>
//#include <

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

    // rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR-6-85-5-A");
    // if (robotUR5.isNull ()) {
    //     RW_THROW ("COULD not find device UR5 ... check model");
    // }

    // // find camera frames
    // MovableFrame::Ptr camera = wc->findFrame< MovableFrame > ("Camera_Right");
    // if (camera.isNull ()) {
    //     RW_THROW ("COULD not find movable frame Camera_Right ... check model");
    // }
    // const rw::core::PropertyMap& camera_right_properties = camera->getPropertyMap();
    // if(!camera_right_properties.has("Camera"))
    // {
    //     RW_THROW ("Camera frame does not have Camera property.");
    // }

    // const std::string camera_right_parameters = camera_right_properties.get<std::string> ("Camera");
    // // std::cout << "\n" << camera_right_parameters;
    // std::istringstream iss (camera_right_parameters, std::istringstream::in);
    // double fovy;
    // int width;
    // int height;
    // iss >> fovy >> width >> height;
    // std::cout << "Camera_Right properties: fov " << fovy << " width " << width << " height " << height
    //           << std::endl;



    // // // rw::graphics::SceneViewer::Ptr gldrawer = //rwstudio->getView ()->getSceneViewer ();
    // // // SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
    // // // rwlibs::simulation::GLFrameGrabbr::Ptr camR_FrameGrabber = 


    // // RobWorkStudioApp app ("");
    // rws::RobWorkStudioApp app("");
    // RWS_START(app)
    // {
    //     std::cout << "\nINSIDE APP SCOPE\n";
    //     rws::RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
    //     rwstudio->postOpenWorkCell (WC_FILE);
    //     rw::common::TimerUtil::sleepMs (5000);
        
    //     const rw::graphics::SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
    //     const rwlibs::simulation::GLFrameGrabber::Ptr framegrabber =
    //         rw::ownedPtr (new rwlibs::simulation::GLFrameGrabber (width, height, fovy));
    //     framegrabber->init (gldrawer);
    //     rwlibs::simulation::SimulatedCamera::Ptr simcam =
    //         rw::ownedPtr (new rwlibs::simulation::SimulatedCamera ("SimulatedCamera", fovy, camera, framegrabber));
    //     simcam->setFrameRate (100);
    //     simcam->initialize ();
    //     simcam->start ();
    //     simcam->acquire ();

    //     static const double DT = 0.001;
    //     const rwlibs::simulation::Simulator::UpdateInfo info (DT);
    //     State state = wc->getDefaultState ();
    //     int cnt     = 0;
    //     const rw::sensor::Image* img;
    //     while (!simcam->isImageReady ()) {
    //         std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
    //         simcam->update (info, state);
    //         cnt++;
    //     }
    //     img = simcam->getImage ();
    //     img->saveAsPPM ("Image1.ppm");
    //     simcam->acquire ();
    //     while (!simcam->isImageReady ()) {
    //         std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
    //         simcam->update (info, state);
    //         cnt++;
    //     }
    //     std::cout << "Took " << cnt << " steps" << std::endl;
    //     img = simcam->getImage ();
    //     std::cout << "Image: " << img->getWidth () << "x" << img->getHeight () << " bits "
    //               << img->getBitsPerPixel () << " channels " << img->getNrOfChannels ()
    //               << std::endl;
    //     img->saveAsPPM ("Image2.ppm");

    //     simcam->stop ();
    //     app.close ();
    // }
    // RWS_END()

    SamplePlugin visionplugin();
    //roviplugin.initialize();
    visionplugin.open(wc);
    // roviplugin


    std::cout << "\nPROGRAM DONE\n";

    return 0;
}
