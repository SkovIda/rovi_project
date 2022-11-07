
#include <rw/invkin.hpp>
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

using namespace rw::kinematics;
using namespace rw::math;

std::vector< Q > getConfigurations (const std::string nameGoal, const std::string nameTcp,
                                    rw::models::SerialDevice::Ptr robot,
                                    rw::models::WorkCell::Ptr wc, State& state)
{
    // Get, make and print name of frames
    const std::string robotName     = robot->getName ();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp  = robotName + "." + "TCP";

    // Find frames and check for existence
    Frame::Ptr goal_f      = wc->findFrame (nameGoal);
    Frame::Ptr tcp_f       = wc->findFrame (nameTcp);
    Frame::Ptr robotBase_f = wc->findFrame (nameRobotBase);
    Frame::Ptr robotTcp_f  = wc->findFrame (nameRobotTcp);
    if (goal_f.isNull () || tcp_f.isNull () || robotBase_f.isNull () || robotTcp_f.isNull ()) {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (goal_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (tcp_f.isNull () ? "NO!" : "YES!")
                  << std::endl;
        std::cout << " Found \"" << nameRobotBase
                  << "\": " << (robotBase_f.isNull () ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp
                  << "\": " << (robotTcp_f.isNull () ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    Transform3D<> baseTGoal    = Kinematics::frameTframe (robotBase_f, goal_f, state);
    Transform3D<> tcpTRobotTcp = Kinematics::frameTframe (tcp_f, robotTcp_f, state);

    // get grasp frame in robot tool frame
    Transform3D<> targetAt = baseTGoal * tcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler =
        rw::core::ownedPtr (new rw::invkin::ClosedFormIKSolverUR (robot, state));

    return closedFormSovler->solve (targetAt, state);
}

int main (int argc, char** argv)
{
    // load workcell
    rw::models::WorkCell::Ptr wc =
        rw::loaders::WorkCellLoader::Factory::load ("../../../WorkCell/Scene.wc.xml");
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    // find relevant frames
    MovableFrame::Ptr cylinderFrame = wc->findFrame< MovableFrame > ("Cylinder");
    if (cylinderFrame.isNull ()) {
        RW_THROW ("COULD not find movable frame Cylinder ... check model");
    }

    rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR-6-85-5-A");
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }

    // create Collision Detector
    rw::proximity::CollisionDetector detector (
        wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    // get the default state
    State state = wc->getDefaultState ();
    std::vector< Q > collisionFreeSolutions;

    for (double rollAngle = 0; rollAngle < 360.0;
         rollAngle += 1.0) {    // for every degree around the roll axis

        cylinderFrame->moveTo (Transform3D<> (Vector3D<> (cylinderFrame->getTransform (state).P ()),
                                              RPY<> (rollAngle * Deg2Rad, 0, 0)),
                               state);

        std::vector< Q > solutions =
            getConfigurations ("GraspTarget", "GraspTCP", robotUR5, wc, state);

        for (unsigned int i = 0; i < solutions.size (); i++) {
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ (solutions[i], state);

            if (!detector.inCollision (state)) {
                collisionFreeSolutions.push_back (solutions[i]);    // save it
                break;                                              // we only need one
            }
        }
    }

    std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
              << std::endl;

    // visualize them
    rw::trajectory::TimedStatePath tStatePath;
    double time = 0;
    for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
        robotUR5->setQ (collisionFreeSolutions[i], state);
        tStatePath.push_back (rw::trajectory::TimedState (time, state));
        time += 0.01;
    }

    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../scene/visu.rwplay");

    return 0;
}
