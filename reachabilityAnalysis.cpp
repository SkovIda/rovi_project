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

std::vector< Q > getCollisionFreeSolutions(State _state, MovableFrame::Ptr _graspTargetFrame, MovableFrame::Ptr _bottleFrame, rw::models::SerialDevice::Ptr _robotUR5, rw::models::WorkCell::Ptr _wc)
{
    std::vector< Q > _collisionFreeSolutions;
    // create Collision Detector
    rw::proximity::CollisionDetector _detector (
        _wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    for (double rollAngle = 0; rollAngle < 360.0;
         rollAngle += 1.0) {    // for every degree around the roll axis

        _bottleFrame->setTransform (Transform3D<> (Vector3D<> (_bottleFrame->getTransform (_state).P ()),
                                              RPY<> (rollAngle * Deg2Rad, 0, 90 * Deg2Rad)),
                               _state);

        std::vector< Q > solutions =
            getConfigurations ("GraspTarget", "GraspTCP", _robotUR5, _wc, _state);

        for (unsigned int i = 0; i < solutions.size (); i++) {
            // set the robot in that configuration and check if it is in collision
            _robotUR5->setQ (solutions[i], _state);

            if (!_detector.inCollision (_state)) {
                _collisionFreeSolutions.push_back (solutions[i]);    // save it
                break;                                              // we only need one
            }
        }
    }
    return _collisionFreeSolutions;
}


// std::vector< Q > getCollisionFreePlaceSolutions(State _state, MovableFrame::Ptr _placeFrame, rw::models::SerialDevice::Ptr _robotUR5, rw::models::WorkCell::Ptr _wc)
// {
//     std::vector< Q > _collisionFreeSolutions;
//     // create Collision Detector
//     rw::proximity::CollisionDetector _detector (
//         _wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

//     for (double rollAngle = 0; rollAngle < 360.0;
//          rollAngle += 1.0) {    // for every degree around the roll axis

//         _placeFrame->setTransform (Transform3D<> (Vector3D<> (_placeFrame->getTransform (_state).P ()),
//                                               RPY<> (rollAngle * Deg2Rad, 0, 90 * Deg2Rad)),
//                                _state);

//         std::vector< Q > solutions =
//             getConfigurations ("PlaceTarget", "GraspTCP", _robotUR5, _wc, _state);

//         for (unsigned int i = 0; i < solutions.size (); i++) {
//             // set the robot in that configuration and check if it is in collision
//             _robotUR5->setQ (solutions[i], _state);

//             if (!_detector.inCollision (_state)) {
//                 _collisionFreeSolutions.push_back (solutions[i]);    // save it
//                 break;                                              // we only need one
//             }
//         }
//     }
//     return _collisionFreeSolutions;
// }



int main (int argc, char** argv)
{
    // load workcell
    rw::models::WorkCell::Ptr wc =
        rw::loaders::WorkCellLoader::Factory::load ("../WorkCell/Scene.wc.xml");
    if (wc.isNull ()) {
        RW_THROW ("COULD NOT LOAD scene... check path!");
    }

    // find relevant frames
    MovableFrame::Ptr bottleFrame = wc->findFrame< MovableFrame > ("Bottle");
    if (bottleFrame.isNull ()) {
        RW_THROW ("COULD not find movable frame Bottle ... check model");
    }

    rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR-6-85-5-A");
    if (robotUR5.isNull ()) {
        RW_THROW ("COULD not find device UR5 ... check model");
    }

    MovableFrame::Ptr graspTargetFrame = wc->findFrame< MovableFrame > ("GraspTarget");
    if (graspTargetFrame.isNull ()) {
        RW_THROW ("COULD not find movable frame GraspTarget ... check model");
    }

    // MovableFrame::Ptr placeBottleFrame = wc->findFrame< MovableFrame > ("PlaceTarget");
    // if (placeBottleFrame.isNull ()) {
    //     RW_THROW ("COULD not find movable frame PlaceTarget ... check model");
    // }

    // // create Collision Detector
    // rw::proximity::CollisionDetector detector (
    //     wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    // get the default state
    State state = wc->getDefaultState ();
    std::vector< Q > collisionFreeSolutions;

    // // graspTargetFrame to grasp the bottle from the side:
    // graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
    //                                           RPY<> (0, 90 * Deg2Rad, 0)),
    //                            state);

    // // graspTargetFrame to grasp the bottle from the top:
    // graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
    //                                           RPY<> (-90 * Deg2Rad, 90 * Deg2Rad, 0)),
    //                            state);


    // Grasping styles for grasping the bottle:
    std::vector<RPY<>> graspStyles = {RPY<> (0, 90 * Deg2Rad, 0),RPY<> (-90 * Deg2Rad, 90 * Deg2Rad, 0)};

    // // Comment this in to perform reachability analysis of the Place task:
    // // Move bottleFrame to the center of the place area: <RPY> -90 0 90 </RPY> <Pos> 0.3 -0.474 0.21 </Pos>
    // bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (0.3, -0.948, 0)),
    //                                         RPY<> (0, 0, 90 * Deg2Rad)),
    //                         state);

    graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
                                              RPY<> (0, 0, 0)),
                               state);

    rw::trajectory::TimedStatePath tStatePath;
    double time = 0;

    for( unsigned int i = 0; i < graspStyles.size(); i++)
    {
        // Reachability Analysis for Grasping Task:
        // set the graspTargetFrame to grasp the bottle from the side and from the top:
        graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0, 0)),
                                              graspStyles[i]),
                               state);
        
        collisionFreeSolutions = getCollisionFreeSolutions(state, graspTargetFrame, bottleFrame, robotUR5, wc);

        std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
              << std::endl;
        for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
        robotUR5->setQ (collisionFreeSolutions[i], state);
        tStatePath.push_back (rw::trajectory::TimedState (time, state));
        time += 0.01;
        }
    }

    // Comment this in to perform reachability analysis of the Place task:
    // Move bottleFrame to the center of the place area: <RPY> -90 0 90 </RPY> <Pos> 0.3 -0.474 0.21 </Pos>
    bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (0.3, -0.948, 0)),
                                            RPY<> (0, 0, 90 * Deg2Rad)),
                            state);

    for( unsigned int i = 0; i < graspStyles.size(); i++)
    {
        // Reachability Analysis for Grasping Task:
        // set the graspTargetFrame to grasp the bottle from the side and from the top:
        graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0, 0)),
                                              graspStyles[i]),
                               state);
        
        collisionFreeSolutions = getCollisionFreeSolutions(state, graspTargetFrame, bottleFrame, robotUR5, wc);

        std::cout << "Current position of the robot vs object to be grasped has: "
              << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
              << std::endl;
        for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
        robotUR5->setQ (collisionFreeSolutions[i], state);
        tStatePath.push_back (rw::trajectory::TimedState (time, state));
        time += 0.01;
        }
    }

    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../visu.rwplay");

    // for (double rollAngle = 0; rollAngle < 360.0;
    //      rollAngle += 1.0) {    // for every degree around the roll axis

    //     bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P ()),
    //                                           RPY<> (rollAngle * Deg2Rad, 0, 90 * Deg2Rad)),
    //                            state);

    //     std::vector< Q > solutions =
    //         getConfigurations ("GraspTarget", "GraspTCP", robotUR5, wc, state);

    //     for (unsigned int i = 0; i < solutions.size (); i++) {
    //         // set the robot in that configuration and check if it is in collision
    //         robotUR5->setQ (solutions[i], state);

    //         if (!detector.inCollision (state)) {
    //             collisionFreeSolutions.push_back (solutions[i]);    // save it
    //             break;                                              // we only need one
    //         }
    //     }
    // }

    // std::cout << "Current position of the robot vs object to be grasped has: "
    //           << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
    //           << std::endl;

    // // visualize them
    // rw::trajectory::TimedStatePath tStatePath;
    // double time = 0;
    // for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
    //     robotUR5->setQ (collisionFreeSolutions[i], state);
    //     tStatePath.push_back (rw::trajectory::TimedState (time, state));
    //     time += 0.01;
    // }

    // rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../visu.rwplay");

    // std::vector<std::string> grasping_style_names = {"Grasp bottle from the top", "Grasp bottle from the side"};
    // std::vector< Q > collisionFreeSolutions_GraspTop;
    // std::vector< Q > collisionFreeSolutions_GraspSide;
    // std::vector<std::vector< Q >> collisionFreeSolutions = {collisionFreeSolutions_GraspTop, collisionFreeSolutions_GraspSide};

    // std::vector< Transform3D<> > graspTargetFrame_transformations;
    // graspTargetFrame_transformations.push_back(Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
    //                                           RPY<> (0, 90 * Deg2Rad, 0)));
    // // graspTargetFrame_transformations.push_back(Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0, 0)),
    // //                                           RPY<> (-90 * Deg2Rad, 90 * Deg2Rad, 0)));

    // //for (Transform3D<> grasping_target_frame: graspTargetFrame_transformations)
    // for(size_t nth_graspstyle = 0; nth_graspstyle <= grasping_style_names.size(); nth_graspstyle++)
    // {
    //     std::string name = grasping_style_names[nth_graspstyle];
    //     Transform3D<> grasping_target_frame = graspTargetFrame_transformations[nth_graspstyle];

    //     std::cout << "\n\nIterate through 3Dtransforms stored in a vector\n\n";
    //     graspTargetFrame->setTransform (grasping_target_frame,
    //                            state);

    //     for (double rollAngle = 0; rollAngle < 360.0;
    //     rollAngle += 1.0) {    // for every degree around the roll axis

    //         bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P ()),
    //                                             RPY<> (rollAngle * Deg2Rad, 0, 90 * Deg2Rad)),
    //                             state);

    //         std::vector< Q > solutions =
    //             getConfigurations ("GraspTarget", "GraspTCP", robotUR5, wc, state);

    //         for (unsigned int i = 0; i < solutions.size (); i++) {
    //             // set the robot in that configuration and check if it is in collision
    //             robotUR5->setQ (solutions[i], state);

    //             if (!detector.inCollision (state)) {
    //                 collisionFreeSolutions[nth_graspstyle].push_back (solutions[i]);    // save it
    //                 break;                                              // we only need one
    //             }
    //         }
    //     }
    // }


    // // visualize them
    // rw::trajectory::TimedStatePath tStatePath;
    // for(size_t nth_graspstyle = 0; nth_graspstyle <= grasping_style_names.size(); nth_graspstyle++)
    // {
    //     std::cout << "Current position of the robot vs object to be grasped has: "
    //                 << collisionFreeSolutions[nth_graspstyle].size () << " collision-free inverse kinematics solutions!"
    //                 << std::endl;
    
    //     double time = 0;
    //     for (unsigned int i = 0; i < collisionFreeSolutions[nth_graspstyle].size (); i++) {
    //         robotUR5->setQ (collisionFreeSolutions[nth_graspstyle][i], state);
    //         tStatePath.push_back (rw::trajectory::TimedState (time, state));
    //         time += 0.01;
    //     }
    // }

    // rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../visu.rwplay");

    return 0;
}
