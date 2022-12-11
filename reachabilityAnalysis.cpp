#include <rw/invkin.hpp>
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <numeric>

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

    MovableFrame::Ptr robotBaseFrame = wc->findFrame< MovableFrame > ("URReference");
    if (robotBaseFrame.isNull ()) {
        RW_THROW ("COULD not find movable frame URReference ... check model");
    }

    // MovableFrame::Ptr placeBottleFrame = wc->findFrame< MovableFrame > ("PlaceTarget");
    // if (placeBottleFrame.isNull ()) {
    //     RW_THROW ("COULD not find movable frame PlaceTarget ... check model");
    // }

    // // create Collision Detector
    // rw::proximity::CollisionDetector detector (
    //     wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    // // Save the number of collision free solitions:
    // for all robot base positions:
    // For the two different grasping styles: Grasp bottle from the side and grasp bottle from the top
    // For both the 'pick area' and the 'place area'
    std::ofstream collisionfree_solutions_wrt_basePose;
    collisionfree_solutions_wrt_basePose.open("reachability_analysis_collisionfree_solutions.csv");
    // Write headers of the .csv output file:
    collisionfree_solutions_wrt_basePose << "xPos_idx,yPos_idx,Pick_GraspStyle_side,Pick_GraspStyle_top,Place_GraspStyle_side,Place_GraspStyle_top\n";

    // get the default state
    State state = wc->getDefaultState ();
    std::vector< Q > collisionFreeSolutions;

    // Move BaseFrame of the UR5 robot:
    robotBaseFrame->setTransform (Transform3D<> (Vector3D<> (robotBaseFrame->getTransform (state).P () + Vector3D<> (-0.3, -0.2, 0)),
                                              RPY<> (0, 0, 0)),
                               state);


    //////////// Reachability analysis of the Pick task:
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
    
    int n_xPos = 7;
    int n_yPos = 5;
    // double xIncrement = 0.1;
    // double yIncrement = 0.1;
    double pose_translation_stepsize = 0.1;
    double yPos_range = 0.4;
    std::cout << "Move Base:\n";
    for(int x_idx = 0; x_idx < n_xPos; x_idx++){
        for(int y_idx = 0; y_idx < n_yPos; y_idx++){
            std::cout << "Move base: Pos_idx = [" << x_idx << "," << y_idx << "]\n";
            collisionfree_solutions_wrt_basePose << x_idx << "," << y_idx;
            // Do stuff HERE:
            for( unsigned int i = 0; i < graspStyles.size(); i++)
            {
                // Reachability Analysis for Grasping Task:
                // set the graspTargetFrame to grasp the bottle from the side and from the top:
                graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0, 0)),
                                                    graspStyles[i]),
                                    state);
                
                collisionFreeSolutions = getCollisionFreeSolutions(state, graspTargetFrame, bottleFrame, robotUR5, wc);

                // std::cout << "Current position of the robot vs object to be grasped has: "
                //     << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
                //     << std::endl;

                collisionfree_solutions_wrt_basePose << "," << collisionFreeSolutions.size ();

                for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
                robotUR5->setQ (collisionFreeSolutions[i], state);
                tStatePath.push_back (rw::trajectory::TimedState (time, state));
                time += 0.01;
                }
            }

            ////////////// Reachability analysis of the Place task:
            // Move bottleFrame to the center of the place area (Position of this in TableFrame is approx.: (0.3 -0.474 0.21)
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

                // std::cout << "Current position of the robot vs object to be placed has: "
                //     << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
                //     << std::endl;

                collisionfree_solutions_wrt_basePose << "," << collisionFreeSolutions.size ();

                for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
                robotUR5->setQ (collisionFreeSolutions[i], state);
                tStatePath.push_back (rw::trajectory::TimedState (time, state));
                time += 0.01;
                }
            }

            collisionfree_solutions_wrt_basePose <<  "\n";

            // Move bottleFrame back to the pick area (Position of this in TableFrame is approx.: (0 0.474 0.21)
            bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (-0.3, 0.948, 0)),
                                                    RPY<> (0, 0, 90 * Deg2Rad)),
                                    state);

            // Move y 0.1
            robotBaseFrame->setTransform (Transform3D<> (Vector3D<> (robotBaseFrame->getTransform (state).P () + Vector3D<> (0, pose_translation_stepsize, 0)),
                                              RPY<> (0, 0, 0)),
                               state);
        }
        // Move x 0.1 and y -0.4-translation_stepsize = -0.5:
        robotBaseFrame->setTransform (Transform3D<> (Vector3D<> (robotBaseFrame->getTransform (state).P () + Vector3D<> (pose_translation_stepsize, -yPos_range-pose_translation_stepsize, 0)),
                                              RPY<> (0, 0, 0)),
                               state);
    }
    collisionfree_solutions_wrt_basePose.close();

    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../visu.rwplay");

    // // graspTargetFrame to grasp the bottle from the side:
    // graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
    //                                           RPY<> (0, 90 * Deg2Rad, 0)),
    //                            state);

    // // graspTargetFrame to grasp the bottle from the top:
    // graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
    //                                           RPY<> (-90 * Deg2Rad, 90 * Deg2Rad, 0)),
    //                            state);



// /////////////////////////////////////////////////////////////////////////////////////////////
//     //////////// Reachability analysis of the Pick task:
//     // Grasping styles for grasping the bottle:
//     std::vector<RPY<>> graspStyles = {RPY<> (0, 90 * Deg2Rad, 0),RPY<> (-90 * Deg2Rad, 90 * Deg2Rad, 0)};

//     // // Comment this in to perform reachability analysis of the Place task:
//     // // Move bottleFrame to the center of the place area: <RPY> -90 0 90 </RPY> <Pos> 0.3 -0.474 0.21 </Pos>
//     // bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (0.3, -0.948, 0)),
//     //                                         RPY<> (0, 0, 90 * Deg2Rad)),
//     //                         state);

//     graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
//                                               RPY<> (0, 0, 0)),
//                                state);

//     rw::trajectory::TimedStatePath tStatePath;
//     double time = 0;

//     for( unsigned int i = 0; i < graspStyles.size(); i++)
//     {
//         // Reachability Analysis for Grasping Task:
//         // set the graspTargetFrame to grasp the bottle from the side and from the top:
//         graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0, 0)),
//                                               graspStyles[i]),
//                                state);
        
//         collisionFreeSolutions = getCollisionFreeSolutions(state, graspTargetFrame, bottleFrame, robotUR5, wc);

//         std::cout << "Current position of the robot vs object to be grasped has: "
//               << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
//               << std::endl;
//         for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
//         robotUR5->setQ (collisionFreeSolutions[i], state);
//         tStatePath.push_back (rw::trajectory::TimedState (time, state));
//         time += 0.01;
//         }
//     }

//     ////////////// Reachability analysis of the Place task:
//     // Move bottleFrame to the center of the place area (Position of this in TableFrame is approx.: (0.3 -0.474 0.21)
//     bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (0.3, -0.948, 0)),
//                                             RPY<> (0, 0, 90 * Deg2Rad)),
//                             state);

//     for( unsigned int i = 0; i < graspStyles.size(); i++)
//     {
//         // Reachability Analysis for Grasping Task:
//         // set the graspTargetFrame to grasp the bottle from the side and from the top:
//         graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0, 0)),
//                                               graspStyles[i]),
//                                state);
        
//         collisionFreeSolutions = getCollisionFreeSolutions(state, graspTargetFrame, bottleFrame, robotUR5, wc);

//         std::cout << "Current position of the robot vs object to be placed has: "
//               << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
//               << std::endl;
//         for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
//         robotUR5->setQ (collisionFreeSolutions[i], state);
//         tStatePath.push_back (rw::trajectory::TimedState (time, state));
//         time += 0.01;
//         }
//     }

//     // Move bottleFrame back to the pick area (Position of this in TableFrame is approx.: (0 0.474 0.21)
//     bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (-0.3, 0.948, 0)),
//                                             RPY<> (0, 0, 90 * Deg2Rad)),
//                             state);


//     rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../visu.rwplay");

//     ////////////////////////////////////////////////////////////////////////////////////////

    return 0;
}
