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

/**
 * This code is heavily inspired/copied from the solution provided for Programming Exercise 5.
 * The parts, that were not copied from the solution for Programming Exercise 5, was by Ida Blirup Skov
 */


/**
 * @brief Get the configuration of the robot for a given pose of the TCP.
 * This function is a copy of a function provided in the solution for Programming Exercise 5
 * @param nameGoal 
 * @param nameTcp 
 * @param robot 
 * @param wc 
 * @param state 
 * @return std::vector< Q > Configuration of the robot
 */
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


/**
 * @brief Get the Collision Free Solutions of the robot for every degree around the roll axis.
 * The content of this function is almost an exact copy of the solution provided for Programming Exercise 5, except some small changes made by Ida Blirup Skov
 * @param _state 
 * @param _graspTargetFrame 
 * @param _bottleFrame 
 * @param _robotUR5 
 * @param _wc 
 * @return std::vector< Q > 
 */
std::vector< Q > getCollisionFreeSolutions(State _state, MovableFrame::Ptr _graspTargetFrame,
                                        MovableFrame::Ptr _bottleFrame,
                                        rw::models::SerialDevice::Ptr _robotUR5,
                                        rw::models::WorkCell::Ptr _wc)
{
    std::vector< Q > _collisionFreeSolutions;
    // create Collision Detector
    rw::proximity::CollisionDetector _detector (
        _wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    for (double rollAngle = 0; rollAngle < 360.0;
         rollAngle += 1.0) {    // for every degree around the roll axis

        _bottleFrame->setTransform (Transform3D<> (Vector3D<> (_bottleFrame->getTransform (_state).P ()),
                                              RPY<> (rollAngle * Deg2Rad, 0, 90 * Deg2Rad)),
                               _state);     // Some small changes were made to the input arguments of this function call.

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

    /**
     * Save the number of collision free solutions
     * For all robot base positions:
     * Number of collision free solutions for the two different grasping styles: Grasp bottle from the side and grasp bottle from the top 
     * Number of collision free solutions for both the 'pick area' and the 'place area'
     **/
    std::ofstream collisionfree_solutions_wrt_basePose_outfile;
    collisionfree_solutions_wrt_basePose_outfile.open("reachability_analysis_collisionfree_solutions.csv");
    // Write headers of the .csv output file:
    collisionfree_solutions_wrt_basePose_outfile << "xPos_idx,yPos_idx,Pick_GraspStyle_side,Pick_GraspStyle_top,Place_GraspStyle_side,Place_GraspStyle_top\n";

    // get the default state
    State state = wc->getDefaultState ();
    std::vector< Q > collisionFreeSolutions;

    // Move the BaseFrame of the UR5 robot to the first base position used to test the reachability:
    robotBaseFrame->setTransform (Transform3D<> (Vector3D<> (robotBaseFrame->getTransform (state).P () + Vector3D<> (-0.3, -0.2, 0)),
                                              RPY<> (0, 0, 0)),
                               state);

    // Grasping styles for grasping the bottle: [Grasp from the side, Grasp from the top]
    std::vector<RPY<>> graspStyles = {RPY<> (0, 90 * Deg2Rad, 0),RPY<> (-90 * Deg2Rad, 90 * Deg2Rad, 0)};

    graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
                                              RPY<> (0, 0, 0)),
                               state);

    // Init objects to save a timed state path of the robot to enable replay of it in RobWorkStudio:
    rw::trajectory::TimedStatePath tStatePath;
    double time = 0;
    
    // Move the base of the robot along positions on a grid with dimensions=(n_xPos X n_yPos):
    int n_xPos = 7;
    int n_yPos = 5;

    double pose_translation_stepsize = 0.1;
    double yPos_range = 0.4;

    for(int x_idx = 0; x_idx < n_xPos; x_idx++){
        for(int y_idx = 0; y_idx < n_yPos; y_idx++){
            std::cout << "Move base: Pos_idx = [" << x_idx << "," << y_idx << "]\n";
            collisionfree_solutions_wrt_basePose_outfile << x_idx << "," << y_idx;  // Write grid index to .csv file

            ////////////// Reachability analysis of the Pick task:
            for( unsigned int i = 0; i < graspStyles.size(); i++)   // For each grasping possibility:
            {
                // set the graspTargetFrame to grasp the bottle from the side or from the top:
                graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0, 0)),
                                                    graspStyles[i]),
                                    state);
                
                collisionFreeSolutions = getCollisionFreeSolutions(state, graspTargetFrame, bottleFrame, robotUR5, wc);

                // std::cout << "Current position of the robot vs object to be grasped has: "
                //     << collisionFreeSolutions.size () << " collision-free inverse kinematics solutions!"
                //     << std::endl;

                // Write number of collision free solutions to .csv file
                collisionfree_solutions_wrt_basePose_outfile << "," << collisionFreeSolutions.size ();

                // Add collision free solutions to replay file:
                for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
                robotUR5->setQ (collisionFreeSolutions[i], state);
                tStatePath.push_back (rw::trajectory::TimedState (time, state));
                time += 0.01;
                }
            }

            // Move bottleFrame to the center of the place area (Position of this in TableFrame is approx.: (0.3 -0.474 0.21)
            bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (0.3, -0.948, 0)),
                                                    RPY<> (0, 0, 90 * Deg2Rad)),
                                    state);

            ////////////// Reachability analysis of the Place task:
            for( unsigned int i = 0; i < graspStyles.size(); i++)   // For each grasping possibility:
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

                collisionfree_solutions_wrt_basePose_outfile << "," << collisionFreeSolutions.size ();

                for (unsigned int i = 0; i < collisionFreeSolutions.size (); i++) {
                robotUR5->setQ (collisionFreeSolutions[i], state);
                tStatePath.push_back (rw::trajectory::TimedState (time, state));
                time += 0.01;
                }
            }
            collisionfree_solutions_wrt_basePose_outfile <<  "\n";

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
    collisionfree_solutions_wrt_basePose_outfile.close();

    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../visu.rwplay");

    return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// int main (int argc, char** argv)
// {
//     // load workcell
//     rw::models::WorkCell::Ptr wc =
//         rw::loaders::WorkCellLoader::Factory::load ("../WorkCell/Scene.wc.xml");
//     if (wc.isNull ()) {
//         RW_THROW ("COULD NOT LOAD scene... check path!");
//     }

//     // find relevant frames
//     MovableFrame::Ptr bottleFrame = wc->findFrame< MovableFrame > ("Bottle");
//     if (bottleFrame.isNull ()) {
//         RW_THROW ("COULD not find movable frame Bottle ... check model");
//     }

//     rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice< rw::models::SerialDevice > ("UR-6-85-5-A");
//     if (robotUR5.isNull ()) {
//         RW_THROW ("COULD not find device UR5 ... check model");
//     }

//     MovableFrame::Ptr graspTargetFrame = wc->findFrame< MovableFrame > ("GraspTarget");
//     if (graspTargetFrame.isNull ()) {
//         RW_THROW ("COULD not find movable frame GraspTarget ... check model");
//     }

//     MovableFrame::Ptr robotBaseFrame = wc->findFrame< MovableFrame > ("URReference");
//     if (robotBaseFrame.isNull ()) {
//         RW_THROW ("COULD not find movable frame URReference ... check model");
//     }

//     // /**
//     //  * Save the number of collision free solutions
//     //  * For all robot base positions:
//     //  * Number of collision free solutions for the two different grasping styles: Grasp bottle from the side and grasp bottle from the top 
//     //  * Number of collision free solutions for both the 'pick area' and the 'place area'
//     //  **/
//     // std::ofstream collisionfree_solutions_wrt_basePose_outfile;
//     // collisionfree_solutions_wrt_basePose_outfile.open("reachability_analysis_collisionfree_solutions.csv");
//     // // Write headers of the .csv output file:
//     // collisionfree_solutions_wrt_basePose_outfile << "xPos_idx,yPos_idx,Pick_GraspStyle_side,Pick_GraspStyle_top,Place_GraspStyle_side,Place_GraspStyle_top\n";

//     // get the default state
//     State state = wc->getDefaultState ();
//     // std::vector< Q > collisionFreeSolutions;

//     // Grasping styles for grasping the bottle: [Grasp from the side, Grasp from the top]
//     std::vector<RPY<>> graspStyles = {RPY<> (0, 90 * Deg2Rad, 0),RPY<> (-90 * Deg2Rad, 90 * Deg2Rad, 0)};
//     std::vector<std::string> grasp_names = {"side", "top"};

//     graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0.028, 0)),
//                                               RPY<> (0, 0, 0)),
//                                state);
    
//     // Init objects to save a timed state path of the robot to enable replay of it in RobWorkStudio:
//     rw::trajectory::TimedStatePath tStatePath;
//     double time = 0;
//     std::vector< Q > collisionFreeSolutions;

//     // std::vector<double> radius_list = {0.2, 0.3};
//     //double max_radius = 0.9;
//     // double radius = 0.3;
//     double ur5_arm_reach = 0.85; // in milimeters
//     double center_cylinder_radius_incl_buffer = 0.20; // in milimeters
//     double grid_stepsize = 0.01;

//     double prev_x_coor = 0.0;
//     double prev_y_coor = 0.474;
//     double prev_z_coor = 0.21;

//     // create Collision Detector
//     rw::proximity::CollisionDetector _detector (wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

//     for( unsigned int i = 0; i < graspStyles.size(); i++)   // For each grasping possibility:
//     {
//         std::ofstream ws_coordinates_outfile;
//         ws_coordinates_outfile.open("workspace_grasp_from_the_" + grasp_names[i] + "_grid_ver2.csv");
//         ws_coordinates_outfile << "x,y,z";

//         std::cout << "\nSOLUTIONS FOUND:\nGrasp from the " << grasp_names[i];
//         // Reachability Analysis for Grasping Task:
//         // set the graspTargetFrame to grasp the bottle from the side and from the top:
//         graspTargetFrame->setTransform (Transform3D<> (Vector3D<> (graspTargetFrame->getTransform (state).P () + Vector3D<> (0, 0, 0)),
//                                             graspStyles[i]),
//                             state);

//         for(double z_coor = 0.21; z_coor <= ur5_arm_reach; z_coor += grid_stepsize){
//             std::cout << "\nz_coor=" << z_coor;
//             // std::cout << "\n\nx_coor=" << x_coor;
//             for(double x_coor = -ur5_arm_reach; x_coor <= ur5_arm_reach; x_coor += grid_stepsize){
//                 // if(abs(x_coor) <= center_cylinder_radius_incl_buffer)
//                 // {
//                 //     continue;
//                 // }
//                 std::cout << "\n\tx_coor=" << x_coor;
//                 //double z_coor = radius*sin(phi*Deg2Rad);
//                 bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (0, 0, z_coor - prev_z_coor)),
//                                                     RPY<> (0, 0, 90 * Deg2Rad)), state);

//                 for(double y_coor = -ur5_arm_reach; y_coor <= ur5_arm_reach; y_coor += grid_stepsize){
//                     if(sqrt(abs(y_coor*y_coor) + abs(x_coor*x_coor)) < center_cylinder_radius_incl_buffer)
//                     {
//                         continue;
//                     }
//                     // double x_coor = radius*cos(theta*Deg2Rad);
//                     // double y_coor = radius*sin(theta*Deg2Rad);
//                     // double x_coor = radius*cos(theta*Deg2Rad)*sin(phi*Deg2Rad);
//                     // double y_coor = radius*sin(theta*Deg2Rad)*sin(phi*Deg2Rad);

//                     // Vector3D<> translate_bottle = Vector3D<>(prev_x_coor - x_coor, prev_y_coor - y_coor, 0);
//                     Vector3D<> translate_bottle = Vector3D<>(x_coor - prev_x_coor, y_coor - prev_y_coor, 0);

//                     bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + translate_bottle),
//                                                     RPY<> (0, 0, 90 * Deg2Rad)), state);

//                     collisionFreeSolutions={};

//                     for (double rollAngle = 0; rollAngle < 360.0;
//                         rollAngle += 1.0) {    // for every degree around the roll axis

//                         bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P ()),
//                                                             RPY<> (rollAngle * Deg2Rad, 0, 90 * Deg2Rad)),state);

//                         std::vector< Q > solutions =
//                             getConfigurations ("GraspTarget", "GraspTCP", robotUR5, wc, state);

//                         for (unsigned int itr = 0; itr < solutions.size (); itr++) {
//                             // set the robot in that configuration and check if it is in collision
//                             robotUR5->setQ (solutions[itr], state);

//                             if (!_detector.inCollision (state)) {
//                                 collisionFreeSolutions.push_back(solutions[itr]);
//                                 tStatePath.push_back (rw::trajectory::TimedState (time, state));
//                                 time += 0.01;
//                                 break;          // we only need one
//                             }
//                         }
//                         if(collisionFreeSolutions.size() > 0){
//                             //std::cout << "\n\t(" << x_coor << "," << y_coor << "," << z_coor << ")";
//                             ws_coordinates_outfile << "\n" << x_coor << "," << y_coor << "," << z_coor;
//                             break;
//                         }
//                     }
//                     prev_x_coor = x_coor;
//                     prev_y_coor = y_coor;
//                 }
//                 prev_z_coor = z_coor;
//             }
//         }
//         // for(double radius = 0.15; radius <= max_radius; radius += 0.1){
//         //     std::cout << "\n\nRadius=" << radius;
//         //     for(double z_coor = 0.21; z_coor <= max_radius; z_coor += 0.1){
//         //         std::cout << "\n\t" << z_coor;
//         //         //double z_coor = radius*sin(phi*Deg2Rad);
//         //         bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + Vector3D<> (0, 0, z_coor - prev_z_coor)),
//         //                                             RPY<> (0, 0, 90 * Deg2Rad)), state);
//         //         for(double theta = 0.0; theta < 360.0; theta += 1.0){
//         //             double x_coor = radius*cos(theta*Deg2Rad);
//         //             double y_coor = radius*sin(theta*Deg2Rad);
//         //             // double x_coor = radius*cos(theta*Deg2Rad)*sin(phi*Deg2Rad);
//         //             // double y_coor = radius*sin(theta*Deg2Rad)*sin(phi*Deg2Rad);

//         //             // Vector3D<> translate_bottle = Vector3D<>(prev_x_coor - x_coor, prev_y_coor - y_coor, 0);
//         //             Vector3D<> translate_bottle = Vector3D<>(x_coor - prev_x_coor, y_coor - prev_y_coor, 0);

//         //             bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P () + translate_bottle),
//         //                                             RPY<> (0, 0, 90 * Deg2Rad)), state);

//         //             collisionFreeSolutions={};

//         //             for (double rollAngle = 0; rollAngle < 360.0;
//         //                 rollAngle += 1.0) {    // for every degree around the roll axis

//         //                 bottleFrame->setTransform (Transform3D<> (Vector3D<> (bottleFrame->getTransform (state).P ()),
//         //                                                     RPY<> (rollAngle * Deg2Rad, 0, 90 * Deg2Rad)),state);

//         //                 std::vector< Q > solutions =
//         //                     getConfigurations ("GraspTarget", "GraspTCP", robotUR5, wc, state);

//         //                 for (unsigned int itr = 0; itr < solutions.size (); itr++) {
//         //                     // set the robot in that configuration and check if it is in collision
//         //                     robotUR5->setQ (solutions[itr], state);

//         //                     if (!_detector.inCollision (state)) {
//         //                         collisionFreeSolutions.push_back(solutions[itr]);
//         //                         tStatePath.push_back (rw::trajectory::TimedState (time, state));
//         //                         time += 0.01;
//         //                         break;          // we only need one
//         //                     }
//         //                 }
//         //                 if(collisionFreeSolutions.size() > 0){
//         //                     //std::cout << "\n\t(" << x_coor << "," << y_coor << "," << z_coor << ")";
//         //                     ws_coordinates_outfile << "\n" << x_coor << "," << y_coor << "," << z_coor;
//         //                     break;
//         //                 }
//         //             }
//         //             prev_x_coor = x_coor;
//         //             prev_y_coor = y_coor;
//         //         }
//         //         prev_z_coor = z_coor;
//         //     }
//         // }
//         ws_coordinates_outfile.close();
//     }

//     rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, "../visu_ws_grid_sample.rwplay");

//     return 0;
// }
// /////////////////////////////////////////////////////////////////////////////////////////////////////