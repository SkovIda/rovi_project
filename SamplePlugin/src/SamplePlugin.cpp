#include "SamplePlugin.hpp"

#include <filesystem>
#include <fstream>

SamplePlugin::SamplePlugin () : RobWorkStudioPlugin ("SamplePluginUI", QIcon ((std::filesystem::path(__FILE__).parent_path()/"pa_icon.png").c_str()))
{
    setupUi (this);

    _timer = new QTimer (this);
    connect (_timer, SIGNAL (timeout ()), this, SLOT (timer ()));

    // now connect stuff from the ui component
    connect (_btn_im, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_scan, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn0, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn1, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn2, SIGNAL (pressed ()), this, SLOT (btnPressed ()));   // Written by Ida Blirup Skov
    connect (_testSparseStereo3DPose, SIGNAL (pressed ()), this, SLOT (btnPressed ()));   // Written by Ida Blirup Skov
    connect (_spinBox, SIGNAL (valueChanged (int)), this, SLOT (btnPressed ()));

    _framegrabber = NULL;

    _cameras    = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};

    // Intrinsic Parameters of the cameras: (written by Ida Blirup Skov)
    cv::Mat KA(3,4,CV_64F);
    KA.at<double>(0,0) = 514.682;
    KA.at<double>(0,2) = 320.0;
    KA.at<double>(0,3) = 0.0;
    KA.at<double>(1,1) = 514.682;
    KA.at<double>(1,2) = 240.0;
    KA.at<double>(2,2) = 1.0;
    cv::print(KA);

    // Transformation matrix - Right camera: (written by Ida Blirup Skov)
    cv::Mat H_camera_right(4,4,CV_64F);
    H_camera_right.at<cv::Vec4d>(0) = cv::Vec4d(-1.0, -1.22465e-16, -1.49976e-32, -0.15);
    H_camera_right.at<cv::Vec4d>(1) = cv::Vec4d(-1.22465e-16, 1.0, 1.22465e-16, -0.474);
    H_camera_right.at<cv::Vec4d>(2) = cv::Vec4d(0.0, 1.22465e-16, -1.0, 0.925);
    H_camera_right.at<cv::Vec4d>(3) = cv::Vec4d(0.0, 0.0, 0.0, 1.0);
    //cv::print(H_camera_right);

    // Transformation matrix - Left Camera: (written by Ida Blirup Skov)
    cv::Mat H_camera_left(4,4,CV_64F);
    H_camera_left.at<cv::Vec4d>(0) = cv::Vec4d(-1.0, -1.22465e-16, -1.49976e-32, 0.15);
    H_camera_left.at<cv::Vec4d>(1) = cv::Vec4d(-1.22465e-16, 1.0, 1.22465e-16, -0.474);
    H_camera_left.at<cv::Vec4d>(2) = cv::Vec4d(0.0, 1.22465e-16, -1.0, 0.925);
    H_camera_left.at<cv::Vec4d>(3) = cv::Vec4d(0.0, 0.0, 0.0, 1.0);
    //cv::print(H_camera_left);

    // calculate the projection matrices for the Right and Left camera: (written by Ida Blirup Skov)
    _proj_right = KA * H_camera_right;
    _proj_left = KA * H_camera_left;

    // _sigma = 255.0*90.0/100.0;

}

SamplePlugin::~SamplePlugin ()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize ()
{
    log ().info () << "INITALIZE"
                   << "\n";

    getRobWorkStudio ()->stateChangedEvent ().add (
        std::bind (&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Auto load workcell
    std::filesystem::path wc_path (__FILE__);
    wc_path          = wc_path.parent_path() / "../../WorkCell/Scene.wc.xml";
	std::cout << "wc path: " << wc_path << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (wc_path.string ());
    getRobWorkStudio ()->setWorkCell (wc);

}

void SamplePlugin::open (WorkCell* workcell)
{
    log ().info () << "OPEN"
                   << "\n";
    _wc    = workcell;
    _state = _wc->getDefaultState ();

    log ().info () << workcell->getFilename () << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame ("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "TextureImage", _textureRender, textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame ("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "BackgroundImage", _bgRender, bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame (_cameras[0]);
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap ().has ("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber             = new GLFrameGrabber (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber->init (gldrawer);
            }
        }

        Frame* cameraFrame25D = _wc->findFrame (_cameras25D[0]);
        if (cameraFrame25D != NULL) {
            if (cameraFrame25D->getPropertyMap ().has ("Scanner25D")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam =
                    cameraFrame25D->getPropertyMap ().get< std::string > ("Scanner25D");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D          = new GLFrameGrabber25D (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber25D->init (gldrawer);
            }
        }
        _device = _wc->findDevice ("UR-6-85-5-A");
        _step   = -1;
    }
}

void SamplePlugin::close ()
{
    log ().info () << "CLOSE"
                   << "\n";

    // Stop the timer
    _timer->stop ();
    // Remove the texture render
    Frame* textureFrame = _wc->findFrame ("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("TextureImage", textureFrame);
    }
    // Remove the background render
    Frame* bgFrame = _wc->findFrame ("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("BackgroundImage", bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc           = NULL;
}

Mat SamplePlugin::toOpenCVImage (const Image& img)
{
    Mat res (img.getHeight (), img.getWidth (), CV_8SC3);
    res.data = (uchar*) img.getImageData ();
    return res;
}

// int SamplePlugin::search_closest(const std::vector<double> &sorted_array, double x)
// {
//     // Index of value in sorted_array that is closest to x (sorted_array[index] >= x):
//     auto iter_geq = std::lower_bound(
//         sorted_array.begin(),
//         sorted_array.end(),
//         x
//     );

//     // Dont look for value left of index 0 if x is lower than lowest value in sorted_array:
//     if (iter_geq == sorted_array.begin()) {
//         return 0;
//     }

//     // Pointer too closest values:
//     double a = *(iter_geq - 1);     // val < x
//     double b = *(iter_geq);         // val >= x

//     // Return index of value closest to x:
//     if (fabs(x - a) < fabs(x - b)) {
//         return iter_geq - sorted_array.begin() - 1;
//     }

//     return iter_geq - sorted_array.begin();
// }

void SamplePlugin::btnPressed ()
{
    QObject* obj = sender ();
    if (obj == _btn0) {
        //		log().info() << "Button 0\n";
        //		// Toggle the timer on and off
        //		if (!_timer25D->isActive())
        //		    _timer25D->start(100); // run 10 Hz
        //		else
        //			_timer25D->stop();
        _timer->stop ();
        rw::math::Math::seed ();
        double extend  = 0.05;
        double maxTime = 60;
        Q from (6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
        Q to (6, 1.847, -2.465, -1.602, -0.647, 1.571, 0);    // From pose estimation
        createPathRRTConnect (from, to, extend, maxTime);
    }
    else if (obj == _btn1) {
        log ().info () << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive ()) {
            _timer->start (100);    // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;
    }
    else if(obj == _btn2){
        std::cout << "Button 2 pressed = Pose Estimation: Sparse Stereo" << std::endl;
        poseEstimationSparseStereo();
    }
    else if(obj == _testSparseStereo3DPose){
        std::cout << "Button Pressed: Test Sparse Stereo 3D Pose Estimation" << std::endl;
        testBottle3DPoseEstimationSparseStereo("helhel.csv");
        // if (!_timer->isActive ()) {
        //     _timer->start (100);    // run 10 Hz
        // }
        // _step = 0;
    }
    else if (obj == _spinBox) {
        log ().info () << "spin value:" << _spinBox->value () << "\n";
    }
    else if (obj == _btn_im) {
        getImage ();
    }
    else if (obj == _btn_scan) {
        get25DImage ();
    }
}

void SamplePlugin::get25DImage ()
{
    if (_framegrabber25D != NULL) {
        for (size_t i = 0; i < _cameras25D.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame25D = _wc->findFrame (_cameras25D[i]);    // "Camera");
            _framegrabber25D->grab (cameraFrame25D, _state);

            // const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage ());

            std::ofstream output (_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth () << "\n";
            output << "HEIGHT " << img->getHeight () << "\n";
            output << "POINTS " << img->getData ().size () << "\n";
            output << "DATA ascii\n";
            for (const auto& p_tmp : img->getData ()) {
                rw::math::Vector3D< float > p = p_tmp;
                output << p (0) << " " << p (1) << " " << p (2) << "\n";
            }
            output.close ();
        }
    }
}

void SamplePlugin::getImage ()
{
    if (_framegrabber != NULL) {
        for (size_t i = 0; i < _cameras.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame (_cameras[i]);    // "Camera");
            _framegrabber->grab (cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage ());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat (rw_image->getHeight (),
                                     rw_image->getWidth (),
                                     CV_8UC3,
                                     (rw::sensor::Image*) rw_image->getImageData ());

            // Convert to OpenCV image
            Mat imflip, imflip_mat;
            cv::flip (image, imflip, 1);
            cv::cvtColor (imflip, imflip_mat, COLOR_RGB2BGR);

            cv::imwrite (_cameras[i] + ".png", imflip_mat);

            // Show in QLabel
            QImage img (imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p         = QPixmap::fromImage (img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));
        }
    }
    std::cout << "Camera_Right" << std::endl;
    printProjectionMatrix ("Camera_Right");
    std::cout << "Camera_Left" << std::endl;
    printProjectionMatrix ("Camera_Left");
}

void SamplePlugin::timer ()
{
    if (0 <= _step && (size_t) _step < _path.size ()) {
        _device->setQ (_path.at (_step), _state);
        getRobWorkStudio ()->setState (_state);
        _step++;
    }
}

void SamplePlugin::stateChangedListener (const State& state)
{
    _state = state;
}

bool SamplePlugin::checkCollisions (Device::Ptr device, const State& state,
                                    const CollisionDetector& detector, const Q& q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ (q, testState);
    colFrom = detector.inCollision (testState, &data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin (); it != fps.end (); it++) {
            cerr << (*it).first->getName () << " " << (*it).second->getName () << endl;
        }
        return false;
    }
    return true;
}

void SamplePlugin::createPathRRTConnect (Q from, Q to, double extend, double maxTime)
{
    _device->setQ (from, _state);
    getRobWorkStudio ()->setState (_state);
    CollisionDetector detector (_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    PlannerConstraint constraint = PlannerConstraint::make (&detector, _device, _state);
    QSampler::Ptr sampler        = QSampler::makeConstrained (QSampler::makeUniform (_device),
                                                       constraint.getQConstraintPtr ());
    QMetric::Ptr metric          = MetricFactory::makeEuclidean< Q > ();
    QToQPlanner::Ptr planner =
        RRTPlanner::makeQToQPlanner (constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    _path.clear ();
    if (!checkCollisions (_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions (_device, _state, detector, to))
        cout << to << " is in colission!" << endl;
    ;
    Timer t;
    t.resetAndResume ();
    planner->query (from, to, _path, maxTime);
    t.pause ();

    if (t.getTime () >= maxTime) {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

    const int duration = 10;

    if (_path.size () == 2) {    // The interpolated path between Q start and Q goal is collision
                                 // free. Set the duration with respect to the desired velocity
        LinearInterpolator< Q > linInt (from, to, duration);
        QPath tempQ;
        for (int i = 0; i < duration + 1; i++) {
            tempQ.push_back (linInt.x (i));
        }

        _path = tempQ;
    }
}

void SamplePlugin::printProjectionMatrix (std::string frameName)
{
    Frame* cameraFrame = _wc->findFrame (frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap ().has ("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan (fovy * (2 * M_PI) / 360.0 / 2.0);

            Eigen::Matrix< double, 3, 4 > KA;
            KA << fovy_pixel, 0, width / 2.0, 0, 0, fovy_pixel, height / 2.0, 0, 0, 0, 1, 0;

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            Transform3D<> camPosOGL   = cameraFrame->wTf (_state);
            Transform3D<> openGLToVis = Transform3D<> (RPY<> (-Pi, 0, Pi).toRotation3D ());
            Transform3D<> H           = inverse (camPosOGL * inverse (openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e () << std::endl;
        }
    }
}

// Mat SamplePlugin::addGaussianNoice(Mat& cam_img)
// {
//     cv::Mat imageCopy = cam_img.clone();
//     std::cout << "\nsigma: " << _sigma << std::endl;
//     int i = 0;
//     std::vector<double> p;
//     std::vector<double> p_cum;

//     for(int k=-255; k < 255; k++)
//     {
//         double p_k = exp(-pow(k,2.0)/(2.0*pow(_sigma,2.0))) / (_sigma * sqrt(2.0*M_PI));
//         p.push_back(p_k);
//         if(i==0)
//         {p_cum.push_back(p[i]);}
//         else
//         {p_cum.push_back(p_cum[i-1]+p[i]);}
//         i++;
//     }

//     cv::Vec3b SNR_signal = 0;
//     cv::Vec3b SNR_noise = 0;

//     for(int i = 0; i < imageCopy.rows; i++)
//     {
//         for(int j = 0; j < imageCopy.cols; j++)
//         {
//             double random_number = (double)rand()/(double)RAND_MAX;
//             int noise_val = -(256-1)+search_closest(p_cum,random_number);
//             cv::Vec3b noise(noise_val,noise_val,noise_val);
//             cv::Vec3b gaussian_noise = imageCopy.at<cv::Vec3b>(i,j) + noise;

//             for(size_t k = 0; k < 3; k++)
//             {
//                 if(gaussian_noise[k] >= 255)
//                 {
//                     gaussian_noise[k] = 256-1;
//                 }
//                 else if(gaussian_noise[k] <= 0)
//                 {
//                     gaussian_noise[k] = 0;
//                 }
//             }

//             imageCopy.at<cv::Vec3b>(i,j) = gaussian_noise;

//             SNR_signal += imageCopy.at<cv::Vec3b>(i,j).mul(imageCopy.at<cv::Vec3b>(i,j));
//             SNR_noise += gaussian_noise.mul(gaussian_noise);

//         }
//     }
//     std::cout << SNR_signal.div(SNR_noise) << std::endl;

//     return imageCopy;
// }

cv::Mat SamplePlugin::poseEstimationSparseStereo()
{
    // // Show Bottle image in QLabel
    // QImage img (_img_bottle.data, _img_bottle.cols, _img_bottle.rows, _img_bottle.step, QImage::Format_BGR888);
    // QPixmap p         = QPixmap::fromImage (img);
    // unsigned int maxW = 480;
    // unsigned int maxH = 640;
    // _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));

    std::vector<cv::Mat> camera_images;
    std::vector<cv::Point> circle_center_points;

    // // _bottle_keypoint_image
    // // Show Bottle image in QLabel
    // QImage img (_bottle_keypoint_image.data, _bottle_keypoint_image.cols, _bottle_keypoint_image.rows, _bottle_keypoint_image.step, QImage::Format_Grayscale8);
    // QPixmap p         = QPixmap::fromImage (img);
    // unsigned int maxW = 480;
    // unsigned int maxH = 640;
    // _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));

    int gauss_kernel_size = 7;
    double sigma = 1.5;

    int min_radius = 15;
    int max_radius = 22;
    
    if (_framegrabber != NULL) {
        for (size_t i = 0; i < _cameras.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame (_cameras[i]);    // "Camera");
            _framegrabber->grab (cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage ());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat (rw_image->getHeight (),
                                     rw_image->getWidth (),
                                     CV_8UC3,
                                     (rw::sensor::Image*) rw_image->getImageData ());

            // Convert to OpenCV image
            Mat imflip, imflip_mat;
            cv::flip (image, imflip, 1);
            cv::cvtColor (imflip, imflip_mat, COLOR_RGB2BGR);

            // Save images
            cv::imwrite (_cameras[i] + ".png", imflip_mat);
            camera_images.push_back(imflip_mat);

            // Convert camera images to grayscale:
            cv::Mat camera_img_blurred_gray;
            cv::cvtColor(camera_images[i], camera_img_blurred_gray, cv::COLOR_BGR2GRAY);

            // // add noise to camera image:
            // //camera_img_blurred_gray = addGaussianNoice(camera_img_blurred_gray);
            // addGaussianNoice(camera_img_blurred_gray);
            // cv::imwrite (_cameras[i] + "_noice.png", camera_img_blurred_gray);

            // Blurr the images with a Gaussian filter:
            cv::GaussianBlur(camera_img_blurred_gray, camera_img_blurred_gray, cv::Size(gauss_kernel_size,gauss_kernel_size),sigma,sigma);

            // Find circles with a radius of [min_radius, max_radius] in the image by using hough transform:
            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(camera_img_blurred_gray, circles, cv::HOUGH_GRADIENT, 2, camera_img_blurred_gray.rows/2,
                                100, 30, min_radius, max_radius);

            // Save the center point of the circle with the most votes - it's the first circle in the list returned by HoughCircles()
            if(circles.size() > 0){
                cv::Vec3i c = circles[0];
                cv::Point center = Point(cvRound(c[0]), cvRound(c[1]));
                int radius = cvRound(c[2]);
                // Draw circle center:
                cv::circle(camera_images[i], center, 3, cv::Scalar(0,250,0), -1, 8, 0);
                // Draw circle outline:
                cv::circle(camera_images[i], center, radius, cv::Scalar(0,0,250),3, 8,0);
                circle_center_points.push_back(center);
                std::cout << "\nCircle radius=" << radius << std::endl;
            }
            
            // for(size_t idx_circle = 0; idx_circle < circles.size(); idx_circle++){
            //     cv::Vec3i c = circles[idx_circle];
            //     cv::Point center = Point(cvRound(c[0]), cvRound(c[1]));
            //     int radius = cvRound(c[2]);
            //     // Draw circle center:
            //     cv::circle(camera_images[i], center, 3, cv::Scalar(0,250,0), -1, 8, 0);
            //     // Draw circle outline:
            //     cv::circle(camera_images[i], center, radius, cv::Scalar(0,0,250),3, 8,0);

            //     circle_center_points.push_back(center);

            //     std::cout << "\nCircle radius=" << radius << std::endl;
            // }

            cv::imwrite (_cameras[i] + "_circles.png", camera_images[i]);

            // Show in QLabel
            QImage img (imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p         = QPixmap::fromImage (img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));
        }
    }

    // std::cout << "Camera_Right" << std::endl;
    // printProjectionMatrix ("Camera_Right");
    // std::cout << "Camera_Left" << std::endl;
    // printProjectionMatrix ("Camera_Left");

    // Triangulate points:
    cv::Mat pnts3D(1,1,CV_64FC4);
    cv::Mat camera_right_points(1,1,CV_64FC2);
    cv::Mat camera_left_points(1,1,CV_64FC2);
    camera_right_points.at<cv::Vec2d>(0)[0] = circle_center_points[0].x;
    camera_right_points.at<cv::Vec2d>(0)[1] = circle_center_points[0].y;
    camera_left_points.at<cv::Vec2d>(0)[0] = circle_center_points[1].x;
    camera_left_points.at<cv::Vec2d>(0)[1] = circle_center_points[1].y;
    
    triangulatePoints(_proj_left, _proj_right, camera_left_points, camera_right_points, pnts3D);
    std::cout << "Image points: " << camera_right_points  << "\t"  << camera_left_points << std::endl;
    std::cout << "Triangulated point (normalized): " << pnts3D / pnts3D.at<double>(3,0) << std::endl;

    return pnts3D / pnts3D.at<double>(3,0);
}

void SamplePlugin::testBottle3DPoseEstimationSparseStereo(std::string output_filename)
{
    // Find the bottle frame: (written by Ida Blirup Skov)
    MovableFrame::Ptr _bottleFrame = _wc->findFrame< MovableFrame > ("Bottle");
    if (_bottleFrame != NULL) {
        _bottleFrame->setTransform (Transform3D<> (Vector3D<> (_bottleFrame->getTransform (_state).P () + Vector3D<> (0.25, 0, 0)),
                                              RPY<> (0, 0, 90 * Deg2Rad)),
                               _state);
        getRobWorkStudio ()->setState (_state);
        std::cout << "\nMoved the bottle" << std::endl;
    }
    else{
        std::cout << "\nCOULD not find movable frame Bottle ... check model" << std::endl;
    }

    cv::Mat pnts3Dtrue(1,1,CV_64FC4);
    // pnts3Dtrue.at<cv::Vec4d>(0)=cv::Vec4d(0.25,0.474,0.21,1.0);
    pnts3Dtrue.at<double>(0,0)=0.25;
    pnts3Dtrue.at<double>(1,0)=0.474;
    pnts3Dtrue.at<double>(2,0)=0.21;
    pnts3Dtrue.at<double>(3,0)=1.0;

    std::ofstream outfile;
    outfile.open("pose_estimation_sparse_stereo.csv");
    outfile << "sigma,x,y,z,x_est,y_est,z_est";

    double dx = -0.05;
    double sigma = 1.5;

    for(int i = 0; i < 9; i++){
        outfile << "\n" << sigma;
        // Move the bottle to a different position in the scene:
        _bottleFrame->setTransform (Transform3D<> (Vector3D<> (_bottleFrame->getTransform (_state).P () + Vector3D<> (dx, 0, 0)),
                                              RPY<> (0, 0, 90 * Deg2Rad)),
                               _state);
        
        //std::cout << "\nMoved the bottle" << std::endl;

        // update the state in RobWorkStudio:
        getRobWorkStudio ()->setState (_state);

        pnts3Dtrue.at<double>(0,0) = pnts3Dtrue.at<double>(0,0)+dx;
        outfile << "," << pnts3Dtrue.at<double>(0,0)
                    << "," << pnts3Dtrue.at<double>(1,0)
                    << "," << pnts3Dtrue.at<double>(2,0);

        std::cout << "\n" << pnts3Dtrue.at<double>(0,0)
                    << "," << pnts3Dtrue.at<double>(1,0)
                    << "," << pnts3Dtrue.at<double>(2,0) << std::endl;

        // Get center of the top of the bottle
        cv::Mat pnts3Dnorm(1,1,CV_64FC4);
        pnts3Dnorm = poseEstimationSparseStereo();
        // std::cout << "Triangulated point (normalized): " << poseEstimationSparseStereo()<< std::endl;
        // std::cout << "3D Point:\n\t" << pnts3Dnorm.at<double>(0,0)
        //             << pnts3Dnorm.at<double>(1,0)
        //             << pnts3Dnorm.at<double>(2,0)
        //             << std::endl;

        outfile  << "," << pnts3Dnorm.at<double>(0,0)
                    << "," << pnts3Dnorm.at<double>(1,0)
                    << "," << pnts3Dnorm.at<double>(2,0);
        
    }
    outfile.close();
}
