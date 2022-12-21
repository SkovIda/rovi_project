#include "SamplePlugin.hpp"

#include <filesystem>

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
    connect (_btn2, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_spinBox, SIGNAL (valueChanged (int)), this, SLOT (btnPressed ()));

    _framegrabber = NULL;

    _cameras    = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};

    cv::Mat KA(3,4,CV_64F);
    KA.at<double>(0,0) = 514.682;
    KA.at<double>(0,2) = 320.0;
    KA.at<double>(0,3) = 0.0;
    KA.at<double>(1,1) = 514.682;
    KA.at<double>(1,2) = 240.0;
    KA.at<double>(2,2) = 1.0;
    cv::print(KA);

    cv::Mat H_camera_right(4,4,CV_64F);
    H_camera_right.at<cv::Vec4d>(0) = cv::Vec4d(-1.0, -1.22465e-16, -1.49976e-32, -0.05);
    H_camera_right.at<cv::Vec4d>(1) = cv::Vec4d(-1.22465e-16, 1.0, 1.22465e-16, -0.474);
    H_camera_right.at<cv::Vec4d>(2) = cv::Vec4d(0.0, 1.22465e-16, -1.0, 0.925);
    H_camera_right.at<cv::Vec4d>(3) = cv::Vec4d(0.0, 0.0, 0.0, 1.0);
    cv::print(H_camera_right);

    cv::Mat H_camera_left(4,4,CV_64F);
    H_camera_left.at<cv::Vec4d>(0) = cv::Vec4d(-1.0, -1.22465e-16, -1.49976e-32, 0.15);
    H_camera_left.at<cv::Vec4d>(1) = cv::Vec4d(-1.22465e-16, 1.0, 1.22465e-16, -0.474);
    H_camera_left.at<cv::Vec4d>(2) = cv::Vec4d(0.0, 1.22465e-16, -1.0, 0.925);
    H_camera_left.at<cv::Vec4d>(3) = cv::Vec4d(0.0, 0.0, 0.0, 1.0);
    cv::print(H_camera_left);

    // cv::Mat H_camera_right(4,4,CV_64F);
    // H_camera_right.at<cv::Vec4d>(0) = cv::Vec4d(-0.984808,-1.22465e-16,-0.173648,0.0129034);
    // H_camera_right.at<cv::Vec4d>(1) = cv::Vec4d(-1.4187e-16,1,9.93384e-17,-0.474);
    // H_camera_right.at<cv::Vec4d>(2) = cv::Vec4d(0.173648,1.22465e-16,-0.984808,0.936994);
    // H_camera_right.at<cv::Vec4d>(3) = cv::Vec4d(0, 0, 0, 1);
    // cv::print(H_camera_right);

    // cv::Mat H_camera_left(4,4,CV_64F);
    // H_camera_left.at<cv::Vec4d>(0) = cv::Vec4d(-0.984808,-1.22465e-16,-0.173648,-0.0129034);
    // H_camera_left.at<cv::Vec4d>(1) = cv::Vec4d(-1.4187e-16,1,9.93384e-17,-0.474);
    // H_camera_left.at<cv::Vec4d>(2) = cv::Vec4d(0.173648,1.22465e-16,-0.984808,0.936994);
    // H_camera_left.at<cv::Vec4d>(3) = cv::Vec4d(0, 0, 0, 1.0);
    // cv::print(H_camera_left);

    _proj_right = KA * H_camera_right;
    _proj_left = KA * H_camera_left;

    // KA[0][3] = 0;
    // _proj_left = 

    // std::filesystem::path bottle_img_path (__FILE__);
    // bottle_img_path = bottle_img_path.parent_path() / "../bottle_img.png";
    // _img_bottle = cv::imread(bottle_img_path, cv::IMREAD_COLOR);
    // if(_img_bottle.empty()){
    //     std::cout << "COULD NOT LOAD TRAIN IMAGE: bottle_img.png" << std::endl;
    // }
    // else{
    //     _detector = cv::ORB::create(10000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 21, 20);
    //     _extractor = cv::ORB::create();
    //     _detector->detect(_img_bottle, _bottle_keypoints);
    //     _extractor->compute(_img_bottle, _bottle_keypoints, _bottle_descriptors);

    //     cv::drawKeypoints(_img_bottle, _bottle_keypoints, _bottle_keypoint_image);

    //     //_matcher = _matcher(cv::NORM_HAMMING, _cross_check_matches);
    // }
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


// cv::Mat SamplePlugin::getProjectionMatrix(std::string frameName){
//     cv::Mat projection_matrix(4,4,CV_64F);
//     Frame* cameraFrame = _wc->findFrame (frameName);
//     if (cameraFrame != NULL) {
//         if (cameraFrame->getPropertyMap ().has ("Camera")) {
//             // Read the dimensions and field of view
//             double fovy;
//             int width, height;
//             std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
//             std::istringstream iss (camParam, std::istringstream::in);
//             iss >> fovy >> width >> height;

//             double fovy_pixel = height / 2 / tan (fovy * (2 * M_PI) / 360.0 / 2.0);

//             Eigen::Matrix< double, 3, 4 > KA;
//             KA << fovy_pixel, 0, width / 2.0, 0, 0, fovy_pixel, height / 2.0, 0, 0, 0, 1, 0;

//             // std::cout << "Intrinsic parameters:" << std::endl;
//             // std::cout << KA << std::endl;
//             Transform3D<> camPosOGL   = cameraFrame->wTf (_state);
//             Transform3D<> openGLToVis = Transform3D<> (RPY<> (-Pi, 0, Pi).toRotation3D ());
//             Transform3D<> H           = inverse (camPosOGL * inverse (openGLToVis));

//             // std::cout << "Extrinsic parameters:" << std::endl;
//             // std::cout << H.e () << std::endl;

//             projection_matrix.at<cv::Vec4>(0)[0] = H(0,0);
//         }
//     }
//     return projection_matrix;
// }

void SamplePlugin::poseEstimationSparseStereo()
{
    // Show Bottle image in QLabel
    QImage img (_img_bottle.data, _img_bottle.cols, _img_bottle.rows, _img_bottle.step, QImage::Format_BGR888);
    QPixmap p         = QPixmap::fromImage (img);
    unsigned int maxW = 480;
    unsigned int maxH = 640;
    _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));

    std::vector<cv::Mat> camera_images;
    std::vector<cv::Point> circle_center_points;

    // cv::Scalar low_blue = cv::Scalar(55, 0, 0);
    // cv::Scalar high_blue = cv::Scalar(188,255,255);
    //double threshold = 100.0;

    // // _bottle_keypoint_image
    // // Show Bottle image in QLabel
    // QImage img (_bottle_keypoint_image.data, _bottle_keypoint_image.cols, _bottle_keypoint_image.rows, _bottle_keypoint_image.step, QImage::Format_Grayscale8);
    // QPixmap p         = QPixmap::fromImage (img);
    // unsigned int maxW = 480;
    // unsigned int maxH = 640;
    // _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));

    int gauss_kernel_size = 7;
    int sigma = 1.5;
    
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

            // /////////////////////////////////////////////////////
            // cv::Mat hsv_cam_img;
            // cv::cvtColor(camera_images[i], hsv_cam_img, COLOR_BGR2HSV);

            // cv::Mat img_thresholded;
            // cv::inRange(hsv_cam_img, low_blue, high_blue, img_thresholded);
            // double threshold;
            // //cv::threshold(camera_images[i], img_thresholded, threshold, 255, cv::THRESH_OTSU);
            // //std::cout << "\nThreshold: " << threshold << std::endl;

            // // // Morphological opening:
            // // cv::erode(img_thresholded, img_thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
            // // cv::dilate(img_thresholded, img_thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

            // // // Morphological closing:
            // // cv::dilate(img_thresholded, img_thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
            // // cv::erode(img_thresholded, img_thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

            // cv::imwrite (_cameras[i] + "_thresholded.png", img_thresholded);

            // //binary_camera_images.push_back(img_thresholded);
            
            // // cv::cvtColor(img_thresholded, img_thresholded, cv::COLOR_HSV2GRAY);

            // cv::Mat camera_img_blurred_gray;
            // cv::cvtColor(camera_img_blurred_gray, img_thresholded, cv::COLOR_HSV2BGR);
            // cv::cvtColor(camera_img_blurred_gray, camera_img_blurred_gray, cv::COLOR_BGR2GRAY);
            // cv::imwrite (_cameras[i] + "_after_thresh_HSV2GRAY.png", img_thresholded);
            // ///////////////////////////////////////////////////

            // Convert camera images to grayscale and blurr
            cv::Mat camera_img_blurred_gray;
            cv::cvtColor(camera_images[i], camera_img_blurred_gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(camera_img_blurred_gray, camera_img_blurred_gray, cv::Size(gauss_kernel_size,gauss_kernel_size),sigma,sigma);

            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(camera_img_blurred_gray, circles, cv::HOUGH_GRADIENT, 2, camera_img_blurred_gray.rows/2,
                                100, 30, 15, 22);


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

            // // cv::cvtColor (imflip_mat, image, COLOR_RGB2GRAY);

            // // detecting keypoints
            // std::vector<cv::KeyPoint> keypoints;
            // cv::Mat descriptors;
            // _detector->detect(imflip_mat, keypoints);
            // _extractor->compute(imflip_mat, keypoints, descriptors);
            
            // std::vector<cv::DMatch> matches;
            // cv::BFMatcher matcher(cv::NORM_HAMMING, _cross_check_matches);

            // matcher.match(descriptors, _bottle_descriptors, matches);

            // cv::Mat img_matches;
            // cv::drawMatches(imflip_mat, keypoints, _img_bottle, _bottle_keypoints, matches, img_matches);

            // cv::imwrite (_cameras[i] + "_matches.png", img_matches);

            // std::vector<cv::Point2f> bottle_points, camera_view_points;
            // for (const auto &m : matches) {
            //     bottle_points.push_back(_bottle_keypoints[m.trainIdx].pt);
            //     camera_view_points.push_back(keypoints[m.queryIdx].pt);
            // }

            // std::vector<char> inlier_mask;
            // cv::findHomography(bottle_points, camera_view_points, cv::RANSAC, 2.0, inlier_mask, 10000);
            // const int n_inliers = cv::sum(inlier_mask)[0];
            // std::cout << "Number of robust inliers in " + _cameras[i] + ": " << n_inliers << std::endl;

            // cv::print(bottle_points);
            // std::cout << std::endl;
            // cv::print(camera_view_points);
            // for (const char &pt : inlier_mask){
            //     std::cout << "\t" << to_string(pt) << std::endl;
            // }

            // cv::Mat img_robust_matches;
            // cv::drawMatches(imflip_mat, keypoints, _img_bottle, _bottle_keypoints, matches, img_robust_matches,
            //                             cv::Scalar::all(-1), cv::Scalar::all(-1), inlier_mask,
            //                             cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            // cv::imwrite (_cameras[i] + "_robust_matches.png", img_robust_matches);


            // // // Show in QLabel
            // // QImage img (imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            // // QPixmap p         = QPixmap::fromImage (img);
            // // unsigned int maxW = 480;
            // // unsigned int maxH = 640;
            // // _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));


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

    // Triangulate points:
    cv::Mat pnts3D(1,1,CV_64FC4);
    cv::Mat cam0pnts(1,1,CV_64FC2);
    cv::Mat cam1pnts(1,1,CV_64FC2);
    cam0pnts.at<cv::Vec2d>(0)[0] = circle_center_points[0].x;
    cam0pnts.at<cv::Vec2d>(0)[1] = circle_center_points[0].y;
    cam1pnts.at<cv::Vec2d>(0)[0] = circle_center_points[1].x;
    cam1pnts.at<cv::Vec2d>(0)[1] = circle_center_points[1].y;
    
    triangulatePoints(_proj_left, _proj_right, cam1pnts, cam0pnts, pnts3D);
    std::cout << "Image points: " << cam0pnts << "\t" << cam1pnts << std::endl;
    std::cout << "Triangulated point (normalized): " << pnts3D / pnts3D.at<double>(3,0) << std::endl;


    // cv::Scalar low_blue = cv::Scalar(55, 0, 0);
    // cv::Scalar high_blue = cv::Scalar(188,255,255);

    // double threshold = 100.0;
    // std::vector<cv::Mat> binary_camera_images;
    // for (size_t i = 0; i < _cameras.size (); i++) {
        // cv::Mat hsv_cam_img;
        // cv::cvtColor(camera_images[i], hsv_cam_img, COLOR_BGR2HSV);

        // cv::Mat img_thresholded;
        // cv::inRange(hsv_cam_img, low_blue, high_blue, img_thresholded);
        // // cv::threshold(camera_images[i], binary_thresholded, threshold, 255, cv::THRESH_OTSU);

        // // Morphological opening:
        // cv::erode(img_thresholded, img_thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
        // cv::dilate(img_thresholded, img_thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

        // // Morphological closing:
        // cv::dilate(img_thresholded, img_thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
        // cv::erode(img_thresholded, img_thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

        // cv::imwrite (_cameras[i] + "_thresholded.png", img_thresholded);

        // binary_camera_images.push_back(img_thresholded);
        
        // cv::cvtColor(img_thresholded, img_thresholded, cv::COLOR_BGR2GRAY);


//////////////////////////////////////////////////////
        // cv::Mat camera_img_blurred_gray;
        // cv::cvtColor(camera_images[i], camera_img_blurred_gray, cv::COLOR_BGR2GRAY);
        // cv::GaussianBlur(camera_img_blurred_gray, camera_img_blurred_gray, cv::Size(9,9),2,2);

        // std::vector<cv::Vec3f> circles;
        // cv::HoughCircles(camera_img_blurred_gray, circles, cv::HOUGH_GRADIENT, 2, camera_img_blurred_gray.rows/2,
        //                     200, 75); //
        
        // for(size_t idx_circle = 0; idx_circle < circles.size(); idx_circle++){
        //     cv::Vec3i c = circles[idx_circle];
        //     cv::Point center = Point(cvRound(c[0]), cvRound(c[1]));
        //     int radius = cvRound(c[2]);
        //     // Draw circle center:
        //     cv::circle(camera_images[i], center, 3, cv::Scalar(0,250,0), -1, 8, 0);
        //     // Draw circle outline:
        //     cv::circle(camera_images[i], center, radius, cv::Scalar(0,0,250),3, 8,0);
        // }

        // cv::imwrite (_cameras[i] + "_circles.png", camera_images[i]);
////////////////////////////////////////////////////////////



        // std::vector<std::vector<cv::Point>> contours;
        // std::vector<cv::Vec4i> hierarchy;
        // cv::findContours(binary_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        // cv::Mat contours_image = camera_images[i].clone();
        // for(size_t contour = 0; contour < contours.size(); contour++){
        //     cv::Scalar color( rand()&0xFF, rand()&0xFF, rand()&0xFF);
        //     cv::drawContours(contours_image, contours, contour, color, cv::FILLED, 8, hierarchy);
        // }
        

        // std::vector<std::vector<cv::Point>> circles;
        // cv::Mat circle_image = camera_images[i].clone();
        // for(size_t contour = 0; contour < contours.size(); contour++){
        //     double area = contourArea(contours[contour]);
        //     double arc_length = cv::arcLength(contours[contour],true);
        //     double circularity = 4*CV_PI * area / (arc_length*arc_length);
        //     if(circularity > 0.88){
        //         std::cout << "Circle found! contour #" << contour << "\ncircularity: " << circularity << std::endl;
        //         circles.push_back(contours[contour]);

        //         cv::Scalar color(10*contour, 0, 0);
        //         cv::drawContours(circle_image, contours, contour, color, 1, 8, hierarchy);
        //     }
        // }

        
    // }

    // // detecting keypoints
    // std::vector<cv::KeyPoint> keypoints0, keypoints1;
    // cv::Mat descriptors0, descriptors1;
    // _detector->detect(camera_images[0], keypoints0);
    // _extractor->compute(camera_images[0], keypoints0, descriptors0);

    // _detector->detect(camera_images[1], keypoints1);
    // _extractor->compute(camera_images[1], keypoints1, descriptors1);
    
    // std::vector<cv::DMatch> matches;
    // cv::BFMatcher matcher(cv::NORM_HAMMING, _cross_check_matches);

    // matcher.match(descriptors0, descriptors1, matches);

    // cv::Mat img_matches;
    // cv::drawMatches(camera_images[0], keypoints0, camera_images[1], keypoints1, matches, img_matches);

    // cv::imwrite ("matches_between_Right_Left_cameras.png", img_matches);

    // std::vector<cv::Point2f> camera0_view_points, camera1_view_points;
    // for (const auto &m : matches) {
    //     camera0_view_points.push_back(keypoints1[m.trainIdx].pt);
    //     camera1_view_points.push_back(keypoints1[m.queryIdx].pt);
    // }

    // std::vector<char> inlier_mask;
    // cv::findHomography(camera0_view_points, camera1_view_points, cv::RANSAC, 2.0, inlier_mask, 10000);
    // const int n_inliers = cv::sum(inlier_mask)[0];
    // std::cout << "Number of robust inliers: " << n_inliers << std::endl;

    // cv::Mat img_robust_matches;
    // cv::drawMatches(camera_images[0], keypoints0, camera_images[1], keypoints1, matches, img_robust_matches,
    //                             cv::Scalar::all(-1), cv::Scalar::all(-1), inlier_mask,
    //                             cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imwrite ("robust_matches_between_Right_Left_cameras.png", img_robust_matches);

}
