#include "tools.h"

int getch(void) {
    struct termios tm, tm_old;
    int            fd = 0, ch;

    if (tcgetattr(fd, &tm) < 0) {  // Save the current terminal settings
        return -1;
    }

    tm_old = tm;
    cfmakeraw(&tm);                        // Change the terminal settings to raw mode, in which all input data is processed in bytes
    if (tcsetattr(fd, TCSANOW, &tm) < 0) {  // Settings after changes on settings
        return -1;
    }

    ch = getchar();
    if (tcsetattr(fd, TCSANOW, &tm_old) < 0) {  // Change the settings to what they were originally
        return -1;
    }

    return ch;
}

int kbhit(void) {
    struct termios oldt, newt;
    int            ch;
    int            oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

std::string getCurrentDateTime()
{
    // Get current time as time_point
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    // Format time to "YYYY-MM-DD_HH-MM"
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M");

    return ss.str();
}

std::vector<OBColorPoint> getRGBPointsFromFrame(std::shared_ptr<ob::Frame> frame)
{
    std::vector<OBColorPoint> points;
    int pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    points.reserve(pointsSize);

    OBColorPoint* point = (OBColorPoint*)frame->data();
    for (int i = 0; i < pointsSize; ++point, ++i) {
        if (point->x == 0 && point->y == 0 && point->z == 0) {
            continue;
        }
        points.emplace_back(*point);
    }
    return points;
}

// Save colored point cloud data to ply
void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string dirName, std::string fileName) {
    std::vector<OBColorPoint> points = getRGBPointsFromFrame(frame);
    int pointsSize = points.size();

    auto path = dirName + "/" + fileName;
    FILE* fp = fopen(path.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    for (int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", points[i].x, points[i].y, points[i].z, (int)points[i].r, (int)points[i].g, (int)points[i].b);
    }

    fflush(fp);
    fclose(fp);
}

void saveRGBPointsToPly(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud, std::string dirName, std::string fileName)
{
    auto path = dirName + "/" + fileName;

    FILE* fp = fopen(path.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointCloud->size());
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    for (int i = 0; i < pointCloud->size(); ++i) {
        pcl::PointXYZRGB& pt = pointCloud->at(i);
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", pt.x, pt.y, pt.z, pt.r, pt.g, pt.b);
    }
    fflush(fp);
    fclose(fp);
}

// std::string getAppRootPath() {
//     char appPath[MAX_PATH];
//     GetModuleFileName(NULL, appPath, MAX_PATH); // Get the full path to the executable
//     std::string fullPath(appPath);
//     size_t pos = fullPath.find_last_of("\\/");
//     return fullPath.substr(0, pos); // Return the directory containing the executable
// }

// int CALLBACK BrowseCallbackProc(HWND hwnd, UINT uMsg, LPARAM lParam, LPARAM lpData) {
//     if (uMsg == BFFM_INITIALIZED) {
//         // Set the initial folder when the dialog is initialized
//         std::string appRootPath = reinterpret_cast<const char*>(lpData);
//         SendMessage(hwnd, BFFM_SETSELECTION, TRUE, (LPARAM)appRootPath.c_str());
//     }
//     return 0;
// }

// std::string askForDirectory(const std::string & title) {
//     BROWSEINFO bi = { 0 };
//     TCHAR path[MAX_PATH];

//     std::string appRootPath = getAppRootPath(); // Get the root directory of the app

//     bi.lpszTitle = (LPCSTR)title.c_str();
//     bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE | BIF_USENEWUI;
//     bi.pszDisplayName = path;
//     bi.lpfn = BrowseCallbackProc;  // Set the callback function
//     bi.lParam = (LPARAM)appRootPath.c_str(); // Pass the app root path to the callback

//     // Display the directory selection dialog
//     LPITEMIDLIST pidl = SHBrowseForFolder(&bi);

//     if (pidl != nullptr) {
//         // Get the selected path
//         if (SHGetPathFromIDList(pidl, path)) {
//             CoTaskMemFree(pidl); // Free memory allocated for the path
//             return std::string(path);
//         }
//         CoTaskMemFree(pidl); // Free memory if path extraction fails
//     }

//     return std::string();
// }


std::vector<std::string> getFilesInDirectory(const std::string& directory, const std::string& extension)
{
    std::vector<std::string> paths;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
    {
        if (entry.path().extension() == extension)
        {
            paths.push_back(entry.path().string());
        }
    }
    return paths;
}

IMU_PoseEstimator::IMU_PoseEstimator()
    : currentOrientation(Eigen::Quaternionf::Identity())
    , currentPosition(Eigen::Vector3f::Zero())
    , currentVelocity(Eigen::Vector3f::Zero())
    , gyroBias(Eigen::Vector3f::Zero())
    , gravity(0, 0, -9.81)
    , __started(false)
    , lastGyroTimestamp(0)
    , lastAccelTimestamp(0)
{
}

void IMU_PoseEstimator::Start()
{
    if (__started) return;
    std::cout << "Starting IMU Pose Estimation..." << std::endl;
    __started = true;
}

void IMU_PoseEstimator::GiveGyroscopeData(const OBGyroValue& gyro, uint64_t timestamp)
{
    //if (!__started) return;
    
    if (gyroBias == Eigen::Vector3f::Zero()) {
        gyroBias = Eigen::Vector3f(gyro.x, gyro.y, gyro.z);
        printf("Gyro bias: (%f, %f, %f)\n", gyroBias.x(), gyroBias.y(), gyroBias.z());
    }
    UpdateOrientationFromGyro(gyro, timestamp);
}

void IMU_PoseEstimator::GiveAccelerometerData(const OBAccelValue& accel, uint64_t timestamp)
{
    //if (!__started) return;
    
    //UpdateOrientationFromAccel(accel);
    UpdatePosition(accel, timestamp);
}

void IMU_PoseEstimator::UpdateOrientationFromGyro(const OBGyroValue& gyro, uint64_t timestamp)
{
    if (lastGyroTimestamp == 0) {
        lastGyroTimestamp = timestamp;
        return;
    }

    float deltaTime = (timestamp - lastGyroTimestamp) / 1000.0f; // Convert milliseconds to seconds
    lastGyroTimestamp = timestamp;

    Eigen::Vector3f angularVelocity(gyro.x, gyro.y, gyro.z);
    angularVelocity -= gyroBias;

    Eigen::Quaternionf deltaOrientation(1, 0.5f * angularVelocity.x() * deltaTime, 0.5f * angularVelocity.y() * deltaTime, 0.5f * angularVelocity.z() * deltaTime);

    // Correction
    if (abs(deltaOrientation.x()) < 0.00001f) deltaOrientation.x() = 0;
    if (abs(deltaOrientation.y()) < 0.00001f) deltaOrientation.y() = 0;
    if (abs(deltaOrientation.z()) < 0.00001f) deltaOrientation.z() = 0;

    currentOrientation = currentOrientation * deltaOrientation;
    currentOrientation.normalize();
}

void IMU_PoseEstimator::UpdateOrientationFromAccel(const OBAccelValue& accel)
{
    Eigen::Vector3f accelVector(accel.x, accel.y, accel.z);
    accelVector.normalize();

    Eigen::Quaternionf accelOrientation = GetAccelOrientation(accelVector);

    float alpha = 0.98f;
    currentOrientation = currentOrientation.slerp(1 - alpha, accelOrientation);
    currentOrientation.normalize();
}

Eigen::Quaternionf IMU_PoseEstimator::GetAccelOrientation(const Eigen::Vector3f& accel)
{
    float roll = atan2(accel.y(), accel.z());
    float pitch = atan2(-accel.x(), sqrt(accel.y() * accel.y() + accel.z() * accel.z()));

    return Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
}

void IMU_PoseEstimator::UpdatePosition(const OBAccelValue& accel, uint64_t timestamp)
{
    if (lastAccelTimestamp == 0) {
        lastAccelTimestamp = timestamp;
        return;
    }

    float deltaTime = (timestamp - lastAccelTimestamp) / 1000.0f; // Convert milliseconds to seconds
    lastAccelTimestamp = timestamp;

    Eigen::Vector3f localAccel(accel.x, accel.y, accel.z);
    Eigen::Vector3f worldAccel = currentOrientation * localAccel - gravity;

    currentPosition += currentVelocity * deltaTime + 0.5f * worldAccel * deltaTime * deltaTime;
    currentVelocity += worldAccel * deltaTime;
}

PointCloudVisualizer::PointCloudVisualizer()
    : __cloudRealtime(new pcl::PointCloud<pcl::PointXYZRGB>())
    , __cloudReconstruction(nullptr)
    , __viewer(new pcl::visualization::PCLVisualizer("Human Face Reconstruction"))
    , __shouldQuit(false)
    , __imuPoseEstimator(nullptr)
{
    __viewer->initCameraParameters();

    // Create two viewports: left for real-time, right for reconstruction
    __viewer->createViewPort(0.0, 0.0, 0.5, 1.0, __realtimeViewport);        // Left half for real-time point cloud
    __viewer->createViewPort(0.5, 0.0, 1.0, 1.0, __reconstructionViewport);  // Right half for reconstruction point cloud
    __viewer->setBackgroundColor(0.254, 0.262, 0.415, __realtimeViewport);          // Set background color for the left viewport
    __viewer->setBackgroundColor(0.592, 0.250, 0.388, __reconstructionViewport);          // Set background color for the right viewport

    bool res;
    // Add real-time point cloud to the left viewport
    __viewer->addText("Real-time Camera View", 10, 10, 20, 1, 1, 1, "RealtimeText", __realtimeViewport);
    // Reverse Up Y-axis because of Gemini 2
    __viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0, __realtimeViewport);

    // Add reconstruction point cloud to the right viewport
    __viewer->addText("Face Reconstruction", 10, 10, 20, 1, 1, 1, "ReconstructionText", __reconstructionViewport);
    // Reverse Up Y-axis because of Gemini 2
    __viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0, __reconstructionViewport);

    // Text for position, rotation, ...
    __viewer->addText("Position: (0, 0, 0)", 10, 30, 10, 1, 1, 1, "PositionText", __realtimeViewport);
    __viewer->addText("Rotation: (0, 0, 0)", 10, 50, 10, 1, 1, 1, "RotationText", __realtimeViewport);
    __viewer->addText("Accelerometer: (0, 0, 0)", 10, 70, 10, 1, 1, 1, "AccelerometerText", __realtimeViewport);
    __viewer->addText("Gyroscope: (0, 0, 0)", 10, 90, 10, 1, 1, 1, "GyroscopeText", __realtimeViewport);

}

void PointCloudVisualizer::UpdatePointCloudRealtime(const std::shared_ptr<ob::Frame>& frame)
{
    Timer timer("finv");
    std::vector<OBColorPoint> points = getRGBPointsFromFrame(frame);
    int pointsSize = points.size();
    timer.PrintTimePassed();

    __cloudRealtime->clear();
    __cloudRealtime->points.reserve(pointsSize);
    for (int i = 0; i < pointsSize; i++) {
        __cloudRealtime->points.emplace_back(
            points[i].x, points[i].y, points[i].z, points[i].r, points[i].g, points[i].b
        );
    }
    timer.PrintTimePassed();

    if (!__viewer->updatePointCloud(__cloudRealtime, "PointCloudRealtime"))
    {
        __viewer->addPointCloud<pcl::PointXYZRGB>(__cloudRealtime, "PointCloudRealtime", __realtimeViewport);
        __viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "PointCloudRealtime");
    }
    timer.PrintTimePassed();
    __viewer->spinOnce(1, true);
    timer.PrintTimePassed();
}

void PointCloudVisualizer::UpdatePointCloudRealtime(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    __cloudRealtime = cloud;
    if (!__viewer->updatePointCloud(__cloudRealtime, "PointCloudRealtime"))
    {
        __viewer->addPointCloud<pcl::PointXYZRGB>(__cloudRealtime, "PointCloudRealtime", __realtimeViewport);
        __viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "PointCloudRealtime");
    }
    __viewer->spinOnce(1, true);

    // Update Camera position for reconstruction
    if (__cloudReconstruction != nullptr)
    {
        __viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0, __reconstructionViewport);
    }
}

void PointCloudVisualizer::UpdatePointCloudReconstruction(const std::shared_ptr<ob::Frame>& frame)
{
    std::vector<OBColorPoint> points = getRGBPointsFromFrame(frame);
    int pointsSize = points.size();

    if (__imuPoseEstimator) {
        __imuPoseEstimator->Start();
        Eigen::Vector3f position = __imuPoseEstimator->GetPosition();
        Eigen::Quaternionf orientation = __imuPoseEstimator->GetOrientation();

        __cloudReconstruction->clear();
        for (int i = 0; i < pointsSize; i++) {
            // Transform the points to the current pose
            Eigen::Vector3f point(points[i].x, points[i].y, points[i].z);
            point = orientation * point + position;

            auto& newPoint = __cloudReconstruction->emplace_back();
            newPoint.x = point.x();
            newPoint.y = point.y();
            newPoint.z = point.z();
            newPoint.r = points[i].r;
            newPoint.g = points[i].g;
            newPoint.b = points[i].b;
        }
    }

    // Update the point cloud in the right viewport (reconstruction)
    if (!__viewer->updatePointCloud(__cloudReconstruction, "PointCloudReconstruction"))
    {
        __viewer->addPointCloud<pcl::PointXYZRGB>(__cloudReconstruction, "PointCloudReconstruction", __reconstructionViewport);
        __viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "PointCloudReconstruction");
    }
}

void PointCloudVisualizer::UpdatePointCloudReconstruction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    if (cloud == nullptr) return;
    __cloudReconstruction = cloud;
    if (!__viewer->updatePointCloud(__cloudReconstruction, "PointCloudReconstruction"))
    {
        __viewer->addPointCloud<pcl::PointXYZRGB>(__cloudReconstruction, "PointCloudReconstruction", __reconstructionViewport);
        __viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "PointCloudReconstruction");
    }
}

void PointCloudVisualizer::SetKeyboardCallback(std::function<void(const pcl::visualization::KeyboardEvent&)> callback)
{
    __viewer->registerKeyboardCallback(callback);
}

void PointCloudVisualizer::SetMouseCallback(std::function<void(const pcl::visualization::MouseEvent&)> callback)
{
    __viewer->registerMouseCallback(callback);
}

void PointCloudVisualizer::SetIMUPoseEstimator(IMU_PoseEstimator* imuPoseEstimator)
{
    __imuPoseEstimator = imuPoseEstimator;
}

void PointCloudVisualizer::UpdatePositionText(const Eigen::Vector3f& position)
{
    const std::string text = "Position: (" + std::to_string(position.x()) + ", " + std::to_string(position.y()) + ", " + std::to_string(position.z()) + ")";
    __viewer->updateText(text, 10, 30, 10, 1, 1, 1, "PositionText");
}

void PointCloudVisualizer::UpdateOrientationText(const Eigen::Quaternionf& orientation)
{
    const std::string text = "Rotation: (" + std::to_string(orientation.x()) + ", " + std::to_string(orientation.y()) + ", " + std::to_string(orientation.z()) + ")";
    __viewer->updateText(text, 10, 50, 10, 1, 1, 1, "RotationText");
}

void PointCloudVisualizer::UpdateGyroscopeText(const OBGyroValue& gyro)
{
    const std::string text = "Gyroscope: (" + std::to_string(gyro.x) + ", " + std::to_string(gyro.y) + ", " + std::to_string(gyro.z) + ")";
    __viewer->updateText(text, 10, 90, 10, 1, 1, 1, "GyroscopeText");
}

void PointCloudVisualizer::UpdateAccelerometerText(const OBAccelValue& accel)
{
    const std::string text = "Accelerometer: (" + std::to_string(accel.x) + ", " + std::to_string(accel.y) + ", " + std::to_string(accel.z) + ")";
    __viewer->updateText(text, 10, 70, 10, 1, 1, 1, "AccelerometerText");
}

Timer::Timer(const std::string& name)
    : __name(name)
    , start_time(std::chrono::high_resolution_clock::now())
    , checkpoint(0)
{
}

void Timer::reset() {
    start_time = std::chrono::high_resolution_clock::now();
}

double Timer::elapsedMilliseconds() const {
    return std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - start_time).count();
}

double Timer::elapsedSeconds() const {
    return std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - start_time).count();
}

void Timer::PrintTimePassed()
{
    std::cout << "Checkpoint(" << __name << ")[" << checkpoint++ << "]: Time elapsed: " << elapsedMilliseconds() << " ms\n";
}
