#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <stdexcept>
#include <cmath>

cv::Mat grayFrame;

struct AltitudeRecord {
    double timestamp;
    double altitude;
    double vario;
};

struct AttitudeRecord {
    double timestamp;
    double roll;
    double pitch;
    double yaw;
};

struct CameraInfo {
    CameraInfo(double fov, int resX, int resY, double minD, double maxD, int f) :
        fov{fov}, resolutionX{resX}, resolutionY{resY},
        minDist{minD}, maxDist{maxD}, fps{f},
        focalLength{resX / (std::tan(fov / 2) * 2)} {}

    const double fov;
    const int resolutionX;
    const int resolutionY;
    const double minDist;
    const double maxDist;
    const int fps;
    const double focalLength;
};

struct GyroData {
    double roll, pitch, yaw;
};


#include "pid/pid.hpp"

class CsvDataLoader;

class DroneAdapter {
public:
    const CameraInfo cameraInfo = CameraInfo(
        37.4 * CV_PI / 180,
        1920, 1080, 0.01, 1000.0, 15
    );

    CsvDataLoader* csv_loader;
    cv::Mat current_frame;
    cv::Mat current_gray;
    double current_timestamp = 0.0;

    void setFrame(const cv::Mat& frame, double timestamp);
    cv::Mat getGrayscaleImage();
    GyroData getGyroData();
    double getAltitude();
};

class CsvDataLoader {
public:
    std::vector<AltitudeRecord> altitude_data;
    std::vector<AttitudeRecord> attitude_data;

    void loadAltitude(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) throw std::runtime_error("Cannot open altitude CSV: " + path);

        std::string line;
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            AltitudeRecord record;
            std::getline(ss, token, ','); record.timestamp = std::stod(token);
            std::getline(ss, token, ','); record.altitude = std::stod(token);
            std::getline(ss, token, ','); record.vario = std::stod(token);
            altitude_data.push_back(record);
        }
        std::cout << "[INFO] Loaded " << altitude_data.size() << " altitude records" << std::endl;
    }

    void loadAttitude(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) throw std::runtime_error("Cannot open attitude CSV: " + path);

        std::string line;
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            AttitudeRecord record;
            std::getline(ss, token, ','); record.timestamp = std::stod(token);
            std::getline(ss, token, ','); record.roll = std::stod(token);
            std::getline(ss, token, ','); record.pitch = std::stod(token);
            std::getline(ss, token, ','); record.yaw = std::stod(token);
            attitude_data.push_back(record);
        }
        std::cout << "[INFO] Loaded " << attitude_data.size() << " attitude records" << std::endl;
    }

    AltitudeRecord interpolateAltitude(double timestamp) const {
        if (altitude_data.empty()) throw std::runtime_error("No altitude data");
        if (timestamp <= altitude_data.front().timestamp) return altitude_data.front();
        if (timestamp >= altitude_data.back().timestamp) return altitude_data.back();

        auto upper = std::upper_bound(altitude_data.begin(), altitude_data.end(), timestamp,
            [](double t, const AltitudeRecord& r) { return t < r.timestamp; });
        auto lower = upper - 1;

        double alpha = (timestamp - lower->timestamp) / (upper->timestamp - lower->timestamp);
        return {timestamp,
                lower->altitude + alpha * (upper->altitude - lower->altitude),
                lower->vario + alpha * (upper->vario - lower->vario)};
    }

    AttitudeRecord interpolateAttitude(double timestamp) const {
        if (attitude_data.empty()) throw std::runtime_error("No attitude data");
        if (timestamp <= attitude_data.front().timestamp) return attitude_data.front();
        if (timestamp >= attitude_data.back().timestamp) return attitude_data.back();

        auto upper = std::upper_bound(attitude_data.begin(), attitude_data.end(), timestamp,
            [](double t, const AttitudeRecord& r) { return t < r.timestamp; });
        auto lower = upper - 1;

        double alpha = (timestamp - lower->timestamp) / (upper->timestamp - lower->timestamp);
        return {timestamp,
                lower->roll + alpha * (upper->roll - lower->roll),
                lower->pitch + alpha * (upper->pitch - lower->pitch),
                lower->yaw + alpha * (upper->yaw - lower->yaw)};
    }
};

AltitudeRecord interpolateAltitude(CsvDataLoader* loader, double timestamp) {
    return loader->interpolateAltitude(timestamp);
}

AttitudeRecord interpolateAttitude(CsvDataLoader* loader, double timestamp) {
    return loader->interpolateAttitude(timestamp);
}

void DroneAdapter::setFrame(const cv::Mat& frame, double timestamp) {
    current_frame = frame.clone();
    current_timestamp = timestamp;
    if (!current_frame.empty()) {
        cv::cvtColor(current_frame, current_gray, cv::COLOR_BGR2GRAY);
    }
}

cv::Mat DroneAdapter::getGrayscaleImage() {
    if (current_gray.empty()) throw std::runtime_error("No frame set");
    return current_gray;
}

GyroData DroneAdapter::getGyroData() {
    AttitudeRecord att = csv_loader->interpolateAttitude(current_timestamp);
    return {att.roll * CV_PI / 180.0, att.pitch * CV_PI / 180.0, att.yaw * CV_PI / 180.0};
}

double DroneAdapter::getAltitude() {
    return csv_loader->interpolateAltitude(current_timestamp).altitude / 100.0;  // cm to m
}


class OpticalFlow {
public:
    DroneAdapter* drone;
    cv::Mat prevFrame;
    std::vector<cv::Point2f> prevPoints;
    cv::Point2f opticalFlow{0.0f, 0.0f};
    static constexpr double alpha = 0.3;

    explicit OpticalFlow(DroneAdapter& d) : drone(&d) {}

    void calc(int x, int y, int len) {
        grayFrame = drone->getGrayscaleImage();

        int x0 = std::max(x - len, 0);
        int y0 = std::max(y - len, 0);
        int x1 = std::min(x + len, grayFrame.cols - 1);
        int y1 = std::min(y + len, grayFrame.rows - 1);
        cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

        if (prevFrame.empty()) {
            prevFrame = grayFrame.clone();
            cv::goodFeaturesToTrack(grayFrame(roi), prevPoints, 200, 0.01, 5);
            for (auto &p : prevPoints) { p.x += roi.x; p.y += roi.y; }
            opticalFlow = cv::Point2f(0, 0);
            return;
        }

        if (prevPoints.size() < 50) {
            std::vector<cv::Point2f> newPoints;
            cv::goodFeaturesToTrack(grayFrame(roi), newPoints, 200, 0.01, 5);
            for (auto &p : newPoints) { p.x += roi.x; p.y += roi.y; }
            prevPoints.insert(prevPoints.end(), newPoints.begin(), newPoints.end());
        }

        if (prevPoints.empty()) return;

        std::vector<cv::Point2f> nextPoints;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prevFrame, grayFrame, prevPoints, nextPoints, status, err);
        prevFrame = grayFrame.clone();

        cv::Point2f flowSum(0, 0);
        std::vector<cv::Point2f> goodNext, goodPrev;
        for (size_t i = 0; i < nextPoints.size(); ++i) {
            if (status[i]) {
                flowSum += nextPoints[i] - prevPoints[i];
                goodNext.push_back(nextPoints[i]);
                goodPrev.push_back(prevPoints[i]);
            }
        }

        if (!goodNext.empty()) {
            cv::Point2f meanFlow = flowSum * (1.0f / goodNext.size());
            opticalFlow = alpha * meanFlow + (1.0f - alpha) * opticalFlow;
        } else {
            opticalFlow = cv::Point2f(0, 0);
        }
        prevPoints = goodNext;
    }

    cv::Point2f getOpticalFlow() const { return opticalFlow; }
};

class VecDown {
public:
    DroneAdapter* drone;
    cv::Point2f vecDown;
    cv::Point2f vecDownDisplacement;
    bool hasPrev = false;

    explicit VecDown(DroneAdapter& d) : drone(&d) {}

    cv::Vec3d calcVecDown3d() {
        GyroData g = drone->getGyroData();
        cv::Vec3f vecDown{0.0f, 0.0f, -1.0f};

        cv::Matx33d Rx(1, 0, 0, 0, cos(-g.roll), -sin(-g.roll), 0, sin(-g.roll), cos(-g.roll));
        cv::Matx33d Ry(cos(-g.pitch), 0, sin(-g.pitch), 0, 1, 0, -sin(-g.pitch), 0, cos(-g.pitch));
        cv::Matx33d Rz(cos(-g.yaw), -sin(-g.yaw), 0, sin(-g.yaw), cos(-g.yaw), 0, 0, 0, 1);

        return Rz * Ry * Rx * vecDown;
    }

    cv::Point2f calcVecDownProjection() {
        cv::Vec3d v = calcVecDown3d();
        double depth = -v[2];
        if (depth <= 0.0) return {0.0, 0.0};

        double x_screen = -drone->cameraInfo.focalLength * (v[0] / depth);
        double y_screen = drone->cameraInfo.focalLength * (v[1] / depth);

        float u = drone->cameraInfo.resolutionX / 2.0 + x_screen;
        float v_scr = drone->cameraInfo.resolutionY / 2.0 + y_screen;

        return {
            std::max(std::min(u, (float)drone->cameraInfo.resolutionX), 0.0f),
            std::max(std::min(v_scr, (float)drone->cameraInfo.resolutionY), 0.0f)
        };
    }

    void calc() {
        if (!hasPrev) {
            vecDown = calcVecDownProjection();
            hasPrev = true;
        }
        cv::Point2f newVecDown = calcVecDownProjection();
        vecDownDisplacement = newVecDown - vecDown;
        vecDown = newVecDown;
    }

    cv::Point2f getVecDown() const { return vecDown; }
    cv::Point2f getVecDownDisplacement() const { return vecDownDisplacement; }
};

class VecMove {
public:
    DroneAdapter* drone;
    VecDown vecDown;
    OpticalFlow opticalFlow;
    cv::Point2f vecMove;
    static constexpr int accountFlowPixels = 500;
    static constexpr double vecDownDisplacementCoef = 0.5;

    explicit VecMove(DroneAdapter& d) : drone(&d), vecDown(d), opticalFlow(d) {}

    void calc() {
        vecDown.calc();
        cv::Point2f p = vecDown.getVecDown();
        opticalFlow.calc((int)p.x, (int)p.y, accountFlowPixels);

        if (p.x < 0 || p.x >= drone->cameraInfo.resolutionX ||
            p.y < 0 || p.y >= drone->cameraInfo.resolutionY) {
            vecMove = p / std::sqrt(drone->cameraInfo.resolutionX * drone->cameraInfo.resolutionX +
                                    drone->cameraInfo.resolutionY * drone->cameraInfo.resolutionY);
            return;
        }

        cv::Point2f meanOpticalFlow = opticalFlow.getOpticalFlow();
        vecMove = (drone->getAltitude() / drone->cameraInfo.focalLength) *
                  (vecDown.getVecDownDisplacement() * vecDownDisplacementCoef - meanOpticalFlow);
    }

    cv::Point2f getVecMove() const { return vecMove; }
};

std::string expandTilde(const std::string& path) {
    if (path.empty() || path[0] != '~') return path;
    const char* home = getenv("HOME");
    if (!home) throw std::runtime_error("HOME not set");
    return std::string(home) + path.substr(1);
}

std::string extractDirectory(const std::string& filepath) {
    size_t pos = filepath.find_last_of('/');
    return (pos == std::string::npos) ? "." : filepath.substr(0, pos);
}

int main(int argc, char* argv[]) {
    std::cout << "\n[INIT] Video Optical Flow Analyzer Starting..." << std::endl;

    std::string video_path = argc > 1 ? argv[1] : expandTilde("~/Downloads/down2/flow_output.mp4");
    std::string csv_dir = argc > 2 ? argv[2] : extractDirectory(video_path);

    std::cout << "[INIT] Video path: " << video_path << std::endl;
    std::cout << "[INIT] CSV directory: " << csv_dir << std::endl;

    try {
        std::cout << "\n[LOAD] Loading CSV data..." << std::endl;
        CsvDataLoader csv_loader;
        csv_loader.loadAltitude(csv_dir + "/altitude_data.csv");
        csv_loader.loadAttitude(csv_dir + "/attitude_data.csv");

        std::cout << "\n[VIDEO] Opening video file..." << std::endl;
        cv::VideoCapture video(video_path);
        if (!video.isOpened()) throw std::runtime_error("Cannot open video: " + video_path);

        double fps = video.get(cv::CAP_PROP_FPS);
        int total_frames = (int)video.get(cv::CAP_PROP_FRAME_COUNT);

        std::cout << "[VIDEO] Resolution: " << (int)video.get(cv::CAP_PROP_FRAME_WIDTH)
                  << "x" << (int)video.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
        std::cout << "[VIDEO] FPS: " << fps << std::endl;
        std::cout << "[VIDEO] Total frames: " << total_frames << std::endl;

        std::cout << "\n[INIT] Initializing optical flow (FOV=37.4°)..." << std::endl;
        DroneAdapter drone;
        drone.csv_loader = &csv_loader;
        VecMove vec_move(drone);

        cv::Point2f cumulative_position(0.0f, 0.0f);
        cv::Point2f target_position(0.0f, 0.0f);
        PidController pid(50.0f, 1.0f, 1.0f, 0.0f);
        std::cout << "[INIT] PID controller initialized (k_p=50, k_i=1.0, k_d=1.0)" << std::endl;

        std::string output_path = csv_dir + "/flow_data.csv";
        std::ofstream flow_csv(output_path);
        if (!flow_csv.is_open()) throw std::runtime_error("Cannot create: " + output_path);

        flow_csv << "timestamp,x_change,y_change,roll_cmd,pitch_cmd\n" << std::fixed << std::setprecision(6);
        std::cout << "[OUTPUT] Writing to: " << output_path << std::endl;

        std::cout << "\n[PROCESS] Processing frames...\n" << std::endl;
        int frame_idx = 0;
        cv::Mat frame;

        while (video.read(frame) && frame_idx < total_frames) {
            double timestamp = frame_idx / fps;
            float dt = 1.0f / fps;

            drone.setFrame(frame, timestamp);
            vec_move.calc();
            cv::Point2f displacement_m = vec_move.getVecMove();

            float x_cm = displacement_m.x * 100.0f;
            float y_cm = displacement_m.y * 100.0f;

            cumulative_position.x += displacement_m.x;
            cumulative_position.y += displacement_m.y;

            float32x2_t pos_neon = {cumulative_position.x, cumulative_position.y};
            float32x2_t target_neon = {target_position.x, target_position.y};

            uint32x2_t rc_output = pid.calculate_raw_rc(pos_neon, target_neon);

            uint32_t rc_values[2];
            vst1_u32(rc_values, rc_output);
            uint16_t roll_cmd = static_cast<uint16_t>(rc_values[0]);
            uint16_t pitch_cmd = static_cast<uint16_t>(rc_values[1]);

            flow_csv << timestamp << ","
                     << x_cm << ","
                     << y_cm << ","
                     << roll_cmd << ","
                     << pitch_cmd << "\n";

            if (frame_idx % 30 == 0 || frame_idx == total_frames - 1) {
                std::cout << "\r[PROGRESS] Frame " << frame_idx << "/" << total_frames
                          << " (" << (100 * frame_idx / total_frames) << "%)   " << std::flush;
            }
            frame_idx++;
        }

        std::cout << std::endl;
        flow_csv.close();
        video.release();

        std::cout << "\n[COMPLETE] Analysis complete!" << std::endl;
        std::cout << "[OUTPUT] Flow data saved to: " << output_path << std::endl;
        std::cout << "[OUTPUT] Total frames processed: " << frame_idx << std::endl;

        return 0;

    } catch (const std::exception& ex) {
        std::cerr << "\n[ERROR] " << ex.what() << std::endl;
        std::cerr << "\nUsage: " << argv[0] << " [video_path] [csv_directory]" << std::endl;
        return 1;
    }
}
