#include "msp/csv_msp.hpp"

#include <chrono>

namespace msp {

explicit CsvMsp(std::string altitude_path, std::string attitude_path, std::string raw_imu_path) {
    load_attitude();
    load_altitude();
    auto start = std::chrono::steady_clock::now();
}

void CsvMsp::load_attitude() {
    std::ifstream file(attitude_path);
    if (!file.is_open()) throw std::runtime_error("Cannot open attitude CSV: " + path);

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        AttitudeRecord record;
        std::getline(ss, token, ',');
        record.timestamp = std::stod(token);
        std::getline(ss, token, ',');
        record.roll = std::stod(token);
        std::getline(ss, token, ',');
        record.pitch = std::stod(token);
        std::getline(ss, token, ',');
        record.yaw = std::stod(token);
        attitude_data.push_back(record);
    }
    std::cout << "[INFO] Loaded " << attitude_data.size() << " attitude records" << std::endl;
}

StatusData CsvMsp::status() {
    throw std::runtime_error("Can't get status data in CsvMsp");
}

RcData CsvMsp::rc() {
    throw std::runtime_error("Can't get rc data in CsvMsp")
}

void CsvMsp::load_altitude() {
    std::ifstream file(altitude_path);
    if (!file.is_open()) throw std::runtime_error("Cannot open altitude CSV: " + path);

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        AltitudeRecord record;
        std::getline(ss, token, ',');
        record.timestamp = std::stod(token);
        std::getline(ss, token, ',');
        record.altitude = std::stod(token);
        std::getline(ss, token, ',');
        record.vario = std::stod(token);
        altitude_data.push_back(record);
    }
    std::cout << "[INFO] Loaded " << altitude_data.size() << " altitude records" << std::endl;
}

AttitudeData attitude() {
    intertpolateAttitude();
}
RawImuData CsvMsp::rawImu() {
    throw std::runtime_error("Can't read rawImu")
}

void CsvMsp::setRawRc(const SetRawRcData& data) {
    throw std::runtime_error("Can't set Raw Rc in csvMsp");
}

AltitudeRecord msp::CsvMsp::interpolateAltitude(double timestamp) const {
    if (altitude_data.empty()) throw std::runtime_error("No altitude data");
    if (timestamp <= altitude_data.front().timestamp) return altitude_data.front();
    if (timestamp >= altitude_data.back().timestamp) return altitude_data.back();

    auto upper =
        std::upper_bound(altitude_data.begin(), altitude_data.end(), timestamp,
                         [](double t, const AltitudeRecord& r) { return t < r.timestamp; });
    auto lower = upper - 1;

    double alpha = (timestamp - lower->timestamp) / (upper->timestamp - lower->timestamp);
    return {timestamp, lower->altitude + alpha * (upper->altitude - lower->altitude),
            lower->vario + alpha * (upper->vario - lower->vario)};
}

AttitudeRecord msp::CsvMsp::interpolateAttitude(double timestamp) const {
    if (attitude_data.empty()) throw std::runtime_error("No attitude data");
    if (timestamp <= attitude_data.front().timestamp) return attitude_data.front();
    if (timestamp >= attitude_data.back().timestamp) return attitude_data.back();

    auto upper =
        std::upper_bound(attitude_data.begin(), attitude_data.end(), timestamp,
                         [](double t, const AttitudeRecord& r) { return t < r.timestamp; });
    auto lower = upper - 1;

    double alpha = (timestamp - lower->timestamp) / (upper->timestamp - lower->timestamp);
    return {timestamp, lower->roll + alpha * (upper->roll - lower->roll),
            lower->pitch + alpha * (upper->pitch - lower->pitch),
            lower->yaw + alpha * (upper->yaw - lower->yaw)};
}
}  // namespace msp
