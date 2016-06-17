//
// This file is part of the stargazer library.
//
// Copyright 2016 Claudio Bandera <claudio.bandera@kit.edu (Karlsruhe Institute of Technology)
//
// The stargazer library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The stargazer library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#pragma once


#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

#include "StargazerTypes.h"

namespace stargazer {

inline void readConfig(std::string cfgfile, camera_params_t& camera_intrinsics, landmark_map_t& landmarks) {
    YAML::Node config;

    try {
        config = YAML::LoadFile(cfgfile);
    } catch (YAML::BadFile& e) {
        std::string msg = "Stargazer config file does not exist: " + cfgfile;
        throw std::runtime_error(msg);
    } catch (YAML::ParserException& e) {
        std::string msg = "Wrong YAML syntax in stargazer config file: " + cfgfile;
        throw std::runtime_error(msg);
    }

    if (config["CameraIntrinsics"]) {
        camera_intrinsics[(int)INTRINSICS::f] = config["CameraIntrinsics"]["f"].as<double>();
        camera_intrinsics[(int)INTRINSICS::u0] = config["CameraIntrinsics"]["u0"].as<double>();
        camera_intrinsics[(int)INTRINSICS::v0] = config["CameraIntrinsics"]["v0"].as<double>();
        camera_intrinsics[(int)INTRINSICS::alpha] = config["CameraIntrinsics"]["alpha"].as<double>();
        camera_intrinsics[(int)INTRINSICS::beta] = config["CameraIntrinsics"]["beta"].as<double>();
        camera_intrinsics[(int)INTRINSICS::theta] = config["CameraIntrinsics"]["theta"].as<double>();
    } else {
        std::string msg = "Stargazer Config file is missing camera_intrinsics!: " + cfgfile;
        throw std::runtime_error(msg);
    }

    if (config["Landmarks"]) {
        for (size_t i = 0; i < config["Landmarks"].size(); i++) {
            auto lm = config["Landmarks"][i];
            int id = lm["HexID"].as<int>();
            pose_t lm_pose;
            lm_pose[(int)POSE::X] = lm["x"].as<double>();
            lm_pose[(int)POSE::Y] = lm["y"].as<double>();
            lm_pose[(int)POSE::Z] = lm["z"].as<double>();
            lm_pose[(int)POSE::Rx] = lm["rx"].as<double>();
            lm_pose[(int)POSE::Ry] = lm["ry"].as<double>();
            lm_pose[(int)POSE::Rz] = lm["rz"].as<double>();
            landmarks[id] = Landmark(id);
            landmarks[id].pose = lm_pose;
        }
    } else {
        std::string msg = "Stargazer Config file is missing landmarks!: " + cfgfile;
        throw std::runtime_error(msg);
    }
}

inline void writeConfig(std::string cfgfile, const camera_params_t& camera_intrinsics,
                        const landmark_map_t& landmarks) {
    std::ofstream fout(cfgfile);

    fout << "CameraIntrinsics:\n";
    fout << " f: " << camera_intrinsics[(int)INTRINSICS::f] << "\n";
    fout << " u0: " << camera_intrinsics[(int)INTRINSICS::u0] << "\n";
    fout << " v0: " << camera_intrinsics[(int)INTRINSICS::v0] << "\n";
    fout << " alpha: " << camera_intrinsics[(int)INTRINSICS::alpha] << "\n";
    fout << " beta: " << camera_intrinsics[(int)INTRINSICS::beta] << "\n";
    fout << " theta: " << camera_intrinsics[(int)INTRINSICS::theta] << "\n";
    fout << "\n";
    fout << "Landmarks:\n";

    for (auto& entry : landmarks) {
        fout << " - {";
        fout << " HexID: "
             << "0x" << std::setfill('0') << std::setw(4) << std::hex << entry.first << std::setfill(' ');
        fout << ", x: " << std::setw(8) << entry.second.pose[(int)POSE::X];
        fout << ", y: " << std::setw(8) << entry.second.pose[(int)POSE::Y];
        fout << ", z: " << std::setw(8) << entry.second.pose[(int)POSE::Z];
        fout << ", rx: " << std::setw(8) << entry.second.pose[(int)POSE::Rx];
        fout << ", ry: " << std::setw(8) << entry.second.pose[(int)POSE::Ry];
        fout << ", rz: " << std::setw(8) << entry.second.pose[(int)POSE::Rz];
        fout << " }\n";
    }

    fout.close();
}

} // namespace stargazer
