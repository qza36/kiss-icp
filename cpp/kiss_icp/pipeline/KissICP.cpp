// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "KissICP.hpp"

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

namespace kiss_icp::pipeline {

KissICP::Vector3dVectorTuple KissICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                    const std::vector<double> &timestamps) {
    // Preprocess the input cloud
    const auto &preprocessed_frame = preprocessor_.Preprocess(frame, timestamps, last_delta_);

    // Voxelize
    const auto &[source, frame_downsample] = Voxelize(preprocessed_frame);

    // Get adaptive_threshold
    const double sigma = adaptive_threshold_.ComputeThreshold();

    // Initialize pose with prior map if this is the first frame and prior map is available
    if (!is_initialized_ && config_.use_prior_map && !prior_map_.empty()) {
        last_pose_ = InitializePoseWithPriorMap(source);
        is_initialized_ = true;
    }

    // Compute initial_guess for ICP
    const auto initial_guess = last_pose_ * last_delta_;

    // Run ICP
    const auto new_pose = registration_.AlignPointsToMap(source,         // frame
                                                         local_map_,     // voxel_map
                                                         initial_guess,  // initial_guess
                                                         3.0 * sigma,    // max_correspondence_dist
                                                         sigma);         // kernel

    // Compute the difference between the prediction and the actual estimate
    const auto model_deviation = initial_guess.inverse() * new_pose;

    // Update step: threshold, local map, delta, and the last pose
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    local_map_.Update(frame_downsample, new_pose);
    last_delta_ = last_pose_.inverse() * new_pose;
    last_pose_ = new_pose;

    // Return the (deskew) input raw scan (preprocessed_frame) and the points used for registration
    // (source)
    return {preprocessed_frame, source};
}

KissICP::Vector3dVectorTuple KissICP::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
    const auto voxel_size = config_.voxel_size;
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}
void KissICP::Reset() {
    last_pose_ = Sophus::SE3d();
    last_delta_ = Sophus::SE3d();

    // Clear the local map
    local_map_.Clear();

    // Reset adaptive threshold (it will start fresh)
    adaptive_threshold_ =
        AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
}

Sophus::SE3d KissICP::InitializePoseWithPriorMap(const std::vector<Eigen::Vector3d> &source) {
    if (prior_map_.empty()) {
        return Sophus::SE3d();
    }

    // Create a voxel hash map from the prior map
    VoxelHashMap prior_voxel_map(config_.voxel_size, config_.max_range, config_.max_points_per_voxel);
    prior_voxel_map.Update(prior_map_, Sophus::SE3d());

    // Use registration to align the first frame with the prior map
    const auto sigma = config_.initial_threshold;
    
    try {
        // Align the source points to the prior map  
        const auto initialized_pose = registration_.AlignPointsToMap(
            source,                 // frame
            prior_voxel_map,       // voxel_map 
            Sophus::SE3d(),        // initial_guess (identity)
            3.0 * sigma,           // max_correspondence_dist
            sigma);                // kernel

        // TODO: Add logging when C++ logging is available
        return initialized_pose;
    } catch (...) {
        // TODO: Add warning when C++ logging is available
        return Sophus::SE3d();
    }
}

void KissICP::LoadPriorMap() {
    if (!config_.use_prior_map || config_.prior_map_path.empty()) {
        return;
    }

    std::ifstream file(config_.prior_map_path);
    if (!file.is_open()) {
        std::cerr << "[WARNING] Prior map file not found: " << config_.prior_map_path << std::endl;
        return;
    }

    std::string line;
    prior_map_.clear();
    
    try {
        // Simple PCD loader - for now supports ASCII format xyz points
        bool data_section = false;
        while (std::getline(file, line)) {
            if (line.find("DATA ascii") != std::string::npos) {
                data_section = true;
                continue;
            }
            
            if (data_section && !line.empty() && line[0] != '#') {
                std::istringstream iss(line);
                double x, y, z;
                if (iss >> x >> y >> z) {
                    prior_map_.emplace_back(x, y, z);
                }
            }
        }
        
        std::cout << "[INFO] Loaded prior map with " << prior_map_.size() 
                  << " points from " << config_.prior_map_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[WARNING] Failed to load prior map: " << e.what() << std::endl;
        prior_map_.clear();
    }
    
    file.close();
}

}  // namespace kiss_icp::pipeline
