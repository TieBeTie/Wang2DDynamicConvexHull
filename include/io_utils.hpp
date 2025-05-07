#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry/geometry_utils.hpp"

namespace io_utils {

inline void readCSV(const std::string& filename, std::vector<double>& values) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open input file: " + filename);
  }

  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      try {
        values.push_back(std::stod(line));
      } catch (const std::exception& e) {
        std::cerr << "Warning: Failed to parse line: " << line << std::endl;
      }
    }
  }
}

inline void readCSV(const std::string& filename, std::vector<Angles>& values) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open input file: " + filename);
  }

  std::string line;
  bool header_skipped = false;

  while (std::getline(file, line)) {
    if (!line.empty()) {
      if (!header_skipped && (line.find("alpha") != std::string::npos ||
                              line.find("Alpha") != std::string::npos)) {
        header_skipped = true;
        continue;
      }

      try {
        std::stringstream ss(line);
        std::string alpha_str, beta_str;

        if (std::getline(ss, alpha_str, ',') && std::getline(ss, beta_str)) {
          Angles angle;
          angle.alpha = std::stod(alpha_str);
          angle.beta = std::stod(beta_str);
          values.push_back(angle);
        } else {
          std::cerr << "Warning: Failed to parse line (invalid format): "
                    << line << std::endl;
        }
      } catch (const std::exception& e) {
        std::cerr << "Warning: Failed to parse line: " << line << std::endl;
      }
    }
  }
}

inline void writeCSV(const std::string& filename,
                     const std::vector<Angles>& angles) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open output file: " + filename);
  }

  file << "alpha,beta\n";
  for (const auto& angle : angles) {
    file << angle.alpha << "," << angle.beta << "\n";
  }
}

}  // namespace io_utils
