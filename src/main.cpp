#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Calculator.hpp"
#include "DynamicSlidingConvex2DHull.hpp"
#include "geometry/geometry_utils.hpp"
#include "io_utils.hpp"

void printHelp(const char* programName) {
  std::cout
      << "Usage: " << programName
      << " --input <input.csv> --window <window_size> --output <output.csv>\n";
  std::cout << "\nOptions:\n";
  std::cout << "  --input FILE    Input CSV file with values\n";
  std::cout << "  --window SIZE   Window size for sliding hull algorithm\n";
  std::cout << "  --output FILE   Output CSV file for results\n";
  std::cout << "  --help          Display this help message\n";
}

int main(int argc, char** argv) {
  if (argc == 1) {
    printHelp(argv[0]);
    return 0;
  }

  std::string inputFile;
  std::string outputFile;
  int windowSize = 0;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];

    if (arg == "--help") {
      printHelp(argv[0]);
      return 0;
    } else if (arg == "--input") {
      if (i + 1 < argc) {
        inputFile = argv[++i];
      } else {
        std::cerr << "Error: --input requires a filename argument\n";
        return 1;
      }
    } else if (arg == "--window") {
      if (i + 1 < argc) {
        try {
          windowSize = std::stoi(argv[++i]);
          if (windowSize <= 0) {
            std::cerr << "Error: Window size must be positive\n";
            return 1;
          }
        } catch (const std::exception& e) {
          std::cerr << "Error: Invalid window size\n";
          return 1;
        }
      } else {
        std::cerr << "Error: --window requires a numeric argument\n";
        return 1;
      }
    } else if (arg == "--output") {
      if (i + 1 < argc) {
        outputFile = argv[++i];
      } else {
        std::cerr << "Error: --output requires a filename argument\n";
        return 1;
      }
    } else {
      std::cerr << "Error: Unknown option: " << arg << "\n";
      printHelp(argv[0]);
      return 1;
    }
  }

  if (inputFile.empty() || outputFile.empty() || windowSize == 0) {
    std::cerr << "Error: Required parameters missing\n";
    printHelp(argv[0]);
    return 1;
  }

  try {
    std::vector<double> inputs;
    io_utils::readCSV(inputFile, inputs);
    if (inputs.empty()) {
      std::cerr << "Error: No data found in input file\n";
      return 1;
    }

    std::vector<Angles> angles(inputs.size());
    calculate(inputs, angles, windowSize);

    io_utils::writeCSV(outputFile, angles);

    std::cout << "Processing complete. Results written to " << outputFile
              << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}