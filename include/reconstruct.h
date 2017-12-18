#ifndef STEREORECOGNITION_RECONSTRUCT_H
#define STEREORECOGNITION_RECONSTRUCT_H

#include <iostream>
#include <sstream>

/**
 * I originally wrote the reconstruction of 2D images to 3D in python,
 * due to time instead of re writing the script in c++ i merely save
 * the images to a file and call the python script giving the locations of the file.
 * The output cloud will then be written in a known location using the name of the original
 * file paths.
 *
 * @param leftPath
 * @param rightPath
 */
inline void generatePointCloud(std::string leftPath, std::string rightPath) {
    std::ostringstream command;
    command << "source activate cv && python ../scripts/reconstruct.py "; // Python script
    command << leftPath << " " << rightPath; // The image paths
    command << " " << "cloud"; // the prefix name todo when threading use thread idx
    system(command.str().c_str());
}

#endif //STEREORECOGNITION_RECONSTRUCT_H
