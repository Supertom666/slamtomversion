//
// Created by tom on 26/11/2020.
//
#include <gflags/gflags.h>
#include "myslam/visualodometry.h"

DEFINE_string(config_file, "./config/Parameter.yaml", "config file path");

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();
    int res = malloc_trim(0);
    if (1 == res) {
        std::cout << "memory cleaned__" << std::endl;
    } else {
        std::cout << "memory not cleaned__" << std::endl;
    }
    return 0;
}
