#pragma once

#include "utils/utils.h"
using utils::log;
using utils::logeol;
using utils::loghline;
using utils::logmem;

// STL libraries
#include <iostream>
#include <string>
#include <csignal>
#include <vector>
// #include <unordered_map>
// #include <unordered_set>
#include <thread>
#include <mutex>
#include <set>
#include <tuple>
#include <bitset>
#include <sstream>
#include <fstream>

#include "utils/robin_hood.h"

// Rsyn
#include "rsyn/session/Session.h"
#define RSYN_NO_GUI
#include "rsyn/core/Rsyn.h"
#include "rsyn/phy/PhysicalService.h"
#include "rsyn/ispd18/RoutingGuide.h"
#include "rsyn/io/reader/ISPD2018Reader.h"

// Boost libraries
#include <boost/icl/split_interval_map.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using boostPoint = bg::model::point<DBU, 2, bg::cs::cartesian>;
using boostBox = bg::model::box<boostPoint>;
using RTree = bgi::rtree<std::pair<boostBox, int>, bgi::rstar<32>>; 

struct Parameters {
    std::string lef_file;
    std::string def_file;
    std::string out_file;
    std::string dgr_file;
    std::string dgr_tree_file; // whether DGR picks new trees, if it is not empty, then load the new tree
    int threads = 1;
    int new_sort = 1; // new sort, which uses the number of available edges in DAG as the first priority. 1: enable; 0: disable.
    int phase2 = 1; // 1: enable phase 2; 0: disable phase 2
    // 
    const double weight_wire_length = 0.5;
    const double weight_via_number = 4.0; // used in maze routing to estimate via cost for a turning point
    const double weight_short_area = 500.0;
    //
    const int min_routing_layer = 1;
    const double cost_logistic_slope = 1.0;
    const double max_detour_ratio = 0.25; // allowed stem length increase to trunk length ratio
    const int target_detour_count = 20;
    const double via_multiplier = 2.0;
    //
    const double maze_logistic_slope = 0.5;
    //
    const double pin_patch_threshold = 20.0;
    const int pin_patch_padding = 1;
    const double wire_patch_threshold = 2.0;
    const double wire_patch_inflation_rate = 1.2;
    //
    const bool write_heatmap = true;
    
    Parameters(int argc, char* argv[]) {
        if (argc <= 1) {
            log() << "Too few args..." << std::endl;
            exit(1);
        }
        for (int i = 1; i < argc; i++) {
            if (strcmp(argv[i], "-lef") == 0) {
                lef_file = argv[++i];
            } else if (strcmp(argv[i], "-def") == 0) {
                def_file = argv[++i];
            } else if (strcmp(argv[i], "-dgr") == 0) {
                dgr_file = argv[++i];
            } else if (strcmp(argv[i], "-tree") == 0) {
                dgr_tree_file = argv[++i];
            } else if (strcmp(argv[i], "-output") == 0) {
                out_file = argv[++i];
            } else if (strcmp(argv[i], "-threads") == 0) {
                threads = std::stoi(argv[++i]);
            } else if (strcmp(argv[i], "-sort") == 0) {
                new_sort = std::stoi(argv[++i]);
            } else if (strcmp(argv[i], "-phase2") == 0) {
                phase2 = std::stoi(argv[++i]);
            } else {
                log() << "Unrecognized arg..." << std::endl;
                log() << argv[i] << std::endl;
            }
        }
        log() << "lef file: " << lef_file << std::endl;
        log() << "def file: " << def_file << std::endl;
        log() << "output  : " << out_file << std::endl;
        log() << "dgr file : " << dgr_file  << std::endl;
        log() << "threads : " << threads  << std::endl;
        log() << "sort    : " << new_sort  << std::endl;
        log() << std::endl;
    }
};