//
// Created by sc on 11/4/20.
//
#include "config.h"
using namespace PSLAM;
ConfigPSLAM::ConfigPSLAM(const std::string &path){
    if(!path.empty()) pth=path;
    std::fstream f(pth,std::ios::in);
    if(!f.is_open())return;
}

ConfigPSLAM::~ConfigPSLAM(){}

ConfigPSLAM::ConfigPSLAM() {
    use_thread = true;

    use_fusion = true;

    /// Use graph predict or not
    graph_predict = true;

    /// model path
    pth_model = "/home/sc/research/PersistentSLAM/python/3DSSG/experiments/exp10_2_1/traced/";
    pth_model = "/home/sc/research/PersistentSLAM/python/3DSSG/experiments/exp10_4/traced/";
    pth_model = "/home/sc/research/PersistentSLAM/python/3DSSG/experiments/exp10_5/traced/";

    filter_num_node = 512;

    n_pts = 512;

    neighbor_margin=500; // mm

    /// Only update the information of a node if the node has size change to N percent
    update_thres_node_size = 0.2;
    /// Update a node if its time stamp is older than n
    update_thres_time_stamp = 50;


    pth = "./config_graph.txt";
}