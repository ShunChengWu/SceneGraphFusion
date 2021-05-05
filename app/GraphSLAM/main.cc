#define STB_IMAGE_IMPLEMENTATION
#ifdef __APPLE__
/* Defined before OpenGL and GLUT includes to avoid deprecation messages */
#define GL_SILENCE_DEPRECATION
#endif
#include <iostream>
#include <dataLoader/dataset_loader_facotry.h>
#include "../../Utils/parser.hpp"
#include <ORUtils/Logging.h>
#include <ORUtils/PathTool.hpp>

#ifdef COMPILE_WITH_PSLAM_GUI
#include "../../libGraphSLAMGUI/GraphSLAMGUI.h"
#include "../../Utils/EvaluationHelper.h"
#else
#include "../../libGraphSLAM/GraphSLAM.h"
#include "../Utils/EvaluationHelper.h"
#ifdef COMPILE_WITH_ASSIMP
#include "../libGUI3D/libGUI3D/GUI3D.h"
#include "../../libGraphSLAMGUI/meshRenderer.h"
#endif
#endif

struct Params{
    std::string pth_in;
    std::string pth_out = "./";
    std::string pth_model;
    std::string save_name = "inseg.ply";
    int min_pyr_level=2;

    float depth_edge_threshold = -1; // -1: use default.

    /// Use rendered view from a given mesh (for ScanNet)
    bool use_render=false;
    bool save=true;
    bool save_graph=true;
    bool save_graph_ply = true;
    bool save_surfel_ply = true;
    bool save_kf = true;
    bool save_time = true;

    /// Predict semantic scene graph
    bool use_predict=true;
    bool full_prop=true;
    bool binary=false;
    bool fusion=true;
    ///
    bool thread=true;

    bool verbose=false;
    int segment_filter=512;
};

void ParseCommondLine(int argc, char **argv, Params &params) {
    tools::Parser parser(argc,argv);
    parser.addOption(pkgcname("pth_in", &params.pth_in), "", true);
    parser.addOption(pkgcname("pth_out", &params.pth_out),"");
    parser.addOption(pkgcname("pth_model", &params.pth_model),"path to the folder contains models",false);

    parser.addOption(pkgcname("save_name", &params.save_name),"");

    parser.addOption(pkgcname("min_pyr_level", &params.min_pyr_level),
                     "which level to run InSeg");
    parser.addOption(pkgcname("segment_filter", &params.segment_filter),
                     "Filter out segment that has not enough surfels.");
    parser.addOption(pkgcname("depth_edge_threshold", &params.depth_edge_threshold),
                     "depth_edge_threshold for InSeg");

    parser.addOption(pkgcname("rendered", &params.use_render), "use rendered depth");
    parser.addOption(pkgcname("full_prop", &params.full_prop), "return full prop or just the maximum.");
    parser.addOption(pkgcname("fusion", &params.fusion), "use fusion.");
    parser.addOption(pkgcname("binary", &params.binary), "save ply to binary.");
    parser.addOption(pkgcname("thread", &params.thread), "use thread on scene graph prediction.");
    parser.addOption(pkgcname("verbose", &params.verbose), "verbose.");
    parser.addOption(pkgcname("prediction", &params.use_predict), "do graph prediction.");

    parser.addOption(pkgcname("save", &params.save), "");
    parser.addOption(pkgcname("save_graph", &params.save_graph), "");
    parser.addOption(pkgcname("save_graph_ply", &params.save_graph_ply), "output graph as ply (with semantic id)");
    parser.addOption(pkgcname("save_surfel_ply", &params.save_surfel_ply), "output surfels to ply (inseg output)");
    parser.addOption(pkgcname("save_time", &params.save_time), "");

    auto status = parser.showMsg(params.verbose);
    if(status<1)
        exit(-1);
    if(status==2)
        exit(0);
}

#ifndef COMPILE_WITH_PSLAM_GUI
#ifdef COMPILE_WITH_ASSIMP
class Renderer : SC::GUI3D {
public:
    Renderer(int width, int height, const std::string &path):SC::GUI3D("render",width,height,false),
                                                             mMeshRenderer(width,height,path){
    }

    void Render(cv::Mat &rgb, cv::Mat &depth, const Eigen::Matrix4f &pose, const PSLAM::CameraParameters &params){
        auto view_pose = glUtil::GetViewMatrix(pose);
        auto proj = glUtil::Perspective<float>(params.fx,params.fy,
                                               params.cx,params.cy,
                                               params.width,params.height,
                                               glCam->projection_control_->near,glCam->projection_control_->far);
        cv::Mat t_rgb;
        mMeshRenderer.Render(proj,view_pose,glCam->projection_control_->near,glCam->projection_control_->far,t_rgb,depth);
    }
private:
    PSLAM::MeshRenderer mMeshRenderer;
};
#else
class Renderer{
public:

    Renderer(int width, int height, const std::string &path){
        throw std::runtime_error("did not compile with assimp");
    }

    void Render(cv::Mat &rgb, cv::Mat &depth, const Eigen::Matrix4f &pose, const PSLAM::CameraParameters &params){
        throw std::runtime_error("did not compile with assimp");
    }
};
#endif
#endif

int main(int argc, char** argv) {
    Params params;
    ParseCommondLine(argc, argv, params);
#ifdef NDEBUG
    if(params.verbose) SCLOG_ON(VERBOSE);
    else SCLOG_ON(INFO);
#else
    SCLOG_ON(VERBOSE);
#endif

    if (params.save) tools::PathTool::create_folder(params.pth_out);
    auto path = params.pth_in;
    if (path.find(".txt") != std::string::npos) {
        std::ifstream file(path);
        assert(file.is_open());
        std::getline(file, path, '\n');
    }
    SCLOG(INFO) << "Buliding data loader...";
    std::shared_ptr<PSLAM::DatasetLoader_base> dataset_loader_;
    dataset_loader_.reset(PSLAM::DataLoaderFactory::Make(path));
    dataset_loader_->Reset();

#ifndef COMPILE_WITH_PSLAM_GUI
    std::unique_ptr<Renderer> renderer;
    if(params.use_render) {
        SCLOG(INFO) << "Building renderer...";
        if(path.find("scene") != std::string::npos) {
            auto parent_folder = tools::PathTool::find_parent_folder(path, 1);
            auto scan_id = tools::PathTool::getFileName(parent_folder);
            auto pth_ply = parent_folder + "/" + scan_id + "_vh_clean_2.ply";

            renderer = std::make_unique<Renderer>(dataset_loader_->GetCamParamDepth().width,
                                        dataset_loader_->GetCamParamDepth().height,
                                        pth_ply
            );
        }
    }
#endif

    SCLOG(INFO) << "Building framework...";
    PSLAM::ConfigPSLAM configPslam;
    /// Adjust configurations according to dataset and level
    if(params.pth_model.empty())
        configPslam.graph_predict = false;
    else
    {
        configPslam.pth_model = params.pth_model;
        configPslam.use_fusion = params.fusion;
        configPslam.use_thread = params.thread;
        configPslam.filter_num_node = params.segment_filter;
        configPslam.graph_predict=params.use_predict;
        configPslam.main_config.max_pyr_level = 4;
        configPslam.main_config.min_pyr_level = params.min_pyr_level;

        if(params.depth_edge_threshold == -1) {
            SCLOG(VERBOSE) << "use predefined depth edge threshold: " << params.depth_edge_threshold;
            if (configPslam.main_config.min_pyr_level == 1)
                configPslam.inseg_config.depth_edge_threshold = 0.99f;
            else if (configPslam.main_config.min_pyr_level == 2)
                configPslam.inseg_config.depth_edge_threshold = 0.99f;
            else if (configPslam.main_config.min_pyr_level == 3)
                configPslam.inseg_config.depth_edge_threshold = 0.98f;
        } else {
            SCLOG(VERBOSE) << "use custom depth edge threshold: " << params.depth_edge_threshold;
            configPslam.inseg_config.depth_edge_threshold = params.depth_edge_threshold;
        }
    }

    PSLAM::GraphSLAM graphSlam(&configPslam, dataset_loader_->GetCamParamDepth());

#ifdef COMPILE_WITH_PSLAM_GUI
    SCLOG(INFO) << "start gui...";
    PSLAM::GraphSLAMGUI gui(&graphSlam, dataset_loader_.get());
    if(path.find("sens") != std::string::npos) {
        auto parent_folder = tools::PathTool::find_parent_folder(path, 1);
        auto scan_id = tools::PathTool::getFileName(parent_folder);
        auto pth_ply = parent_folder + "/" + scan_id + "_vh_clean_2.ply";
        gui.SetRender(dataset_loader_->GetCamParamDepth().width,dataset_loader_->GetCamParamDepth().height,pth_ply);
    }
    gui.run();
#else
    SCLOG(INFO) << "start processing frames...";
    while (true) {
        if (!dataset_loader_->Retrieve())break;
        SCLOG(VERBOSE) << "process frame: " << dataset_loader_->GetFrameIndex();
        const Eigen::Matrix4f pose = dataset_loader_->GetPose().inverse();
        auto rgb = dataset_loader_->GetRGBImage();
        auto d   = dataset_loader_->GetDepthImage();

        if(renderer) {
            auto t_p = dataset_loader_->GetPose();
            t_p.topRightCorner<3, 1>() /= 1000.f;
            t_p.transposeInPlace();
            cv::Mat t_rgb;
            renderer->Render(t_rgb,d,t_p,dataset_loader_->GetCamParamDepth());
        }
        CTICK("[ALL]0.all");
        graphSlam.ProcessFrame(dataset_loader_->GetFrameIndex(), rgb, d, &pose);
        CTOCK("[ALL]0.all");


    }
    SCLOG(INFO) << "frame processing finished.";

#endif
    if(params.save) {
        graphSlam.Stop();

        SCLOG(INFO) << "saving results.";
        if(params.save_graph_ply) {
            auto colorMode = params.use_predict? PSLAM::GraphSLAM::SAVECOLORMODE_SEMANTIC : PSLAM::GraphSLAM::SAVECOLORMODE_RGB;
            if(params.use_predict)
                graphSlam.SaveNodesToPLY(params.segment_filter, params.pth_out, colorMode,
                                     params.binary);
        }

        if(params.save_surfel_ply)
            graphSlam.SaveSurfelsToPLY(params.pth_out,params.save_name,graphSlam.GetInSeg()->map().surfels,params.binary);

        if(params.save_graph) {
            auto scan_id = tools::PathTool::getFileName(tools::PathTool::find_parent_folder(path, 1));
            auto predictions = graphSlam.GetSceneGraph(params.full_prop);
            json11::Json::object json;
            json[scan_id] = predictions;
            ORUtils::JsonUtils::Dump(json, params.pth_out + "/predictions.json");
        }

        if(params.save_time) {
            SCSLAM::EVALUATION::Logging::printResultToFile(params.pth_out, "times.txt");
#ifdef COMPILE_WITH_GRAPHPRED
            if(graphSlam.GetGraphPred()) {
                const auto &times = graphSlam.GetGraphPred()->GetTimes();
                for (const auto &pair: times) {
                    std::fstream file(params.pth_out + "/times_graph_" + pair.first + ".txt", std::ios::out);
                    file << pair.first << "\n";
                    for (const auto &pp : pair.second)
                        file << pp.first << "," << pp.second << "\n";
                    file << "\n";
                    file.close();
                }
            }
#endif
        }
    }


    return 0;
}
