//
// Created by sc on 8/21/20.
//

#include "GraphSLAMGUI.h"
#include <ORUtils/PathTool.hpp>
#include "../Utils/EvaluationHelper.h"
#include <dataLoader/dataset_loader_facotry.h>
using namespace PSLAM;

GraphSLAMGUI::GraphSLAMGUI(GraphSLAM *graphSlam, DatasetLoader_base *dataloader)
        : SC::GUI3D("Graph SLAM GUI", 1280, 800),
          mpGraphSLAM(graphSlam),
          mpDataLoader(dataloader)
{
    glCam = std::make_unique<glUtil::Camera>(window_->width, window_->height, camPose, camUp, yaw, pitch);
    glCam->camera_control_ = std::make_unique<SC::ArcCameraContorl>();
    reinterpret_cast<SC::ArcCameraContorl*>(glCam->camera_control_.get())->SetDistance(5.0);

    registerKeyFunciton(window_, GLFW_KEY_ESCAPE,
                        [&]() {
                            glfwSetWindowShouldClose(window_->window, true);
                        });

    mGraphDrawer.Init();
    mSurfelDrawer.Init(std::bind(&GraphSLAMGUI::GetSurfelColor,this,std::placeholders::_1,std::placeholders::_2));
    mCameraDrawer.Init();
    mTrajectoryDrawer.Init();
    for (auto &drawer : mImageDrawer)
        drawer.Init(mpDataLoader->GetDepthImage().cols, mpDataLoader->GetDepthImage().rows,
                    GL_RGB);
    mRGB = mpDataLoader->GetRGBImage().clone();//cv::Mat::zeros(mpDataLoader->GetDepthImage().rows, mpDataLoader->GetDepthImage().cols, CV_8UC3);
    mDepth = mpDataLoader->GetDepthImage().clone();

#ifdef COMPILE_WITH_GRAPHPRED
    if (mpGraphSLAM->GetConfig()->graph_predict && mpGraphSLAM->GetGraphPred()) {
        for(const auto& label : mpGraphSLAM->GetGraphPred()->mLabels) {
            if (NYU40Name2Labels.find(label.second) == NYU40Name2Labels.end()) {
                SCLOG(WARNING) << "cannot find label name " << label.second << " in NYU40Name2Labels";
            }

        }
    }
#endif
    mSurfelDrawer.mDiffuseStrength=0.5;
    bFaceCulling=false;
    bShowFPS=true;
}

GraphSLAMGUI::~GraphSLAMGUI() {
    SCSLAM::EVALUATION::Logging::printResultToFile("./","times.txt");
}

void GraphSLAMGUI::drawUI() {
    fps_->wait();
//    SC::GUI3D::drawUI();
    MainUI();
    cameraUI();
    glCam->drawUI();
    mouseControl();
}

void GraphSLAMGUI::drawGL() {
    glClearColor(255.00f, 255.00f, 255.00f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    processInput(window_->window);
    basicProcess();
    Process();
}

void GraphSLAMGUI::MainUI(){
    if(!ImGui::Begin("Control Panel",nullptr, ImGuiWindowFlags_MenuBar)){
        ImGui::End();
        return;
    }

    /// Control Buttons
    ImGui::PushID(0);
    ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0 / 7.0f, 0.6f, 0.6f));
    if(ImGui::Button("Stop")) mProcessMode=PROCESS_STOP;
    ImGui::PopStyleColor(1);
    ImGui::PopID();
    ImGui::SameLine();

    ImGui::PushID(1);
    ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(2 / 7.0f, 0.6f, 0.6f));
    if(ImGui::Button("Step")) mProcessMode=PROCESS_ONCE;
    ImGui::PopStyleColor(1);
    ImGui::PopID();
    ImGui::SameLine();

    ImGui::PushID(2);
    ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(4 / 7.0f, 0.6f, 0.6f));
    if(ImGui::Button("Continue")) mProcessMode=PROCESS_CONTINUE;
    ImGui::PopStyleColor(1);
    ImGui::PopID();

    ImGui::Checkbox("Process", &bProcess);

    /// Render options
    ImGui::Separator();
    bNeedUpdate |= ImGui::Checkbox("Surfel", &bRenderSurfel);
    ImGui::SameLine();
    bNeedUpdate |= ImGui::Checkbox("Node", &mGraphDrawer.mbDrawNode);
    ImGui::SameLine();
    bNeedUpdate |= ImGui::Checkbox("Edge", &mGraphDrawer.mbDrawEdge);
    bNeedUpdate |= ImGui::Checkbox("Label", &mGraphDrawer.mbDrawLabel);

    bNeedUpdate |= ImGui::Checkbox("NodeLabel", &bRenderNodeLabel);
    ImGui::SameLine();
    bNeedUpdate |= ImGui::Checkbox("EdgeLabel", &bRenderEdgeLabel);

    ImGui::Checkbox("face culling",&bFaceCulling);
    ImGui::Checkbox("Draw grid", &bShowGrid);
    ImGui::Checkbox("Draw Traj.", &bDrawTraj);
    ImGui::Checkbox("Draw Cam", &bDrawCam);

    ImGui::Checkbox("FPS", &bShowFPS);
    ImGui::Checkbox("Use thread", &mpGraphSLAM->UseThread());

    if(ImGui::Button("Run Prediciton")){
        mpGraphSLAM->RunFullPrediction();
        bNeedUpdate = true;
    }

    /// Select target node
    {
        static int selected_node = mSelectedNodeIdx;
        auto nodes = mpGraphSLAM->GetGraph()->nodes;
        std::vector<int> node_ids;
        for(const auto &n : nodes) {
            if(n.second->surfels.size() < (size_t)mNodeFilterSize) continue;
            node_ids.push_back(n.first);
        }
        if(node_ids.empty()) node_ids.push_back(0);

        int ori = selected_node;
        ImGui::Text("Selected node");
        ImGui::SameLine();
        if (ImGui::Button(std::to_string(node_ids[selected_node]).c_str()))
            ImGui::OpenPopup("node_selection_popup");
        if (ImGui::BeginPopup("node_selection_popup")) {
            for(size_t i=0;i<node_ids.size();++i) {
                if (ImGui::Selectable(std::to_string(node_ids[i]).c_str())) {
                    selected_node = i;
                }
            }
            ImGui::EndPopup();
        }
        if (ori != selected_node) {
            bNeedUpdate = true;
            mSelectedNodeIdx = node_ids[selected_node];
        }
    }

    static float text_size = mGraphDrawer.mTextDrawer.text_size;
    ImGui::PushItemWidth(0.3f * ImGui::GetWindowWidth());
    if(ImGui::DragFloat("TextSize", &text_size, 0.001f, 0.01f, 5.0f, "%.3f", ImGuiDragDropFlags_None)){
        mGraphDrawer.mTextDrawer.text_size = text_size;
    }
    static float node_radius = mGraphDrawer.mNodeDrawer.mfPointSize;

    ImGui::PushItemWidth(0.3f * ImGui::GetWindowWidth());
    if(ImGui::DragFloat("diffuse strength", &mSurfelDrawer.mDiffuseStrength, 0.01f, 0.0f, 1.0f, "%.3f", ImGuiDragDropFlags_None)){
    }

    ImGui::PushItemWidth(0.3f * ImGui::GetWindowWidth());
    if(ImGui::DragFloat("NodeRadius", &node_radius, 0.001f, 0.0f, 10.0f, "%.3f", ImGuiDragDropFlags_None)){
        mGraphDrawer.mNodeDrawer.SetPointSize(node_radius);
    }

    ImGui::PushItemWidth(0.3f * ImGui::GetWindowWidth());
    if(ImGui::DragInt("node size filter",&mNodeFilterSize,1.0,0,1000)) {
        bNeedUpdate = true;
    }
    /// Show all labels and relationships
    if (ImGui::CollapsingHeader("Graph Config"))
        UI_Graph_Config();

    static bool bNodeInfo = false;
    if(mpGraphSLAM) {
#ifdef COMPILE_WITH_GRAPHPRED
        if(mpGraphSLAM->GetGraphPred()){
            if (ImGui::CollapsingHeader("Label names"))
                UI_Class_Relationships();
            if(ImGui::Button("Show Graph Info")){
                bNodeInfo = !bNodeInfo;
            }
            if(bNodeInfo){
                ImGui::Begin("Node Properties", &bNodeInfo);
                UI_Graph_Info();
                ImGui::End();
            }
        }
#endif
    }
    ImGui::Separator();

    /// Render Controls
    {
        /// Color Rendering Menu
        /// color render type
        {
            static int selected_color_type = mColorType;
            const char *color_names[] = {"LABEL", "PHONG", "NORMAL", "COLOR", "UPDATED", "SEMANTIC", "INSTANCE", "PANOPTIC"};

            ImGui::Text("Color Render Type");
            ImGui::SameLine();
            if (ImGui::Button(color_names[selected_color_type]))
                ImGui::OpenPopup("surfel_rendering_popup");
            ImGui::SameLine();
            if (ImGui::BeginPopup("surfel_rendering_popup")) {
                for (int i = 0; i < IM_ARRAYSIZE(color_names); i++)
                    if (ImGui::Selectable(color_names[i]))
                        selected_color_type = i;
                ImGui::EndPopup();
            }
            if (mColorType != selected_color_type) {
                bNeedUpdate = true;
                mColorType = selected_color_type;
            }
        }
        /// Label render type
        {
            static int selected_label_type = mLabelType;
            const char *label_names[] = {"SEGMENT", "INST", "NAME"};

            ImGui::NewLine();
            ImGui::Text("Label Render Type");
            ImGui::SameLine();
            if (ImGui::Button(label_names[selected_label_type]))
                ImGui::OpenPopup("label_rendering_popup");
            if (ImGui::BeginPopup("label_rendering_popup")) {
                for (int i = 0; i < IM_ARRAYSIZE(label_names); i++)
                    if (ImGui::Selectable(label_names[i]))
                        selected_label_type = i;
                ImGui::EndPopup();
            }
            if (mLabelType != selected_label_type) {
                bNeedUpdate = true;
                mLabelType = selected_label_type;
            }
        }
        /// edge type
        {
            static int selected_edge_type = mEdgeType;
            const char *edge_names[] = {"RELATIONSHIP", "NEIGHBOR"};
            ImGui::Text("Edge Render Type");
            ImGui::SameLine();
            if (ImGui::Button(edge_names[selected_edge_type]))
                ImGui::OpenPopup("edge_rendering_popup");
            if (ImGui::BeginPopup("edge_rendering_popup")) {
                for (int i = 0; i < IM_ARRAYSIZE(edge_names); i++)
                    if (ImGui::Selectable(edge_names[i]))
                        selected_edge_type = i;
                ImGui::EndPopup();
            }
            if (mEdgeType != selected_edge_type) {
                bNeedUpdate = true;
                mEdgeType = selected_edge_type;
            }
        }
        /// graph type
        {
            static int selected_edge_type = mDrawGraphType;
            const char *edge_names[] = {"SEGMENTS", "INSTANCE"};
            ImGui::Text("Graph Render Type");
            ImGui::SameLine();
            if (ImGui::Button(edge_names[selected_edge_type]))
                ImGui::OpenPopup("graph_rendering_popup");
            if (ImGui::BeginPopup("graph_rendering_popup")) {
                for (int i = 0; i < IM_ARRAYSIZE(edge_names); i++)
                    if (ImGui::Selectable(edge_names[i]))
                        selected_edge_type = i;
                ImGui::EndPopup();
            }
            if (mDrawGraphType != selected_edge_type) {
                bNeedUpdate = true;
                mDrawGraphType = selected_edge_type;
            }
        }
    }
    ImGui::Separator();

    for(auto &pair:mEdgeUISwitch) {
        if(pair.second) {
            ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);
            std::string name = std::to_string(pair.first) + ": " + mpGraphSLAM->GetGraph()->nodes.at(pair.first)->GetLabel();
            ImGui::Begin(name.c_str(), &pair.second);
            UI_Graph_EdgeInfo(pair.first);
            ImGui::End();
        }
    }


    /// Save
    const std::string pth_out = "./"; //TODO: make this an input
    if(ImGui::Checkbox("RecordImg",&bRecordImg)) ;
    if(ImGui::Button("Save image")) RecordImg();
    if(ImGui::Button("Save Map")) {
        mpGraphSLAM->SaveModel(pth_out);
        printf("map saved at %s\n",pth_out.c_str());
    }
    static bool save_binary_ply = true;
    if(ImGui::Button("Output surfels to ply")) {
        mpGraphSLAM->SaveSurfelsToPLY(mNodeFilterSize,pth_out, "inseg.ply",save_binary_ply);
        printf("ply saved at %s\n",pth_out.c_str());
    }
    ImGui::SameLine();
    ImGui::Checkbox("binary",&save_binary_ply);

    static int node_color_saving_mode = mpGraphSLAM->SAVECOLORMODE_INSTANCE;
    const char* items[] = { "RGB", "Segment", "Instance", "Semantic", "Panoptic"};
    if(ImGui::Combo("Node color mode", &node_color_saving_mode , items, IM_ARRAYSIZE(items))){

    };
    if(ImGui::Button("Save Nodes to ply")) {
        mpGraphSLAM->SaveNodesToPLY(mNodeFilterSize,pth_out, static_cast<PSLAM::GraphSLAM::SAVECOLORMODE>(node_color_saving_mode),save_binary_ply);
        printf("ply saved at %s\n",pth_out.c_str());
    }
    static bool save_full_prop = true;
    if(ImGui::Button("Save Graph")) {
        auto scan_id = tools::PathTool::getFileName(tools::PathTool::find_parent_folder(mpDataLoader->GetDataBase()->folder, 1));
        auto predictions = mpGraphSLAM->GetSceneGraph(save_full_prop);
        json11::Json::object json;
        json[scan_id] = predictions;
        ORUtils::JsonUtils::Dump(json, pth_out + "/predictions.json");

//        mpGraphSLAM->SaveGraph(pth_out,save_full_prop);
//        printf("graph saved at %s\n",pth_out.c_str());
    } ImGui::SameLine();
    ImGui::Checkbox("Full",&save_full_prop);

    ImGui::End();
}

void GraphSLAMGUI::Process(){
//    fps_->updateFPS();
    switch (PROCESS(mProcessMode)) {
        case PROCESS_STOP:
            break;
        case PROCESS_ONCE:
            mProcessMode=PROCESS_STOP;
            [[fallthrough]];
        case PROCESS_CONTINUE:
            bool state = ProcessSLAM();
//            mSurfelDrawer.mUpdatedIndices = mpGraphSLAM->GetGraph()->nodes_to_update;
            bNeedUpdate |= state;
            bNeedUpdateTexture = state;
            if(state) {
                Eigen::Matrix4f pose = mpGraphSLAM->GetInSeg()->pose().inverse();
                pose.topRightCorner<3,1>() /= 1000.f;
                mTrajectoryDrawer.Add({pose.topRightCorner<3, 1>().x(), pose.topRightCorner<3, 1>().y(),
                                       pose.topRightCorner<3, 1>().z()});
            }
            if(!bNeedUpdate)mProcessMode=PROCESS_STOP;;
            break;
    }
#ifdef COMPILE_WITH_GRAPHPRED

    if(mpGraphSLAM->GetGraphPred() && mpGraphSLAM->GetGraphPred()->Updated()) {
        mpGraphSLAM->GetGraphPred()->SetUpdate(false);
        bNeedUpdate=true;
    }
#endif

    auto io  = ImGui::GetIO();
    size_t windowWidth = io.DisplaySize.x, windowHeight = io.DisplaySize.y;
    glViewport(0, 0, windowWidth, windowHeight);

    if(bNeedUpdate) {
        if(bRenderSurfel) {
            std::vector<std::shared_ptr<inseg_lib::Surfel>> tmp;
            for(const auto &node : mpGraphSLAM->GetGraph()->nodes)
                if(node.second->surfels.size() > (size_t)mNodeFilterSize)
                    for(const auto &surfel : node.second->surfels)
                        tmp.push_back(surfel.second);
            mSurfelDrawer.Update(/*mpGraphSLAM->GetInSeg()->map().surfels*/tmp);//TODO: change this to node base to prevent redundant copy on the segment wihtout changes.
        }
    }

    const Eigen::Matrix4f eigen_proj = GLM2E<float,4,4>(glCam->projection_control_->projection_matrix());
    const Eigen::Matrix4f eigen_vm   = GLM2E<float,4,4>(glCam->camera_control_->GetViewMatrix());


#ifdef APPLE
    windowWidth *=2;
    windowHeight*=2;
#endif
    /// Draw2D
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    if(bNeedUpdateTexture){
        cv::Mat rgb;
        cv::cvtColor(mRGB,rgb,cv::COLOR_BGR2RGB);
        cv::flip(rgb, rgb, 0);
        if (dynamic_cast<DatasetLoader_3RScan*>(mpDataLoader))
            cv::rotate(rgb, rgb, cv::ROTATE_90_COUNTERCLOCKWISE);
        mImageDrawer[0].Update(rgb.ptr(), rgb.cols, rgb.rows);

        cv::Mat greyImage;
        cvtColor(mDepth, greyImage, cv::COLOR_GRAY2BGR);
        cv::flip(greyImage, greyImage, 0);
        if (dynamic_cast<DatasetLoader_3RScan*>(mpDataLoader))
            cv::rotate(greyImage, greyImage, cv::ROTATE_90_COUNTERCLOCKWISE);
        double min;
        double max;
        cv::minMaxIdx(greyImage, &min, &max);
        cv::Mat adjMap;
        cv::convertScaleAbs(greyImage, adjMap, 255 / max);
        mImageDrawer[1].Update(adjMap.ptr(),
                               adjMap.cols,
                               adjMap.rows);

        cv::Mat flipLabel;
        cv::flip(mpGraphSLAM->GetInSeg()->GetSegmentedLabelMap(),flipLabel, 0);
        if (dynamic_cast<DatasetLoader_3RScan*>(mpDataLoader))
            cv::rotate(flipLabel, flipLabel, cv::ROTATE_90_COUNTERCLOCKWISE);
        mImageDrawer[2].Update(flipLabel.ptr(),
                               flipLabel.cols,
                               flipLabel.rows);
    }

    {
        bool scannet = false;
        const int size = 3;
        const float sub_window_height = (windowHeight-(0*size+2)) / size;
        const float sub_window_width = scannet ?
                                       sub_window_height*(static_cast<float>(mpDataLoader->GetDepthImage().cols)/static_cast<float>(mpDataLoader->GetDepthImage().rows)) :
                                       sub_window_height*(static_cast<float>(mpDataLoader->GetDepthImage().rows)/static_cast<float>(mpDataLoader->GetDepthImage().cols));
        int i=0;
        for(auto &drawer: mImageDrawer){
            glViewport(windowWidth - sub_window_width,
                       sub_window_height*i++,
                       sub_window_width, sub_window_height);
            drawer.Draw(eigen_proj,eigen_vm);
        }
        glViewport(0, 0, windowWidth, windowHeight);
    }

    /// Draw3D
    bFaceCulling? glEnable(GL_CULL_FACE) : glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    if(bRenderSurfel) mSurfelDrawer.Draw(eigen_proj,eigen_vm);
    Eigen::Matrix4f pose = mpGraphSLAM->GetInSeg()->pose().inverse();
    pose.topRightCorner<3,1>() /= 1000.f;
    if(bDrawCam) {
        mCameraDrawer.SetColor({0,1,1,1});// green
        mCameraDrawer.Draw(pose, eigen_proj,eigen_vm);
    }

    if(bDrawTraj)mTrajectoryDrawer.Draw(eigen_proj,eigen_vm);

    if(bNeedUpdate){
        UpdateGraphRenderer();
    }
    mGraphDrawer.Draw(windowWidth,windowHeight,eigen_vm,eigen_proj);

    if(bRecordImg && bNeedUpdate){
        RecordImg();
    }
    bNeedUpdate = false;
    bNeedUpdateTexture=false;
}

bool GraphSLAMGUI::ProcessSLAM(){
    if(!mpDataLoader->Retrieve()){
        SCLOG(VERBOSE) << "Reach the end of the dataset!";
        return false;
    }
    if(!bProcess) return true;
    CTICK("[GUI][Process]ProcessSLAM");
    const Eigen::Matrix4f pose = mpDataLoader->GetPose();
    auto idx = mpDataLoader->GetFrameIndex();
    mRGB = mpDataLoader->GetRGBImage();
    mDepth = mpDataLoader->GetDepthImage();
#ifdef COMPILE_WITH_ASSIMP
    if(mMeshRender) {
        Eigen::Matrix4f t_p = mpDataLoader->GetPose();
        t_p.topRightCorner<3, 1>() /= 1000.f;
        t_p.transposeInPlace();
        auto view_pose = glUtil::GetViewMatrix(t_p);
        auto proj = glUtil::Perspective<float>(mpDataLoader->GetCamParamDepth().fx,mpDataLoader->GetCamParamDepth().fy,
                                               mpDataLoader->GetCamParamDepth().cx,mpDataLoader->GetCamParamDepth().cy,
                                               mpDataLoader->GetCamParamDepth().width,mpDataLoader->GetCamParamDepth().height,
                                               glCam->projection_control_->near,glCam->projection_control_->far);
        cv::Mat t_rgb;
        mMeshRender->Render(proj,view_pose,glCam->projection_control_->near,glCam->projection_control_->far);
        mDepth = mMeshRender->GetDepth();
    }
#endif
    const Eigen::Matrix4f pose_inv = pose.inverse();
    fps_->start();
    mpGraphSLAM->ProcessFrame(idx,mRGB,mDepth,&pose_inv);
    fps_->stop();
    fps_->checkUpdate();
    CTOCK("[GUI][Process]ProcessSLAM");
    return true;
}

void GraphSLAMGUI::UpdateGraphRenderer(){
    std::map<int, std::shared_ptr<Node>> nodes;
    if(mSelectedNodeIdx==0) {
        nodes = mpGraphSLAM->GetGraph()->nodes;
    } else {
        if(mpGraphSLAM->GetGraph()->nodes.find(mSelectedNodeIdx) != mpGraphSLAM->GetGraph()->nodes.end())
            nodes[mSelectedNodeIdx]=mpGraphSLAM->GetGraph()->nodes.at(mSelectedNodeIdx);
    }


    size_t num_nodes = nodes.size();
    std::map<int, size_t> labelIndexMapper;
    VecEigenf(3) labelPositions;
    VecEigenf(3) labelColors;
    std::vector<std::string> labelNames;
    labelPositions.reserve(num_nodes);
    labelColors.reserve(num_nodes);
    labelNames.reserve(num_nodes);

    std::map<int,int> nodeIndiceMap; // map node to an increment index
    std::vector<unsigned int> edgeIndices;
    Eigen::Vector3f color;

    switch (static_cast<DRAWGRAPHTYPE>(mDrawGraphType)) {
        case DRAWGRAPHTYPE_SEGMENTS:
        {
            /// Nodes
            for(const auto &pair : nodes) {
                if(pair.second->idx == 0) continue; // skip 0 (unlabeled)
                if(pair.second->surfels.empty())continue; // skip empty
                if(mNodeFilterSize>0){ // filter by size
                    if((int)pair.second->surfels.size() < mNodeFilterSize) continue;
                }
                auto tmp = (*(pair.second->surfels.begin())).second;
                GetSurfelColor(color,pair.second->surfels.begin()->second.get());
                labelPositions.emplace_back(pair.second->centroid * 1e-3);
                labelColors.push_back(color);
                if(bRenderNodeLabel) {
                    const auto &name = GetNodeLabel(pair.second.get());
                    if(name == Node::Unknown())
                        labelNames.emplace_back("");
                    else
                        labelNames.emplace_back(name);
                } else {
                    labelNames.emplace_back("");
                }
                if(nodeIndiceMap.find(pair.second->idx) == nodeIndiceMap.end())
                    nodeIndiceMap[pair.second->idx] = labelPositions.size()-1;
            }

            /// Edges
            if(mGraphDrawer.mbDrawEdge) {
                switch (static_cast<DRAWEDGETYPE>(mEdgeType)) {
                    case DRAWEDGETYPE_RELATION:
                    {
                        const auto &nodes_ori = mpGraphSLAM->GetGraph()->nodes;
                        const auto &edges = mpGraphSLAM->GetGraph()->edges;
                        edgeIndices.reserve((edges.size() * 2));
                        for (const auto &pair : edges) {
                            const auto &edge = pair.second;
                            const auto &nodeFrom = nodes_ori.at(edge->nodeFrom);
                            const auto &nodeTo = nodes_ori.at(edge->nodeTo);
                            if (nodeIndiceMap.find(nodeFrom->idx) == nodeIndiceMap.end()) continue;
                            if (nodeIndiceMap.find(nodeTo->idx) == nodeIndiceMap.end()) continue;
                            // Check same part
                            if (pair.second->GetLabel() == PSLAM::Edge::None()) continue;
                            if (pair.second->GetLabel() == PSLAM::Edge::Same()) {
                                if(nodes.at(pair.second->nodeFrom)->GetLabel() !=
                                   nodes.at(pair.second->nodeTo)->GetLabel())
                                    continue;
                            }


                            edgeIndices.push_back(nodeIndiceMap.at(nodeFrom->idx));
                            edgeIndices.push_back(nodeIndiceMap.at(nodeTo->idx));
                            if(bRenderEdgeLabel) {
                                if (edge->GetLabel() == Edge::None()) continue;
                                labelNames.emplace_back(GetEdgeLabel(edge.get()));
                                labelPositions.emplace_back(
                                        (nodeFrom->centroid + nodeTo->centroid) * 0.5 * 1e-3);
                                Eigen::Vector3f edge_color;
                                GetEdgeColor(edge_color,edge.get());
                                labelColors.emplace_back(edge_color);
                            }
                        }
                    }
                        break;
                    case DRAWEDGETYPE_NEIGHBOR:
                    {
                        const auto& nps = mpGraphSLAM->GetGraph()->nodes;
                        for(const auto &pair : nodes) {
                            if(nodeIndiceMap.find(pair.second->idx) == nodeIndiceMap.end())continue;
                            if(nps.find(pair.second->idx) == nps.end()) continue;
                            for (auto &n : nps.at(pair.second->idx)->neighbors) {
                                if(nodeIndiceMap.find(n) == nodeIndiceMap.end())continue;
                                edgeIndices.push_back(nodeIndiceMap.at(pair.second->idx));
                                edgeIndices.push_back(nodeIndiceMap.at(n));

                                if(bRenderEdgeLabel) {
                                    labelNames.emplace_back(std::to_string(pair.second->idx) + "_" + std::to_string(n));
                                    labelPositions.emplace_back(
                                            (nps.at(pair.second->idx)->centroid + nps.at(n)->centroid) * 0.5 * 1e-3);
                                    labelColors.emplace_back(Eigen::Vector3f{0.f, 255.f, 0.f});
                                }
                                //TODO: since the edge is directed, need to add a margin to nodes have forward and backward edges.
                                // otherwise the labels will have overlapping and z-fighting issue.
                            }
                        }
                    }
                        break;
                }
            }
        }
            break;
        case DRAWGRAPHTYPE_INSTANCE:
        {
            /// Nodes
            std::map<int, std::shared_ptr<Node>> tNodes;
            std::map<size_t,std::vector<size_t>> inst_segs;
            {
                std::unique_lock<std::mutex> lock(mpGraphSLAM->GetGraph()->mMutNode);
                for(const auto &pair: nodes) {
                    if(pair.second->idx == 0) continue; // skip 0 (unlabeled)
                    if(pair.second->surfels.empty())continue; // skip empty
                    if(mNodeFilterSize>0) // filter by size
                        if((int)pair.second->surfels.size() < mNodeFilterSize) continue;
                    if(pair.second->GetLabel() == Node::Unknown()) continue;
                    inst_segs[pair.second->instance_idx].push_back(pair.first);
                    tNodes[pair.first] = pair.second;
                    tNodes[pair.second->instance_idx] = pair.second;
                }
            }


            std::map<int, Eigen::Vector3f, std::less<>,
                    Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3f> > > centroids;
            for (const auto &pair:inst_segs) {
                const auto &node = tNodes.at(pair.first);
                // calculate centroid
                auto& centroid = centroids[node->instance_idx];
                centroid.setZero();
                for(const auto &idx : pair.second) {
                    centroid += tNodes.at(idx)->centroid;
                }
                centroid /= pair.second.size();

                color = {1,0,0};
                GetSurfelColor(color,node->surfels.begin()->second.get());
                labelPositions.emplace_back(centroid * 1e-3);
                labelColors.push_back(color);
                if(bRenderNodeLabel) {
                    const auto &name = GetNodeLabel(node.get());
                    if(name == Node::Unknown())
                        labelNames.emplace_back("");
                    else
                        labelNames.emplace_back(name);
                } else {
                    labelNames.emplace_back("");
                }
                if(nodeIndiceMap.find(node->instance_idx) == nodeIndiceMap.end())
                    nodeIndiceMap[node->instance_idx] = labelPositions.size()-1;
            }

            /// Edges
            if(mGraphDrawer.mbDrawEdge) {
                switch (static_cast<DRAWEDGETYPE>(mEdgeType)) {
                    case DRAWEDGETYPE_RELATION:
                    {
                        const auto &edges = mpGraphSLAM->GetGraph()->edges;

                        /// Merge instance predictions
                        std::map<std::pair<size_t,size_t>,std::map<std::string,std::pair<int,float>>> probs;
                        for (const auto &pair : edges) {
                            const auto &edge = pair.second;
                            std::shared_ptr<Node> nodeFrom,nodeTo;
                            {
                                std::unique_lock<std::mutex> lock(mpGraphSLAM->GetGraph()->mMutNode);
                                if(nodes.find(edge->nodeFrom) == nodes.end()) continue;
                                if(nodes.find(edge->nodeTo) == nodes.end()) continue;
                                nodeFrom = nodes.at(edge->nodeFrom);
                                nodeTo = nodes.at(edge->nodeTo);
                            }



                            if(nodeFrom == nodeTo) continue;
                            if(nodeFrom->instance_idx == nodeTo->instance_idx)continue;
                            auto &v = probs[{nodeFrom->instance_idx, nodeTo->instance_idx}];
                            for(const auto &ppair: edge->labelProp) {
                                // ignore same part, since we have filtered it previously
                                if (ppair.first==Edge::Same()) continue;
                                if(v.find(ppair.first) == v.end()) {
                                    v[ppair.first].first=0;
                                    v[ppair.first].second=0;
                                }
                                v.at(ppair.first).first  += 1;
                                v.at(ppair.first).second += ppair.second;
                            }
                        }

                        /// Get label
                        std::map<std::pair<int,int>, std::shared_ptr<PSLAM::Edge> > tEdges;
                        for(const auto &pair: probs) {
                            if (nodeIndiceMap.find(pair.first.first) == nodeIndiceMap.end()) continue;
                            if (nodeIndiceMap.find(pair.first.second) == nodeIndiceMap.end()) continue;
                            std::string max_label = Edge::None();
                            float max_value=0;
                            for(const auto &ppair:pair.second) {
                                auto value = ppair.second.second/ppair.second.first;
                                if (value > max_value) {
                                    max_value = value;
                                    max_label = ppair.first;
                                }
                            }
                            if (max_label == Edge::None()) continue;
                            if (max_label == PSLAM::Edge::Same()) {
                                if(tNodes.at(pair.first.first)->GetLabel() != tNodes.at(pair.first.second)->GetLabel())
                                    continue;
                            }
                            tEdges[pair.first].reset(new PSLAM::Edge() ) ;
                            tEdges.at(pair.first)->nodeFrom = pair.first.first;
                            tEdges.at(pair.first)->nodeTo = pair.first.second;
                            tEdges.at(pair.first)->label = max_label;
                        }

                        edgeIndices.reserve((tEdges.size()));
                        for (const auto &pair : tEdges) {
                            const auto &edge = pair.second;
                            // Check same part
                            edgeIndices.push_back(nodeIndiceMap.at(pair.first.first));
                            edgeIndices.push_back(nodeIndiceMap.at(pair.first.second));
                            if(bRenderEdgeLabel) {
                                labelNames.emplace_back(GetEdgeLabel(edge.get()));
                                labelPositions.emplace_back(
                                        (centroids.at(edge->nodeFrom) + centroids.at(edge->nodeTo)) * 0.5 * 1e-3);
//                                labelColors.emplace_back(Eigen::Vector3f{0.f, 255.f, 0.f});
                                Eigen::Vector3f edge_color;
                                GetEdgeColor(edge_color,edge.get());
                                labelColors.emplace_back(edge_color);
                            }
                        }
                    }
                        break;
                    case DRAWEDGETYPE_NEIGHBOR:
                    {
                        SCLOG(WARNING) << "not implemented.";
//                        const auto& nps = mpGraphSLAM->GetGraph()->nodes;
//                        for(const auto &pair : nodes) {
//                            if(nodeIndiceMap.find(pair.second->idx) == nodeIndiceMap.end())continue;
//                            if(nps.find(pair.second->idx) == nps.end()) continue;
//                            for (auto &n : nps.at(pair.second->idx)->neighbors) {
//                                if(nodeIndiceMap.find(n) == nodeIndiceMap.end())continue;
//                                edgeIndices.push_back(nodeIndiceMap.at(pair.second->idx));
//                                edgeIndices.push_back(nodeIndiceMap.at(n));
//
//                                if(bRenderEdgeLabel) {
//                                    labelNames.emplace_back(std::to_string(pair.second->idx) + "_" + std::to_string(n));
//                                    labelPositions.emplace_back(
//                                            (nps.at(pair.second->idx)->centroid + nps.at(n)->centroid) * 0.5 * 1e-3);
//                                    labelColors.emplace_back(Eigen::Vector3f{0.f, 255.f, 0.f});
//                                }
//                                //TODO: since the edge is directed, need to add a margin to nodes have forward and backward edges.
//                                // otherwise the labels will have overlapping and z-fighting issue.
//                            }
//                        }
                    }
                        break;
                }
            }

        }
            break;
    }
    mGraphDrawer.Update(labelPositions,labelColors,labelNames,edgeIndices);
}

void GraphSLAMGUI::UI_Class_Relationships() {
    ImGuiStyle& style = ImGui::GetStyle();
    float child_w = (ImGui::GetContentRegionAvail().x - 1 * style.ItemSpacing.x) / 2;
    if (child_w < 1.0f)
        child_w = 1.0f;

    {
        ImGui::BeginGroup();
        ImGui::TextUnformatted("Classes");

        const ImGuiWindowFlags child_flags = 0;
        const ImGuiID child_id = ImGui::GetID((void*)(intptr_t)0);
        const bool child_is_visible = ImGui::BeginChild(child_id, ImVec2(child_w, 200.0f), true, child_flags);
#ifdef COMPILE_WITH_GRAPHPRED
        if (child_is_visible) // Avoid calling SetScrollHereY when running with culled items
        {
            for(const auto &pair:mpGraphSLAM->GetGraphPred()->mLabels){
                ImVec4 vec;
                if (mpGraphSLAM->GetGraphPred()->GetParams().at("label_type").string_value() == "NYU40" ||
                    mpGraphSLAM->GetGraphPred()->GetParams().at("label_type").string_value() == "ScanNet20") {
                    auto idx  = NYU40Name2Labels.at(pair.second);
                    const auto& color_ = NYU40LabelColors.at(idx);
                    vec.x = color_.r /255.f;
                    vec.y = color_.g /255.f;
                    vec.z = color_.b /255.f;
                    vec.w = color_.a /255.f;
                } else {
                    const cv::Vec3b &color = inseg_lib::CalculateLabelColor(pair.first+1);
                    vec.x = float(color[2])/255.f;
                    vec.y = float(color[1])/255.f;
                    vec.z = float(color[0])/255.f;
                    vec.w = 1.f;
                }
                ImGui::TextColored(vec, "[%2zu] %s", pair.first,pair.second.c_str());
                //TODO: use label color here
//                ImGui::Text("[%2zu] %s", pair.first,pair.second.c_str());
            }
        }
#endif
        ImGui::EndChild();
        ImGui::EndGroup();
    }
    ImGui::SameLine();
    {
        ImGui::BeginGroup();
        ImGui::TextUnformatted("Relationsihps");

        const ImGuiWindowFlags child_flags = 0;
        const ImGuiID child_id = ImGui::GetID((void*)(intptr_t)1);
        const bool child_is_visible = ImGui::BeginChild(child_id, ImVec2(child_w, 200.0f), true, child_flags);
#ifdef COMPILE_WITH_GRAPHPRED
        if (child_is_visible) // Avoid calling SetScrollHereY when running with culled items
        {
            for(const auto &pair:mpGraphSLAM->GetGraphPred()->mRelationships){
                //TODO: use label color here
                // ImGui::TextColored(ImVec4(1, 1, 0, 1), "Item %d", item);
                ImGui::Text("[%2zu] %s", pair.first,pair.second.c_str());
            }
        }
#endif
        ImGui::EndChild();
        ImGui::EndGroup();
    }

}

void GraphSLAMGUI::UI_Graph_Config(){
    auto *config = mpGraphSLAM->GetConfig();
    ImGui::Checkbox("enable prediction", &config->graph_predict);
    ImGui::Text("Node Update thresholds");
    ImGui::PushItemWidth(0.3f * ImGui::GetWindowWidth());
    ImGui::DragInt("time stamp", &config->update_thres_time_stamp, 1, 0, 512, "%3d", ImGuiDragDropFlags_None);
    ImGui::DragFloat("size difference", &config->update_thres_node_size, 0.01, 0, 5.0, "%4.3f", ImGuiDragDropFlags_None);
    ImGui::Text("Prediction");

//    ImGui::DragInt("neighbor level", &config->n_level, 1, 1, 10, "%2d", ImGuiDragDropFlags_None);
//    ImGui::DragInt("min num of nodes", &config->predict_when_have_at_least_n_node, 1, 2, 128, "%3d", ImGuiDragDropFlags_None);
    ImGui::DragInt("min size per node", &config->filter_num_node, 10, 0, 1e4, "%4d", ImGuiDragDropFlags_None);
}

#include <memory>
#include <string>
#include <stdexcept>

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    std::unique_ptr<char[]> buf( new char[ size ] );
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

void GraphSLAMGUI::UI_Graph_EdgeInfo(size_t node_id) {
    auto nodes = mpGraphSLAM->GetGraph()->nodes;
    if(nodes.find(node_id) == nodes.end()) return;
    if(mpGraphSLAM->GetGraph()->nodes.find(node_id) == mpGraphSLAM->GetGraph()->nodes.end()) return;
    auto &np = nodes.at(node_id);
    auto edges = mpGraphSLAM->GetGraph()->nodes.at(node_id)->edges;
//    (*edges.begin())->nodeFrom
    ImGui::SetNextWindowContentSize(ImVec2(600.0f, 0.0f));
    ImVec2 child_size = ImVec2(0, ImGui::GetFontSize() * 20.0f);
    ImGui::BeginChild("##ScrollingRegion", child_size, false, ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::Columns(3);
    std::vector<std::string> names = {"Predicate", "Id", "Class"};
    std::vector<int> widths = {200,100,200};
    for(size_t i=0;i<names.size();++i) {
        ImGui::Text("%s", names[i].c_str());
        if(widths[i]>0)ImGui::SetColumnWidth(i,widths[i]);
        ImGui::NextColumn();
    }
    ImGui::Separator();

    // build a list for searh
    std::map<int,std::string> list_pred;
    for(auto &edge:edges) list_pred.insert({edge->nodeTo,edge->GetLabel()});

    for(auto &nn:np->neighbors) {
        if (list_pred.find(nn) == list_pred.end()) {
            // no prediction
            ImGui::Text("-");
        } else {
            ImGui::Text("%s",list_pred.at(nn).c_str());
        }
        ImGui::NextColumn();

        ImGui::Text("%lu",nn);

        ImGui::NextColumn();
        std::string name;
        if(mpGraphSLAM->GetGraph()->nodes.find(nn) == mpGraphSLAM->GetGraph()->nodes.end()) {
            name = "Unknown";
        } else {
            name = mpGraphSLAM->GetGraph()->nodes.at(nn)->GetLabel();
        }
        ImGui::Text("%s",name.c_str());
        ImGui::NextColumn();
        ImGui::Separator();
    }
//    ImGui::Columns(1);
    ImGui::EndChild();
}

void GraphSLAMGUI::UI_Graph_Info() {
    //TODO: add
    // node: number of points, centroid, instance id, semantic id, num of edges
    ImGui::SetNextWindowContentSize(ImVec2(1200.0f, 0.0f));
    ImVec2 child_size = ImVec2(0, ImGui::GetFontSize() * 20.0f);
    ImGui::BeginChild("##ScrollingRegion", child_size, false, ImGuiWindowFlags_HorizontalScrollbar);
    std::vector<std::string> names = {"ID", "Label", "Instance", "Size", "Centroid", "Dims", "Neighbors"};
    std::vector<int> widths = {40,80,80,100,200,200,400};
    assert(names.size() == widths.size());
    ImGui::Columns(names.size());
    static int selected = -1;
    ImGui::Separator();
    for(size_t i=0;i<names.size();++i) {
        ImGui::Text("%s", names[i].c_str());
        if(widths[i]>0)ImGui::SetColumnWidth(i,widths[i]);
        ImGui::NextColumn();
    }

    ImGui::Separator();
//    auto &nps = mpGraphSLAM->GetNodeProperties();

    for(auto &pair:mpGraphSLAM->GetGraph()->nodes) {
        if(int(pair.second->surfels.size()) < mNodeFilterSize) continue;

        char label[32];

        /// ID
        sprintf(label, "%4d", pair.first);
        if (ImGui::Selectable(label, selected == pair.first, ImGuiSelectableFlags_SpanAllColumns)) {
            selected = pair.first;
            if(mEdgeUISwitch.find(pair.first) == mEdgeUISwitch.end())
                mEdgeUISwitch[pair.first] = true;
            else
                mEdgeUISwitch[pair.first] = !mEdgeUISwitch[pair.first];
        }
        ImGui::NextColumn();

        /// Label
        ImGui::Text("%s",pair.second->GetLabel().c_str());
        ImGui::NextColumn();

        /// Instance
        ImGui::Text("%d",pair.second->instance_idx.load());
        ImGui::NextColumn();
//        ImGui::SetColumnWidth(0,40);


//        ImGui::SetColumnWidth(1,100);
        ImGui::Text("%8zu",pair.second->surfels.size());
        ImGui::NextColumn();


        auto str = string_format("%7.4f %7.4f %7.4f",pair.second->centroid.x()/1e3,pair.second->centroid.y()/1e3,pair.second->centroid.z()/1e3);
        ImGui::Text("%s",str.c_str());
        ImGui::NextColumn();

        auto dims = (pair.second->bbox_max - pair.second->bbox_min);
        str = string_format("%7.4f %7.4f %7.4f",dims.x()/1e3,dims.y()/1e3,dims.z()/1e3);
        ImGui::Text("%s",str.c_str());
        ImGui::NextColumn();

        std::stringstream  ss;
        for(auto n : pair.second->neighbors) ss << n << " ";
        ImGui::Text("[%zu]: %s",pair.second->neighbors.size(), ss.str().c_str());
        ImGui::NextColumn();
        ImGui::Separator();
    }
    ImGui::Columns(1);
    ImGui::EndChild();
}

void GraphSLAMGUI::GetSurfelColor(Eigen::Vector3f& surfel_color, const inseg_lib::Surfel *surfel){
    switch (mColorType) {
        case COLOR_LABEL: {
            const cv::Vec3b &color = inseg_lib::CalculateLabelColor(surfel->label);
            surfel_color << color(2) / 255.0f, color(1) / 255.0f, color(0) / 255.0f;
        }
            break;
        case COLOR_PHONG:
            surfel_color << 0.9, 0.9, 0.9;
            break;
        case COLOR_NORMAL:
            surfel_color << (surfel->normal.x() + 1.f) / 2.f, (surfel->normal.y() + 1.f) /
                                                              2.f, -surfel->normal.z();
            break;
        case COLOR_COLOR:
            surfel_color << surfel->color[2] / 255.0f, surfel->color[1] / 255.0f, surfel->color[0] / 255.0f;
            break;
        case COLOR_UPDATED: {
            const cv::Vec3b &color = inseg_lib::CalculateLabelColor(surfel->label);
            if (mpGraphSLAM->mLastUpdatedSegments.find(surfel->label) == mpGraphSLAM->mLastUpdatedSegments.end())
                surfel_color << 0.3f * (color(2) / 255.0f), 0.3f * (color(1) / 255.0f), 0.3f *
                                                                                        (color(0) / 255.0f);
            else
                surfel_color << color(2) / 255.0f, color(1) / 255.0f, color(0) / 255.0f;

        }
            break;
        case COLOR_SEMANTIC: {
            auto label = mpGraphSLAM->GetGraph()->nodes.at(surfel->label)->GetLabel();
#ifdef COMPILE_WITH_GRAPHPRED
            if (label != Node::Unknown()) {
                if (mpGraphSLAM->GetGraphPred()->GetParams().at("label_type").string_value() == "NYU40" ||
                    mpGraphSLAM->GetGraphPred()->GetParams().at("label_type").string_value() == "ScanNet20") {
                    auto &name = label;
                    auto idx  = NYU40Name2Labels.at(name);
                    const auto& color_ = NYU40LabelColors.at(idx);
                    surfel_color << color_.r / 255.0f, color_.g / 255.0f, color_.b / 255.0f;
                } else {
                    const cv::Vec3b &color = inseg_lib::CalculateLabelColor(mpGraphSLAM->GetGraphPred()->mLabelsName2Idx.at(label));
                    surfel_color << color(2) / 255.0f, color(1) / 255.0f, color(0) / 255.0f;
                }
            } else
#endif
            {
                surfel_color << 0.f, 0.f, 0.f;
            }
            break;
        }
        case COLOR_INSTANCE: {
            const cv::Vec3b &color = inseg_lib::CalculateLabelColor(mpGraphSLAM->GetGraph()->nodes.at(surfel->label)->instance_idx);
            surfel_color << color(2) / 255.0f, color(1) / 255.0f, color(0) / 255.0f;
            break;
        }
        case COLOR_PANOPTIC: {
            const auto &name = mpGraphSLAM->GetGraph()->nodes.at(surfel->label)->GetLabel();

            if (name != Node::Unknown() && (name == "wall" || name == "floor")) {
                auto idx  = NYU40Name2Labels.at(name);
                const auto& color_ = NYU40LabelColors.at(idx);
                surfel_color << color_.r / 255.0f, color_.g / 255.0f, color_.b / 255.0f;
            } else {
                auto inst =  mpGraphSLAM->GetGraph()->nodes.at(surfel->label)->instance_idx.load();
#ifdef COMPILE_WITH_GRAPHPRED
                if (inst < (int) mpGraphSLAM->GetGraphPred()->mLabels.size()) {
                    const auto & name2 = mpGraphSLAM->GetGraphPred()->mLabels.at(inst);
                    if (name2 == "wall" || name2 == "floor") {
                        inst+=mpGraphSLAM->GetGraphPred()->mLabels.size();
                    }
                }
#endif
                const cv::Vec3b &color = inseg_lib::CalculateLabelColor(inst);
                surfel_color << (float)color(2) / 255.0f, (float)color(1) / 255.0f, (float)color(0) / 255.0f;
            }
            break;
        }
    }

//    if (mpGraphSLAM->GetGraph()->nodes.at(surfel->label)->GetLabel() == Node::Unknown())//TODO:remove me
//        surfel_color << 255.f, 255.f, 255.f;

};

void GraphSLAMGUI::GetEdgeColor(Eigen::Vector3f& edge_color, const PSLAM::Edge *edge) {
#ifdef COMPILE_WITH_GRAPHPRED
    switch (mpGraphSLAM->GetGraphPred()->mRelationshipsName2Idx.at(edge->GetLabel())) {
        case 0:
            edge_color << 255.f , 0.f, 0.f;
            break;
        case 1:
            edge_color << 255.f , 128.f, 0.f;
            break;
        case 2:
            edge_color << 255.f , 255.f, 0.f;
            break;
        case 3:
            edge_color << 128.f , 255.f, 0.f;
            break;
        case 4:
            edge_color << 0.f , 255.f, 0.f;
            break;
        case 5:
            edge_color << 0.f , 255.f, 128.f;
            break;
        case 6:
            edge_color << 0.f , 255.f, 255.f;
            break;
        case 7:
            edge_color << 0.f , 128.f, 255.f;
            break;
        case 8:
            edge_color << 0.f , 0.f, 255.f;
            break;
        default:
            SCLOG(ERROR) << "do not have edge color for " << edge->GetLabel() << " with index(" <<
                    mpGraphSLAM->GetGraphPred()->mRelationshipsName2Idx.at(edge->GetLabel()) << ")";
    }
#else
    edge_color << 0.f, 255.f, 0.f;
#endif
}

std::string GraphSLAMGUI::GetNodeLabel(const Node *node) const{
    switch (static_cast<LABELTYPE>(mLabelType)) {
        case LABEL_SEGMENT:
        {
            return std::to_string(node->idx);
        }
        case LABEL_INSTANCE:
        {
            return std::to_string(node->instance_idx);
        }
        case LABEL_NAME:
        {
            return node->GetLabel();
        }
    }
    return "";
}

std::string GraphSLAMGUI::GetEdgeLabel(const Edge *edge){
    std::string text;
    const auto &nodes = mpGraphSLAM->GetGraph()->nodes;
    const auto &nodeFrom = nodes.at(edge->nodeFrom);
    const auto &nodeTo = nodes.at(edge->nodeTo);
    switch (static_cast<LABELTYPE>(mLabelType)) {
        case LABEL_SEGMENT:
        {
            text = std::to_string(nodeFrom->idx)+"_"+edge->GetLabel()+"_"+std::to_string(nodeTo->idx);
            break;
        }
        case LABEL_INSTANCE:
        {
            text = std::to_string(nodeFrom->instance_idx)+"_"+edge->GetLabel()+"_"+std::to_string(nodeTo->instance_idx);
            break;
        }
        case LABEL_NAME:
        {
            text = nodeFrom->GetLabel() + "_" +
                   edge->GetLabel() + "_" +
                   nodeTo->GetLabel();
            break;
        }
    }
    return text;
}

void GraphSLAMGUI::SetRender(int width, int height, const std::string &path, bool align) {
#ifdef COMPILE_WITH_ASSIMP
    std::string folder, scan_id;
    PSLAM::MeshRenderType type;
    if(path.find("scene") != std::string::npos) {
        auto parent_folder = tools::PathTool::find_parent_folder(path, 1);
        scan_id = tools::PathTool::getFileName(parent_folder);
        folder =  tools::PathTool::find_parent_folder(parent_folder, 1);
        type = PSLAM::MeshRenderType_ScanNet;
    } else {
        auto seq_folder = tools::PathTool::find_parent_folder(path,1);
        scan_id = tools::PathTool::getFileName(seq_folder);
        folder = tools::PathTool::find_parent_folder(seq_folder,1);
        type = PSLAM::MeshRenderType_3RScan;
    }
    mMeshRender.reset( PSLAM::MakeMeshRenderer(width, height, folder,scan_id,type,align) );
#else
    throw std::runtime_error("did not compile with assimp");
#endif
}

void GraphSLAMGUI::RecordImg() {
    glfwGetFramebufferSize(window_->window, &window_->runtimeWidth, &window_->runtimeHeight);
    cv::Mat img(window_->runtimeHeight, window_->runtimeWidth, CV_8UC4);
    glReadBuffer( GL_FRONT );
    glReadPixels(0, 0, window_->runtimeWidth, window_->runtimeHeight, GL_RGBA, GL_UNSIGNED_BYTE,
                 img.data);
    cv::flip(img, img, 0);
    cv::cvtColor(img, img, cv::COLOR_RGBA2BGRA);
    static int iterSave=0;
    static const std::string pth_to_image_folder = "./imgs";
    char name[pth_to_image_folder.length() + 100];
    sprintf(name, (pth_to_image_folder + "/color%04d.png").c_str(), iterSave);
    tools::PathTool::create_folder(pth_to_image_folder);
    cv::imwrite(name, img);
    printf("Image saved to %s\n", name);
    iterSave++;
}
