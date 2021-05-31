//
// Created by sc on 8/24/20.
//
#ifdef COMPILE_WITH_ONNX
#include "GraphPredictor.h"
#include "MemoryBlock.h"
#include "OnnxModelBase.h"
#include <ORUtils/LogUtil.h>
using namespace PSLAM;

void find_nodes_has_same_part(const std::map<int, NodePtr> &nodes, unsigned int s_idx, int t_idx, std::set<int> &has,
                              std::set<int> &exclude){
    if(nodes.find(s_idx) == nodes.end()) return;
    exclude.insert(s_idx);
    for(const auto &edge:nodes.at(s_idx)->edges) {
        if (nodes.find(edge->nodeFrom) == nodes.end()) {
            continue;
            SCLOG(ERROR) << "cannot find nodeFrom: " << edge->nodeFrom;
        }
        if (nodes.find(edge->nodeTo) == nodes.end()) {
            continue;
            SCLOG(ERROR) << "cannot find nodeFrom: " << edge->nodeTo;
        }
        if (exclude.find(edge->nodeTo) != exclude.end()) continue; // skip if excluded
        auto& nodeFrom = nodes.at(edge->nodeFrom);
        auto& nodeTo   = nodes.at(edge->nodeTo);
        if (nodeFrom->GetLabel() != nodeTo->GetLabel()) continue; // skip if label mis-match
        if (nodeFrom->neighbors.find(edge->nodeTo) == nodeFrom->neighbors.end()) continue; // if target node is not the neighbor of the source node
        if(!nodeFrom->CheckConnectivity(nodeTo.get(),5,false)) continue; // if they are not connected, continue
        if (edge->GetLabel() == Edge::Same()) {
            has.insert(edge->nodeTo);
            find_nodes_has_same_part(nodes,edge->nodeTo,t_idx,has,exclude);
        }
    }
}

static MemoryBlock2D ConcatObjFeature(size_t n_node, size_t dim_obj_feature, Ort::Value &obj_feature,
                                      const MemoryBlock2D &descriptor) {
    /// Concat descriptor to the object feature
    MemoryBlock2D out(MemoryBlock::DATA_TYPE::FLOAT, {static_cast<unsigned long>(n_node), dim_obj_feature + 8});
    MemoryBlock2D tmp_out_enc_obj(obj_feature.GetTensorMutableData<float>(), MemoryBlock::FLOAT,
                                  {static_cast<unsigned long>(n_node), dim_obj_feature});
    for (size_t n = 0; n < n_node; ++n) {
        MemoryBlock tmp (descriptor.Get<float>(n,3),descriptor.type(),8);
//                SCLOG(VERBOSE) << tmp;
        size_t offset= 0;
        memcpy(out.Get<float>(n, offset), tmp_out_enc_obj.Get<float>(n,0), sizeof(float)*dim_obj_feature);
        offset += dim_obj_feature;
        memcpy(out.Get<float>(n, offset), tmp.Get<float>(), sizeof(float)*6);
        offset += 6;
        out.at<float>(n, offset) = std::log( tmp.at<float>(6) );
        offset+=1;
        out.at<float>(n, offset) = std::log( tmp.at<float>(7) );
    }
    return out;
}

void calculateBBox(const std::vector<float> &points, float *min, float *max){
    assert(points.size()%3==0);
    for(size_t i=0;i<points.size()/3;++i){
        for(size_t j=0;j<3;++j){
            if(points[3*i+j]<min[j]) min[j]=points[3*i+j];
            if(points[3*i+j]>max[j]) max[j]=points[3*i+j];
        }
    }
}

GraphPredictor::GraphPredictor(const ConfigPSLAM *configPslam):mConfigPslam(configPslam), mUpdated(false){
    source_to_target = false;
    auto path_onnx = configPslam->pth_model+"/";
    mbThread = configPslam->use_thread;
//    mConfigPaths = std::make_unique<ONNX_Model_PATHS>(path_onnx);
    mMemoryInfo = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
//    mPointNetGCN = std::make_unique<PointNetGCN>(mConfigPaths.get(),mMemoryInfo.get(),mbVerbose);

    mEatGCN = std::make_unique<EatGCN>(path_onnx, mMemoryInfo.get());

    /// Read labels
    {
        std::fstream file(path_onnx+"classes.txt", std::ios::in);
        if(!file.is_open())SCLOG(ERROR) << "Cannot open class file at path " << path_onnx+"classes.txt";
        std::string line;
        std::vector<std::string> labels;
        while (std::getline(file, line)) labels.push_back(line);

        SCLOG(VERBOSE) << "Labels";
        for(size_t i=0;i<labels.size();++i) {
            mLabels.insert({i, labels[i]});
            mLabelsName2Idx.insert({labels[i],i});
            SCLOG(VERBOSE) << i << ": " << labels[i];
        }
    }
    {
        std::fstream file(path_onnx+"relationships.txt", std::ios::in);
        if(!file.is_open())SCLOG(ERROR) << "Cannot open relationship file at path " << path_onnx+"relationships.txt";
        std::string line;
        std::vector<std::string> labels;
        while (std::getline(file, line)) labels.push_back(line);
        SCLOG(VERBOSE) << "Relationships";
        for(size_t i=0;i<labels.size();++i) {
            mRelationships.insert({i,labels[i]});
            mRelationshipsName2Idx.insert({labels[i],i});
            SCLOG(VERBOSE) << i << ": " << labels[i];
        }
    }
}

GraphPredictor::~GraphPredictor(){
    mbShouldStop=true;
    if(mbThread) {
        if(mThread.valid()) {
            //wait until stop
            while (mThread.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
                mThread.wait_for(std::chrono::milliseconds(50));
            mThread.get();
        }
    }
}

std::shared_ptr<PSLAM::GraphPredictor::Prediction> GraphPredictor::Predict(
        const std::shared_ptr<GCNINPUTDATA>& data) {
    {
        std::tuple<MemoryBlock2D, MemoryBlock2D> result;
        auto& edge_index = std::get<2>(*data);
        auto& idx2seg = std::get<3>(*data);
        result = mEatGCN->Run(std::get<0>(*data), std::get<1>(*data),std::get<2>(*data));
        auto& cls_obj = std::get<0>(result);
        auto& cls_rel = std::get<1>(result);


        std::map<size_t,std::map<std::string,float>> output_objs;
        for(size_t i=0;i<cls_obj.mDims.x;++i){
            std::map<std::string,float> m;
            for(size_t j=0;j<cls_obj.mDims.y;++j){
                m[mLabels.at(j)] = cls_obj.at<float>(i,j);
            }
            output_objs[idx2seg.at(i)] = std::move(m);
        }
//        std::map<size_t,size_t> output_objs;
//        for(size_t i=0;i<cls_obj.size();++i){
//            output_objs[idx2seg.at(i)] = cls_obj.at(i)+1;
//        }

        std::vector<EdgePtr> output_edges;
        for(size_t i=0;i<cls_rel.mDims.x;++i){
            output_edges.emplace_back(new Edge());
            auto &edge = output_edges.back();
            edge->nodeFrom = idx2seg.at( edge_index.at<int64_t>(0,i) );
            edge->nodeTo = idx2seg.at( edge_index.at<int64_t>(1,i) );
            for(size_t j=0;j<cls_rel.mDims.y;++j) {
                edge->labelProp[mRelationships.at(j)] = cls_rel.at<float>(i,j);
            }
        }

        return std::make_shared<Prediction>(
                std::move(output_objs), std::move(output_edges), std::move(std::get<std::map<int, std::pair<size_t,size_t>>>(*data)));
    }
}

void GraphPredictor::RUN(Graph *graph) {
    mpGraph = graph;
    mbShouldStop = false;
    if(mbThread) {
        if (mThread.valid()) {
            while (mThread.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
                mThread.wait_for(std::chrono::milliseconds(50));
            mThread.get();
        } else {
            mThread = std::async(std::launch::async,
                                 std::bind(&PSLAM::GraphPredictor::Process_IMPL, this));
        }

    } else {
        Process_IMPL();
    }
}

void GraphPredictor::Stop() {
    if(!mbThread) return;
    mbShouldStop=true;
    if (mThread.valid()) {
        while (mThread.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
            mThread.wait_for(std::chrono::milliseconds(50));
        mThread.get();
    }
}

void GraphPredictor::Process_IMPL() {
    bool runOnce=!mbThread;
    bool should_break = false;
    while (!mbShouldStop) {
        if(should_break) break;
        if(runOnce)should_break=true;
        /// Check if there are nodes that need to be updated
        std::map<int, NodePtr> vNodes;

        {
            // sleep this thread for a short time to prevnt continuous lock
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            std::unique_lock<std::mutex> lock(mpGraph->mMutNode);
            for (const auto &node : mpGraph->nodes) {
                if (!node.second->mbNeedUpdateNodeFeature) continue;
                vNodes.insert(node);
            }
        }
        if (vNodes.empty()) continue;

        size_t levels = GetParams().at("n_layers").int_value()+1; // 0: geometric feature, 1: gcn1 2: gcn2
        std::vector<std::map<int, NodePtr>> selected_nodes_level(levels);
        selected_nodes_level.at(0) = vNodes;
        for (size_t l = 1; l < levels; ++l) {
            auto nns = getNeighbor(selected_nodes_level.at(l-1), mConfigPslam->filter_num_node);
            for(auto &nn : nns)
                selected_nodes_level.at(l)[nn.first] = nn.second;
            for(auto &nn : selected_nodes_level.at(l-1))
                selected_nodes_level.at(l)[nn.first] = nn.second;
        }
        /// Save the current size and edge of all levels
        std::map<int,std::pair<size_t,size_t>> sizeAndEdge;
        {
            const auto &last_level = selected_nodes_level.back();
            for(const auto &pair : last_level) {
                auto i = pair.first;
                const auto &node = pair.second;
                sizeAndEdge[i].first = node->surfels.size();
                sizeAndEdge[i].second=0;
                std::unique_lock<std::mutex> lock(node->mMutNode);
                for (const auto &nn : node->neighbors) {
                    if(last_level.find(nn) == last_level.end()) continue;
                    sizeAndEdge.at(i).second += 1;
                }
            }
        }

        /// Update node feature
        SCLOG(VERBOSE) << "UpdateNodeFeature";
        CTICK("[Process_IMPL]1.updateNodeFeature");
        TICK("[Process_IMPL]1.updateNodeFeature");
        UpdateNodeFeature(selected_nodes_level.at(0));
//        UpdateNodeFeature(selected_nodes_level.back());
        TOCK("[Process_IMPL]1.updateNodeFeature");
        CTOCK("[Process_IMPL]1.updateNodeFeature");

        /// Update edge feature
        SCLOG(VERBOSE) << "UpdateEdgeFeature";
        CTICK("[Process_IMPL]2.updateEdgeFeature");
        TICK("[Process_IMPL]2.updateEdgeFeature");
        UpdateEdgeFeature(selected_nodes_level.at(1));
//        UpdateEdgeFeature(selected_nodes_level.back());
        TOCK("[Process_IMPL]2.updateEdgeFeature");
        CTOCK("[Process_IMPL]2.updateEdgeFeature");

        /// Update GCN feature
        SCLOG(VERBOSE) << "updateGCNFeature";

        if (GetParams().at("USE_GCN").bool_value()) {
            for (size_t l = 0; l < size_t(GetParams().at("n_layers").int_value()); ++l) {
                CTICK("[Process_IMPL]3.updateGCNFeature"+std::to_string(l));
                TICK("[Process_IMPL]3.updateGCNFeature"+std::to_string(l));
                UpdateGCNFeature(selected_nodes_level.at(l + 1), l);
//                UpdateGCNFeature(selected_nodes_level.back(), l);
                TOCK("[Process_IMPL]3.updateGCNFeature"+std::to_string(l));
                CTOCK("[Process_IMPL]3.updateGCNFeature"+std::to_string(l));
            }
        }


        /// Predict only the selected nodes
        SCLOG(VERBOSE) << "UpdatePrediction";
        CTICK("[Process_IMPL]4.updatePrediction");
        TICK("[Process_IMPL]4.updatePrediction");
        UpdatePrediction(selected_nodes_level.back(),sizeAndEdge);
        TOCK("[Process_IMPL]4.updatePrediction");
        CTOCK("[Process_IMPL]4.updatePrediction");

        mUpdated=true;
    }
    SCLOG(VERBOSE) << "stoped!";
}

void GraphPredictor::UpdateNodeFeature(const std::map<int,NodePtr> &vNodes) {
    SCLOG(VERBOSE) << "node feature start";
    auto n_nodes = vNodes.size();
    size_t n_pts = mConfigPslam->n_pts;

    size_t d_pts = 3;
    if (GetParams().at("USE_RGB").bool_value()) d_pts += 3;
    if (GetParams().at("USE_NORMAL").bool_value()) d_pts += 3;
    size_t d_edge = GetParams().at("dim_edge").int_value();

    /// Compute Node Shape Feature
    //// build input
    MemoryBlock3D input_nodes(MemoryBlock::DATA_TYPE::FLOAT,
                              {n_nodes, d_pts, n_pts});
    float max_dist, dist;
    CTICK("[UpdateNodeFeature]1.computeAndCopy");
    SCLOG(VERBOSE) << "computeAndCopy";
    size_t n=0;
    std::vector<SurfelPtr> selected_surfels;
    for (const auto &pair: vNodes) {
        auto &node = pair.second;
        const auto &centroid = node->mCentroid * 1e-3;
        max_dist = 0;
        {

            {
                std::unique_lock<std::mutex> lock (node->mMutSelected);
                if (node->selected_surfels.size() < n_pts) {
                    continue;
                    SCLOG(ERROR) << "";
                }
//                n_pts = selected_surfels.size();
                selected_surfels = node->selected_surfels;
            }


            const auto &surfels = selected_surfels;
            CTICK("[UpdateNodeFeature]1.1.copySurfels");
            try
            {
                for (size_t p = 0; p < n_pts; ++p) {
                    const auto &surfel = surfels.at(p);
                    dist = 0;
                    for (size_t d = 0; d < 3; ++d) {
                        input_nodes.at<float>(n, d, p) = surfel->pos[d] / 1000.f - centroid[d];
                        input_nodes.at<float>(n, d + 3, p) = float(surfel->color[2 - d]) / 255.f * 2.f - 1.f;
                        input_nodes.at<float>(n, d + 6, p) = surfel->normal[d];
                        dist += std::pow(input_nodes.at<float>(n, d, p), 2);
                    }
                    if (dist > max_dist) max_dist = dist;
                }
            }
            catch (std::out_of_range& e)
            {
                SCLOG(ERROR) << e.what();
            }

            CTOCK("[UpdateNodeFeature]1.1.copySurfels");
        }

        CTICK("[UpdateNodeFeature]1.2.normPts");
        max_dist = std::sqrt(max_dist);
        for (size_t p = 0; p < n_pts; ++p)
            for (size_t d = 0; d < 3; ++d)
                input_nodes.at<float>(n, d, p) /= max_dist;
        CTOCK("[UpdateNodeFeature]1.2.normPts");


        if(mDebug)
            ONNX::PrintVector<float,size_t>("input_nodes", input_nodes.Get<float>(n,0,0),
                                            {d_pts, n_pts});

        n++;
    }
    CTOCK("[UpdateNodeFeature]1.computeAndCopy");

    MemoryBlock2D descriptor_full(MemoryBlock::DATA_TYPE::FLOAT, {n_nodes, d_edge});
    {
        CTICK("[UpdateNodeFeature]2.collectInfoFromNodes");
        SCLOG(VERBOSE) << "collectInfoFromNodes";
        size_t i=0;
        for(const auto &pair : vNodes) {
//        for(size_t i=0;i<n_nodes;++i) {
            const auto &node = pair.second;
            const auto centroid = node->mCentroid*1e-3;
            const auto stdev    = node->mStd*1e-3;
            const auto bbox_min = node->mBBox_min*1e-3;
            const auto bbox_max = node->mBBox_max*1e-3;

            Eigen::Vector3f dim = (bbox_max-bbox_min);
            float volume = 1;
            for (size_t d = 0; d < 3; ++d) volume *= dim[d];
            float length = *std::max_element(dim.begin(), dim.end());

            for (size_t d = 0; d < 3; ++d) {
                descriptor_full.Row<float>(i)[d] = centroid[d];
                descriptor_full.Row<float>(i)[d + 3] = stdev[d];
                descriptor_full.Row<float>(i)[d + 6] = dim[d];
            }
            descriptor_full.Row<float>(i)[9] = volume;
            descriptor_full.Row<float>(i)[10] = length;
            i++;
        }
        CTOCK("[UpdateNodeFeature]2.collectInfoFromNodes");
    }

    if(mDebug)
        ONNX::PrintVector<float,size_t>("descriptor_full", descriptor_full.Get<float>(),
                                         {descriptor_full.mDims.x,descriptor_full.mDims.y});

    CTICK("[UpdateNodeFeature]3.compute");
    SCLOG(VERBOSE) << "compute";
    std::vector<float *> obj_inputs = {reinterpret_cast<float *>(input_nodes.data())};
    std::vector<std::vector<int64_t>> obj_input_sizes = {{static_cast<long>(n_nodes), static_cast<long>(d_pts), static_cast<long>(n_pts)}};
    auto out_enc_obj = mEatGCN->Compute(PSLAM::EatGCN::OP_ENC_OBJ, obj_inputs, obj_input_sizes);
    size_t dim_obj_feature = out_enc_obj[0].GetTensorTypeAndShapeInfo().GetShape()[1];
    auto out = ConcatObjFeature(n_nodes, dim_obj_feature, out_enc_obj[0], descriptor_full);
    CTOCK("[UpdateNodeFeature]3.compute");

    if(mDebug)
        ONNX::PrintVector<float,int64_t>("nodeFeature", out_enc_obj[0].GetTensorMutableData<float>(),
                {out_enc_obj[0].GetTensorTypeAndShapeInfo().GetShape()[0], out_enc_obj[0].GetTensorTypeAndShapeInfo().GetShape()[1]});

    /// Save Obj and Rel feature to nodes
    CTICK("[UpdateNodeFeature]4.UpdateNodeFeature");
    SCLOG(VERBOSE) << "UpdateNodeFeature";
    {
        size_t size_node_feature = out.mDims.y;
        size_t i = 0;
        for (const auto &pair : vNodes) {
//    for (size_t i=0; i < n_nodes; ++i) {
            auto &node = pair.second;
            auto &node_feature = node->mFeatures["0"];
            if (!node_feature) node_feature.reset(new MemoryBlock(MemoryBlock::FLOAT, size_node_feature));
            auto *feature = out.Row<float>(i);
            memcpy(node_feature->Get<float>(), feature, sizeof(float) * size_node_feature);
            node->mbNeedUpdateNodeFeature = false;
            i++;
        }
    }
    CTOCK("[UpdateNodeFeature]4.UpdateNodeFeature");



    SCLOG(VERBOSE) << "node feature end";
}

void GraphPredictor::UpdateEdgeFeature(const std::map<int,NodePtr> &vNodes){
    SCLOG(VERBOSE) << "edge feature start";
    // Build input for edges
//    const auto &nodes = mpGraph->nodes;
    size_t d_edge = GetParams().at("dim_edge").int_value();
    std::map<size_t, size_t> idx2seg;
    std::map<size_t, size_t> seg2idx;

    /// Collect the neighbor of the selected nodes
//    std::set<int> selected_nodes_idx;
//    for(const auto &node : vNodes) selected_nodes_idx.insert(node->idx);

    /// Build descriptor for these nodes
    SCLOG(VERBOSE) << "Build descriptor for these nodes";
    MemoryBlock2D descriptor_full(MemoryBlock::DATA_TYPE::FLOAT, {vNodes.size(), d_edge});
    {
        CTICK("[UpdateEdgeFeature]1.collectInfoFromNodes");
        size_t i=0;
        for(const auto &pair : vNodes) {
            const auto &node = pair.second;
            idx2seg[i] = node->idx;
            seg2idx[node->idx] = i;

            const auto centroid = node->mCentroid * 1e-3;
            const auto stdev = node->mStd * 1e-3;
            const auto bbox_min = node->mBBox_min * 1e-3;
            const auto bbox_max = node->mBBox_max * 1e-3;

            Eigen::Vector3f dim = (bbox_max - bbox_min);
            float volume = 1;
            for (size_t d = 0; d < 3; ++d) volume *= dim[d];
            float length = *std::max_element(dim.begin(), dim.end());

            for (size_t d = 0; d < 3; ++d) {
                descriptor_full.Row<float>(i)[d] = centroid[d];
                descriptor_full.Row<float>(i)[d + 3] = stdev[d];
                descriptor_full.Row<float>(i)[d + 6] = dim[d];
            }
            descriptor_full.Row<float>(i)[9] = volume;
            descriptor_full.Row<float>(i)[10] = length;

            if(mDebug) {
                std::stringstream ss;//TODO: remove me
                ss << "[" << node->idx << "] ";
                for (size_t j = 0; j < d_edge; ++j)
                    ss << descriptor_full.Row<float>(i)[j] << " ";
                SCLOG(DEBUG) << ss.str();
            }

            i++;
        }
        CTOCK("[UpdateEdgeFeature]1.collectInfoFromNodes");
    }

    /// Count Edges
    CTICK("[UpdateEdgeFeature]2.countEdges");
    SCLOG(VERBOSE) << "countEdges";
    std::vector<std::pair<int,int>> selected_edges;
    {
//        std::unique_lock<std::mutex> lock(mpGraph->mMutEdge);
        for(const auto & pair : vNodes) {
            const auto &n = pair.first;
            std::unique_lock<std::mutex> lock(pair.second->mMutNN);
            for (const auto &nn : pair.second->neighbors) {
                if (vNodes.find(nn) == vNodes.end()) continue;
                if (seg2idx.find(nn) == seg2idx.end()) continue;
                if (seg2idx.at(n) == seg2idx.at(nn)) continue;
                selected_edges.emplace_back(seg2idx.at(n),seg2idx.at(nn));
            }
        }
    }
    size_t n_edges = selected_edges.size();
    CTOCK("[UpdateEdgeFeature]2.countEdges");

    if(mDebug)
    {//TODO: remove me
        std::stringstream ss;
        ss << "edges:\n";
        for(auto &pair:selected_edges)
            ss << pair.first << ", " << pair.second << "\n";

        SCLOG(DEBUG) << ss.str();
    }


    /// Compute Edge descriptor
    CTICK("[UpdateEdgeFeature]3.buildEdgeIndex");
    SCLOG(VERBOSE) << "buildEdgeIndex";
    std::map<int, std::pair<size_t, size_t>> nodeSizeEdgeTracker;
    MemoryBlock2D edge_index(MemoryBlock2D::INT64_T, { 2, n_edges });
    for(size_t i=0;i<selected_edges.size();++i) {
        edge_index.at<int64_t>(0, i) = selected_edges.at(i).first;
        edge_index.at<int64_t>(1, i) = selected_edges.at(i).second;
    }
    CTOCK("[UpdateEdgeFeature]3.buildEdgeIndex");

    /// Find Nodes and Edges feature
    CTICK("[UpdateEdgeFeature]4.computeEdgeFeature");
    SCLOG(VERBOSE) << "computeEdgeFeature";
    auto edge_descriptor = DataUtil::compute_edge_descriptor(descriptor_full, edge_index, source_to_target);
    std::vector<float *> rel_inputs = {static_cast<float *>(edge_descriptor.data())};
    std::vector<std::vector<int64_t>> rel_input_sizes = {{static_cast<long>(n_edges), static_cast<long>(d_edge), 1}};
    auto out_enc_rel = mEatGCN->Compute(PSLAM::EatGCN::OP_ENC_REL, rel_inputs, rel_input_sizes);
    CTOCK("[UpdateEdgeFeature]4.computeEdgeFeature");

    //TODO: remove me
    if(mDebug)
    {
        ONNX::PrintVector<float,size_t>("edge_descriptor",edge_descriptor.Get<float>(), {edge_descriptor.mDims.x,edge_descriptor.mDims.y});
        ONNX::PrintTensorShape("out_enc_rel",out_enc_rel[0]);
        ONNX::PrintVector<float>("out_enc_rel",out_enc_rel[0].GetTensorMutableData<float>(),
                std::vector<size_t>{static_cast<unsigned long>(out_enc_rel[0].GetTensorTypeAndShapeInfo().GetShape()[0]),
                 static_cast<unsigned long>(out_enc_rel[0].GetTensorTypeAndShapeInfo().GetShape()[1])});
    }


    CTICK("[UpdateEdgeFeature]5.copyEdegeFeature");
    SCLOG(VERBOSE) << "copyEdegeFeature";
    const auto &edges = mpGraph->edges;
    int64_t size_edge_feature = out_enc_rel[0].GetTensorTypeAndShapeInfo().GetShape()[1];
    for (size_t i=0; i < n_edges; ++i) {
        auto idx_from = idx2seg.at( edge_index.at<int64_t>(0,i) );
        auto idx_to = idx2seg.at( edge_index.at<int64_t>(1,i) );
        EdgePtr edge;
        {
            std::unique_lock<std::mutex> lock(mpGraph->mMutEdge);
            if (edges.find({idx_from,idx_to}) == edges.end()) {
                edge = std::make_shared<PSLAM::Edge>();
                edge->nodeFrom=idx_from;
                edge->nodeTo=idx_to;
            } else {
                edge = edges.at({idx_from,idx_to});
            }
        }




//        SCLOG(VERBOSE) << "get feature";
        auto &edge_feature = edge->mFeatures["0"];
        if (!edge_feature) edge_feature.reset(new MemoryBlock(MemoryBlock::FLOAT, size_edge_feature));
        float *feature = out_enc_rel[0].GetTensorMutableData<float>()+i*size_edge_feature;
        memcpy(edge_feature->Get<float>(),feature,sizeof(float)*size_edge_feature);
        mpGraph->AddEdge(edge);
    }
    CTOCK("[UpdateEdgeFeature]5.copyEdegeFeature");

    SCLOG(VERBOSE) << "edge feature end";
}

void GraphPredictor::UpdateGCNFeature(const std::map<int,NodePtr> &vNodes, size_t level){
    SCLOG(VERBOSE) << "Update GCN " << level << " feature start";
    const auto &edges = mpGraph->edges;
    std::map<size_t, size_t> idx2seg;
    std::map<size_t, size_t> seg2idx;
    /// Filter out nodes without computed feature
//    std::vector<NodePtr> selected_nodes;
    std::map<int,NodePtr> selected_nodes;
    {
        for(const auto & node : vNodes){
            if(node.second->mFeatures.find(std::to_string(level)) != node.second->mFeatures.end())
                selected_nodes.insert(node);//.push_back(node);
        }
    }

    size_t n_nodes = selected_nodes.size();
    if(n_nodes==0){
        SCLOG(VERBOSE) << "got 0 nodes. return";
        return;
    }
    {
        size_t i=0;
        for (const auto &pair:selected_nodes) {
            idx2seg[i] = pair.second->idx;
            seg2idx[pair.second->idx] = i;
            i++;
        }
    }



    /// Count Edges
    SCLOG(VERBOSE) << "Update countEdges";
    CTICK("[UpdateGCNFeature]1.countEdges");
    std::vector<EdgePtr> selected_edges;
    {
        for(auto & selected_node : selected_nodes) {
            const auto &n = selected_node.first;
            if (seg2idx.find(n) == seg2idx.end()) continue;
            std::unique_lock<std::mutex> lock(selected_node.second->mMutNN);
            for (const auto &nn : selected_node.second->neighbors) {
                if (selected_nodes.find(nn) == selected_nodes.end()) continue;
                if (seg2idx.find(nn) == seg2idx.end()) continue;
                if (seg2idx.at(n) == seg2idx.at(nn)) continue;
                std::unique_lock<std::mutex> lock(mpGraph->mMutEdge);
                if(edges.find({n,nn}) == edges.end()) continue;
                if(edges.at({n,nn})->mFeatures.find(std::to_string(level)) ==
                        edges.at({n,nn})->mFeatures.end()) continue;
                selected_edges.push_back(edges.at({n,nn}));
            }
        }
    }
    size_t n_edges = selected_edges.size();
    CTOCK("[UpdateGCNFeature]1.countEdges");
    if(n_edges==0){
        SCLOG(VERBOSE) << "got 0 edges. return";
        return;
    }

    /// Compute Edge descriptor
    SCLOG(VERBOSE) << "buildEdgeIndex";
    CTICK("[UpdateGCNFeature]2.buildEdgeIndex");
    std::map<int, std::pair<size_t, size_t>> nodeSizeEdgeTracker;
    MemoryBlock2D edge_index(MemoryBlock2D::INT64_T, { 2, n_edges });
    for(size_t i=0;i<selected_edges.size();++i) {
        edge_index.at<int64_t>(0, i) = seg2idx.at(selected_edges.at(i)->nodeFrom);
        edge_index.at<int64_t>(1, i) = seg2idx.at(selected_edges.at(i)->nodeTo);
    }
    CTOCK("[UpdateGCNFeature]2.buildEdgeIndex");

    if(mDebug){
        std::stringstream ss;
        ss << "edges\n";
        for(size_t i=0;i<n_edges;++i)
            ss << edge_index.at<int64>(0,i) << ", " << edge_index.at<int64_t>(1,i) << "\n";
        SCLOG(DEBUG) << ss.str();
    }


    /// Build Node Input
    int64_t dim_obj_f = GetParams().at("dim_o_f").int_value();
    MemoryBlock2D nodeFeatures(MemoryBlock::FLOAT, {selected_nodes.size(), static_cast<unsigned long>(dim_obj_f)});
    SCLOG(VERBOSE) << "copyNodeFeature";
    CTICK("[UpdateGCNFeature]3.copyNodeFeature");
    {
        size_t i=0;
        for(const auto &pair:selected_nodes){
            auto &node = pair.second;
            const auto *feature = node->mFeatures.at(std::to_string(level))->Get<float>();
            memcpy(nodeFeatures.Row<float>(i),feature,sizeof(float)*dim_obj_f);
            i++;
        }
    }
    CTOCK("[UpdateGCNFeature]3.copyNodeFeature");

    if(mDebug)
    ONNX::PrintVector<float,unsigned long>("nodeFeature", nodeFeatures.Get<float>(), {selected_nodes.size(), static_cast<unsigned long>(dim_obj_f)});

    /// Build Edge Input
    int64_t dim_rel_f = GetParams().at("dim_r_f").int_value();
    MemoryBlock2D edgeFeatures(MemoryBlock::FLOAT, {n_edges, static_cast<unsigned long>(dim_rel_f)});
    SCLOG(VERBOSE) << "copyEdgeFeature";
    CTICK("[UpdateGCNFeature]4.copyEdgeFeature");
    for(size_t i=0;i<n_edges;++i){
        auto &edge = selected_edges.at(i);
        const auto *feature = edge->mFeatures.at( std::to_string(level) )->Get<float>();
        memcpy(edgeFeatures.Row<float>(i),feature,sizeof(float)*dim_rel_f);

        if(mDebug)
        {//TODO: remove me
            std::stringstream ss;
            ss.precision(4);
            ss << "["<<edge->nodeFrom<<","<<edge->nodeTo<<"]:";
            for(size_t j=0;j<edge->mFeatures.at( std::to_string(level) )->size();++j) ss << feature[j] << " ";
            SCLOG(DEBUG) << ss.str();
        }
        if(mDebug)
        {//TODO: remove me
            std::stringstream ss;
            ss.precision(4);
            ss << "["<<edge->nodeFrom<<","<<edge->nodeTo<<"]:";
            for(size_t j=0;j<edge->mFeatures.at( std::to_string(level) )->size();++j) ss << edgeFeatures.Row<float>(i)[j] << " ";
            SCLOG(DEBUG) << ss.str();
        }
    }
    CTOCK("[UpdateGCNFeature]4.copyEdgeFeature");

    if(mDebug)
    ONNX::PrintVector<float,unsigned long>("edgeFeatures", edgeFeatures.Get<float>(), {n_edges, static_cast<unsigned long>(dim_rel_f)});

    /// Compute
    SCLOG(VERBOSE) << "compute";
    CTICK("[UpdateGCNFeature]5.compute");
    {
        size_t i_i = source_to_target ? 1 : 0;
        size_t i_j = source_to_target ? 0 : 1;

        std::vector<float> obj_f_i = DataUtil::Collect(nodeFeatures.Get<float>(),
                                                       edge_index.Row<int64_t>(i_i), dim_obj_f, n_edges);
        std::vector<float> obj_f_j = DataUtil::Collect(nodeFeatures.Get<float>(),
                                                       edge_index.Row<int64_t>(i_j), dim_obj_f, n_edges);
        if(mDebug){
            ONNX::PrintVector("obj_f_i", obj_f_i.data(),
                        std::vector<int64_t>{static_cast<long>(n_edges), dim_obj_f});
            ONNX::PrintVector("obj_f_j", obj_f_j.data(),
                        std::vector<int64_t>{static_cast<long>(n_edges), dim_obj_f});
        }

        std::vector<Ort::Value> inputs;
        inputs.emplace_back(ONNX::CreateTensor(mMemoryInfo.get(), obj_f_i.data(),
                {static_cast<long>(n_edges), dim_obj_f}));
        inputs.emplace_back(ONNX::CreateTensor(mMemoryInfo.get(), edgeFeatures.Get<float>(),
                {static_cast<long>(n_edges), dim_rel_f}));
        inputs.emplace_back(ONNX::CreateTensor(mMemoryInfo.get(), obj_f_j.data(),
                {static_cast<long>(n_edges), dim_obj_f}));

        auto output_atten = mEatGCN->ComputeGCN(PSLAM::EatGCN::OP_GCN_ATTEN, level, inputs);
        if (mDebug) {
            ONNX::PrintVector("output_atten", output_atten[0].GetTensorMutableData<float>(),
                        std::vector<int64_t>{output_atten[0].GetTensorTypeAndShapeInfo().GetShape()[0],
                                             output_atten[0].GetTensorTypeAndShapeInfo().GetShape()[1]});
            ONNX::PrintVector("output_edge", output_atten[1].GetTensorMutableData<float>(),
                        std::vector<int64_t>{output_atten[1].GetTensorTypeAndShapeInfo().GetShape()[0],
                                             output_atten[1].GetTensorTypeAndShapeInfo().GetShape()[1]});
        }

        auto dim_hidden = output_atten[0].GetTensorTypeAndShapeInfo().GetShape()[1];
        auto xx = DataUtil::IndexAggr(output_atten[0].GetTensorMutableData<float>(),
                                      edge_index.Row<int64_t>(i_i), dim_hidden, n_edges, n_nodes, 1);
//        if (bVerbose) PrintVector("xx_aggr", xx.data(), std::vector<int64_t>{n_node, dim_hidden});

        auto xxx = DataUtil::Concat(nodeFeatures.Get<float>(), xx.data(), n_nodes,
                                    dim_obj_f, dim_hidden,
                                    dim_obj_f, dim_hidden);
        if (mDebug) ONNX::PrintVector("xxx_concat", xxx.data(), std::vector<int64_t>{static_cast<long>(n_nodes), dim_obj_f + dim_hidden});

        std::vector<float *> gcn_nn_inputs = {xxx.data()};
        std::vector<std::vector<int64_t>> gcn_nn_input_size = {{static_cast<long>(n_nodes), dim_obj_f + dim_hidden}};
        auto gcn_nn2_outputs = mEatGCN->ComputeGCN(PSLAM::EatGCN::OP_GCN_PROP, level, gcn_nn_inputs, gcn_nn_input_size);
        if (level + 1 < size_t(GetParams().at("n_layers").int_value())) {
            DataUtil::Relu(gcn_nn2_outputs[0].GetTensorMutableData<float>(), n_nodes * dim_obj_f);
            DataUtil::Relu(output_atten[1].GetTensorMutableData<float>(), n_edges * dim_rel_f);
        }

        /// Copy Node feature
        {
            size_t n=0;
            for(const auto & pair : selected_nodes) {
                auto& node_feature = pair.second->mFeatures[std::to_string(level+1)];
                if(!node_feature) node_feature.reset(new MemoryBlock(MemoryBlock::FLOAT, dim_obj_f));
                auto *feature = gcn_nn2_outputs[0].GetTensorMutableData<float>() + n * dim_obj_f;
                memcpy(node_feature->Get<float>(), feature, sizeof(float) * dim_obj_f);
                n++;
            }
        }


        /// Copy Edge feature
        for(size_t n=0;n<n_edges;++n) {
            auto& edge_feature = selected_edges.at(n)->mFeatures[std::to_string(level+1)];
            if(!edge_feature) edge_feature.reset(new MemoryBlock(MemoryBlock::FLOAT, dim_rel_f));
            auto *feature = output_atten[1].GetTensorMutableData<float>() + n * dim_rel_f;
            memcpy(edge_feature->Get<float>(), feature, sizeof(float) * dim_rel_f);
        }
    }
    CTOCK("[UpdateGCNFeature]5.compute");
    SCLOG(VERBOSE) << "Update GCN " << level << " feature done";
}


std::map<int,NodePtr> GraphPredictor::getNeighbor(const std::map<int, NodePtr> &vNodes, size_t min_pts) {
    /// Collect the neighbors of the selected nodes that need to be updated
    std::map<int,NodePtr> nodes;
    std::set<int> all_selected_nodes;
    std::vector<std::set<int>> level_selected_indices;
    {
        CTICK("[UpdateShapeFeature]2.findNeighbors");
        std::set<int> selected_nodes;
        for (const auto &n : vNodes)
            selected_nodes.insert(n.second->idx);
        level_selected_indices.push_back(selected_nodes);
        for (size_t l = 1; l < 2; ++l) {
            std::set<int> nn_selections;
            for (auto &s_n : level_selected_indices.at(l - 1)) {
//                    if (nodes.at(s_n)->last_class_predicted == "floor")continue;
                std::unique_lock<std::mutex> lock(mpGraph->mMutNode);
                if (mpGraph->nodes.find(s_n) == mpGraph->nodes.end()) continue;
                std::unique_lock<std::mutex> lock2(mpGraph->nodes.at(s_n)->mMutNN);
                for (auto &nn : mpGraph->nodes.at(s_n)->neighbors) {
                    if (mpGraph->nodes.find(nn) == mpGraph->nodes.end()) continue;
                    if (mpGraph->nodes.at(nn)->surfels.size() <= min_pts) continue;
                    if(all_selected_nodes.find(nn) != all_selected_nodes.end())continue;
                    nn_selections.insert(nn);
                    nodes[nn] = mpGraph->nodes.at(nn);
                    all_selected_nodes.insert(nn);
                }
            }
            level_selected_indices.push_back(nn_selections);
        }
        CTOCK("[UpdateShapeFeature]2.findNeighbors");
    }

    return nodes;
}

void GraphPredictor::UpdatePrediction(const std::map<int,NodePtr> &vNodes,
                                      const std::map<int,std::pair<size_t,size_t>> &sizeAndEdge) {
    CTICK_RESET("[GraphPredictor::UpdatePrediction]1.lock_updateNode");
    CTICK_RESET("[GraphPredictor::UpdatePrediction]2.lock_select_edge");
    CTICK_RESET("[GraphPredictor::UpdatePrediction]3.lock_update_edge");
    CTICK_RESET("[GraphPredictor::UpdatePrediction]4.lock_update_instances");
    SCLOG(VERBOSE) << "Prediction Updated start.";
    const auto &nodes = mpGraph->nodes;
    auto last_level = std::to_string(size_t(GetParams().at("n_layers").int_value()));

    /// Collect the neighbor of the selected nodes

    std::map<size_t, size_t> idx2seg;
    std::map<size_t, size_t> seg2idx;
    /// Filter out nodes without computed feature
    std::vector<NodePtr> selected_nodes;
    std::vector<std::pair<size_t,size_t>> filtered_sizeAndEdge;
    SCLOG(VERBOSE) << "filter input nodes.";
    {
        for(const auto &pair:vNodes) {
            const auto &node = pair.second;
            if(node->mFeatures.find(last_level) != node->mFeatures.end()) {
                selected_nodes.push_back(node);
                filtered_sizeAndEdge.push_back(sizeAndEdge.at(pair.first));
            }
        }
    }
    size_t n_nodes = selected_nodes.size();

    std::map<int,std::set<int>> has_same_part_nodes;
    if(1) /// Node
    {
        int64_t dim_obj_f = GetParams().at("dim_o_f").int_value();
        if (!selected_nodes.empty()) {
            MemoryBlock2D nodeFeatures(MemoryBlock::FLOAT,
                                       {selected_nodes.size(), static_cast<unsigned long>(dim_obj_f)});
            SCLOG(VERBOSE) << "collect node features.";
            for (size_t i = 0; i < n_nodes; ++i) {
                const auto &last_node_feature = selected_nodes.at(i)->mFeatures.at(last_level)->Get<float>();
                memcpy(nodeFeatures.Row<float>(i), last_node_feature, sizeof(float) * dim_obj_f);
            }

            if (mDebug) ONNX::PrintVector("nodeFeatures", nodeFeatures.Get<float>(), std::vector<size_t>{nodeFeatures.mDims.x, nodeFeatures.mDims.y});

            SCLOG(VERBOSE) << "predict.";
            std::vector<Ort::Value> node_features;
            node_features.push_back(ONNX::CreateTensor(mMemoryInfo.get(), nodeFeatures.Get<float>(),
                                                       {static_cast<long>(n_nodes),
                                                        static_cast<long>(dim_obj_f)}));
            auto objcls_prob = mEatGCN->Compute(PSLAM::EatGCN::OP_CLS_OBJ, node_features);
            size_t num_cls = objcls_prob[0].GetTensorTypeAndShapeInfo().GetShape()[1];
            assert(mLabels.size() == num_cls);
            SCLOG(VERBOSE) << "write back.";
            for (size_t i = 0; i < n_nodes; ++i) {
                const auto &node = selected_nodes.at(i);
                std::map<std::string, float> m;
                std::map<std::string, std::pair<size_t, size_t>> mm;
                float *data = objcls_prob[0].GetTensorMutableData<float>() + i * num_cls;
                for (size_t j = 0; j < num_cls; ++j) {
                    m[mLabels.at(j)] = expf(data[j]); // logsoftmax -> softmax
                    mm[mLabels.at(j)] = filtered_sizeAndEdge.at(i);
                }
                node->UpdatePrediction(m, mm, mConfigPslam->use_fusion);

                std::set<int> checked_nodes;
                auto has_same_nodes = has_same_part_nodes[node->idx];
                CTICK("[GraphPredictor::UpdatePrediction]1.lock_updateNode");
                {
                    std::unique_lock<std::mutex> lock(mpGraph->mMutNode);
                    find_nodes_has_same_part(nodes, node->idx, node->idx, has_same_nodes, checked_nodes);
                }
                CTOCK("[GraphPredictor::UpdatePrediction]1.lock_updateNode");
            }
        }
    }
    if(1) /// Edge
    {
        const auto &edges = mpGraph->edges;
        std::vector<EdgePtr> selected_edges;
        int64_t dim_rel_f = GetParams().at("dim_r_f").int_value();
        if(n_nodes==0){
            SCLOG(VERBOSE) << "got 0 nodes. return";
            return;
        }
        for(size_t i=0;i<n_nodes;++i) {
            idx2seg[i] = selected_nodes.at(i)->idx;
            seg2idx[selected_nodes.at(i)->idx] = i;
        }


        for (auto &selected_node : selected_nodes) {
            const auto &n = selected_node->idx;
            if (seg2idx.find(n) == seg2idx.end()) continue;
            CTICK("[GraphPredictor::UpdatePrediction]2.lock_select_edge");
            std::unique_lock<std::mutex> lock(mpGraph->mMutEdge);
            for (const auto &nn : selected_node->neighbors) {
                if (vNodes.find(nn) == vNodes.end()) continue;
                if (seg2idx.find(nn) == seg2idx.end()) continue;
                if (seg2idx.at(n) == seg2idx.at(nn)) continue;
                if (edges.find({n, nn}) == edges.end()) continue;
                if (edges.at({n, nn})->mFeatures.find(last_level) ==
                    edges.at({n, nn})->mFeatures.end())
                    continue;
                selected_edges.push_back(edges.at({n, nn}));
            }
            CTOCK("[GraphPredictor::UpdatePrediction]2.lock_select_edge");
        }

        MemoryBlock2D edgeFeatures(MemoryBlock::FLOAT,
                                   {selected_edges.size(), static_cast<unsigned long>(dim_rel_f)});
        for (size_t i = 0; i < selected_edges.size(); ++i) {
            const auto &last_node_feature = selected_edges.at(i)->mFeatures.at(last_level)->Get<float>();
            memcpy(edgeFeatures.Row<float>(i), last_node_feature, sizeof(float) * dim_rel_f);
        }

        if (mDebug) ONNX::PrintVector("edgeFeatures", edgeFeatures.Get<float>(), std::vector<size_t>{edgeFeatures.mDims.x, edgeFeatures.mDims.y});

        std::vector<Ort::Value> edge_features;
        edge_features.push_back(ONNX::CreateTensor(mMemoryInfo.get(), edgeFeatures.Get<float>(),
                                                   {static_cast<long>(selected_edges.size()),
                                                    static_cast<long>(dim_rel_f)}));
        auto relcls_prob = mEatGCN->Compute(PSLAM::EatGCN::OP_CLS_REL, edge_features);

        size_t num_rel = relcls_prob[0].GetTensorTypeAndShapeInfo().GetShape()[1];
        assert(num_rel == mRelationships.size());
        for(size_t i=0;i<selected_edges.size();++i){
            auto &edge = selected_edges.at(i);
            float *data = relcls_prob[0].GetTensorMutableData<float>() + i * num_rel;
            std::map<std::string, float> prop;
            for(size_t j=0;j<num_rel;++j) {
                prop[mRelationships.at(j)] = GetParams().at("multi_rel_outputs").bool_value()? data[j] : expf(data[j]); // logsoftmax->softmax
            }
            CTICK("[GraphPredictor::UpdatePrediction]3.lock_update_edge");
            edge->UpdatePrediction(prop,mConfigPslam->use_fusion);
            CTOCK("[GraphPredictor::UpdatePrediction]3.lock_update_edge");
        }
    }

    /// Update Instances
    if(1)
    {
        int idx_same_part = -1;
        for (auto &pair:mRelationships) {
            if (pair.second == Edge::Same()) {
                idx_same_part = int(pair.first);
                break;
            }
        }
        assert(idx_same_part >= 0);
        std::set<int> checked_nodes;
        CTICK("[GraphPredictor::UpdatePrediction]4.lock_update_instances");
        for (const auto &pair:selected_nodes) {
            if(has_same_part_nodes.find(pair->idx) == has_same_part_nodes.end()) continue;
            const auto& pre_same_nodes = has_same_part_nodes.at(pair->idx);
            std::set<int> found_nodes;
            std::unique_lock<std::mutex> lock(mpGraph->mMutNode);
            find_nodes_has_same_part(nodes, pair->idx, idx_same_part, found_nodes, checked_nodes);


            // check if there are nodes previously has the same part but removed later
            std::set<int> should_seperate;
            for(auto i : pre_same_nodes)
                if(found_nodes.find(i) == found_nodes.end()) should_seperate.insert(i);
            for(auto i : should_seperate) {
                nodes.at(i)->instance_idx = i;
            }


            if(found_nodes.empty()) continue;
            // find the smallest idx as their instance id
            found_nodes.insert(pair->idx);// add itself
            int instance_id = int(pair->idx);
            nodes.at(pair->idx)->instance_idx = instance_id; // set to itself, in case "same part" relationship is removed.
            for(auto& i : found_nodes)
                if(i < instance_id) instance_id = i;
            for(auto& i : found_nodes)
                nodes.at(i)->instance_idx = instance_id;


            SCLOG(DEBUG) << pair->idx << ": There are  " << found_nodes.size() << " have same part relationship with node " << pair->idx << ". The instance id is: " << instance_id;
            std::stringstream ss;
            for (auto s : found_nodes) {
                ss << s << " ";
            }
            SCLOG(DEBUG) << ss.str();
        }
        CTOCK("[GraphPredictor::UpdatePrediction]4.lock_update_instances");
    }
    SCLOG(VERBOSE) << "Prediction Updated end";
}

bool GraphPredictor::Pin() {
    if(mThread.valid()) {
        //wait until stop
        if( mThread.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            mThread.get();
            return true;
        }
    }
    return false;
}

void GraphPredictor::SetThread(bool option) {mbThread = option; }

bool GraphPredictor::Updated(){return mUpdated;}
void GraphPredictor::SetUpdate(bool option){mUpdated=option;}
const std::map<std::string, json11::Json>& GraphPredictor::GetParams(){
    return mEatGCN->mParams;
}

std::map<std::string,std::vector<std::pair<int,float>>> GraphPredictor::GetTimes() {
    return mTimes;
}
#endif