#include "graph.h"
#include "node.h"

#include <memory>
#include <regex>
#include <string>
#include <fstream>
#include <ORUtils/Logging.h>
#include "config.h"

#include <thread>
#ifdef COMPILE_WITH_JSON
#include <json11.hpp>
#endif

using namespace PSLAM;

Graph::Graph(const ConfigPSLAM *configPslam, bool useThread):mConfigPslam(configPslam),mbThread(useThread){
    if(mbThread)
        mPools = std::make_unique<tools::TaskThreadPool>(std::thread::hardware_concurrency());
}

int Graph::Add(const SurfelPtr &surfel) {
    const int index = surfel->GetLabel();
    NodePtr node;
    {
        std::unique_lock<std::mutex> lock(mMutNode);
        if(nodes.find(index) == nodes.end()) nodes.insert({index, std::make_shared<Node>(index)});
        node = nodes.at(index);
    }
    {
        std::unique_lock<std::mutex> lock(mMutThis);
        nodes_to_update.insert(index);
    }
    return node->Add(surfel);
}

void Graph::Merge(const int seg_idx_to, const int seg_idx_from) {
    try {
        NodePtr node_from, node_to; // Use copy to prevent thread error
        {
            std::unique_lock<std::mutex> lock(mMutNode);
            if (nodes.find(seg_idx_to) == nodes.end()) {
                SCLOG(VERBOSE) << "Cannot find node " << seg_idx_to << " to merge";
                return;
            } else
                node_to = nodes.at(seg_idx_to);
            if (nodes.find(seg_idx_from) == nodes.end()) {
                SCLOG(VERBOSE) << "Cannot find node " << seg_idx_from << " to merge";
                return;
            } else
                node_from = nodes.at(seg_idx_from);
            assert(node_from->idx == seg_idx_from);
            assert(node_to->idx == seg_idx_to);
        }
        RemoveNode(seg_idx_from);

        SCLOG(VERBOSE) << "Merge segment " << seg_idx_to << " and " << seg_idx_from;

        // add surfels from node_remove to node_to
        SCLOG(VERBOSE) << "remove surfels";
        {
            std::unordered_map<int, SurfelPtr> surfels;
            {
                std::unique_lock<std::mutex> lock(node_from->mMutSurfel);
                surfels = node_from->surfels;
            }

            {
                std::unique_lock<std::mutex> lock(mMutThis);//TODO: may not need this?
                for (const auto &pair : surfels){
                    const auto &surfel = pair.second;
                    surfel->SetLabel(seg_idx_to);
                    surfel->index_in_graph = node_to->Add(surfel);
                }
            }
            {
                std::unique_lock<std::mutex> lock(mMutThis);
                nodes_to_update.insert(seg_idx_to);
            }
        }

        // change edges
        std::unordered_set<EdgePtr> n_f_edges;
        {
            std::unique_lock<std::mutex> lock(node_from->mMutEdge);
            n_f_edges = node_from->edges;
        }

        SCLOG(VERBOSE) << "remove edges";
        for (const auto &edge: n_f_edges) {
            if (!edge) continue;
            auto pair = std::make_pair(edge->nodeFrom, edge->nodeTo);

            // check inward or outward
            int other_index;
            bool To;
            if (edge->nodeFrom == seg_idx_from) { // from seg_idx_from to others
                other_index = edge->nodeTo;
                To = true;
            } else if (edge->nodeTo == seg_idx_from) { // from others to seg_idx_from
                other_index = edge->nodeFrom;
                To = false;
            } else {
                SCLOG(WARNING) << "an node has an edge doesn't have the node as one of its support.";
                continue;
            }

            // if the other_index is seg_idx_to, ignore. (remove self edges)
            if (other_index == seg_idx_to) continue;

            // Add new edge
            if (To) {
                AddEdge(seg_idx_to, other_index, edge);
            } else {
                AddEdge(other_index, seg_idx_to, edge);
            }

            // remove the edge from global edge map
            RemoveEdge(pair);
        }

        SCLOG(VERBOSE) << "copy prediction";
        {
            {
                std::unique_lock<std::mutex> lock_from(node_from->mMutPred);
                node_to->UpdatePrediction(node_from->mClsProb, node_from->mSizeAndEdge,
                                          mConfigPslam->use_fusion);
            }
#ifdef COMPILE_WITH_GRAPHPRED
            std::unique_lock<std::mutex> lock_node(node_to->mMutNode);
            node_to->mFeatures = node_from->mFeatures;
#endif
        }
    }
    catch (std::out_of_range& e)
    {
        SCLOG(ERROR) << e.what();
    }
}

void Graph::Update(const SurfelPtr surfel,
                           const Eigen::Vector3f& pos,
                           const Eigen::Vector3f& normal) {
    SCLOG(WARNING) << "not implemented";
}

void Graph::CheckConnectivity(float margin){
    std::set<int> updated;
    for(size_t i=0;i<nodes_to_update.size();++i) {
        auto iter = nodes_to_update.begin();
        std::advance(iter,i);
        auto idx = *iter;
        if (idx == 0) continue; // skip 0

        auto& node = nodes.at(idx);
        if (node->bNeedCheckNeighbor)
            updated.insert(idx);
    }
    // update neighbors
    for(size_t i=0;i<updated.size();++i){
        for(size_t j=i+1;j<updated.size();++j){
            auto it_i = updated.begin();
            auto it_j = updated.begin();
            std::advance(it_i,i);
            std::advance(it_j,j);
            nodes.at(*it_i)->CheckConnectivity(nodes.at(*it_j).get(),margin,true);
        }
    }
}

int Graph::UpdateLabel(const SurfelPtr surfel,
                               const int label) {
    RemoveSurfelFromNode(surfel);
    surfel->SetLabel(label);
    return Add(surfel);;
}

bool HasSamePart(const Node &node){
    for(const auto &edge : node.edges){
        if(edge->GetLabel() == Edge::Same())
            return true;
    }
    return false;
}

int Graph::AddEdge(const EdgePtr &edge){
    try {
        auto pair = std::make_pair(edge->nodeFrom, edge->nodeTo);
        std::unique_lock<std::mutex> lock(mMutEdge);
        std::unique_lock<std::mutex> lock2(mMutNode);
        {
            if (edges.find(pair) == edges.end()) {
                // add new
                if (nodes.find(edge->nodeFrom) == nodes.end()) {
                    SCLOG(WARNING)
                    << "Failed to add an edge since the source node " << edge->nodeFrom << " was removed.";
                    return -1;
                }
                if (nodes.find(edge->nodeTo) == nodes.end()) {
                    SCLOG(WARNING) << "Failed to add an edge since the source node " << edge->nodeTo << " was removed.";
                    return -1;
                }

                if (edge->GetLabel() == "same part") {
                    if (nodes.at(edge->nodeFrom)->GetLabel() !=
                        nodes.at(edge->nodeTo)->GetLabel())
                        return -1;
                }

                edges.insert({{edge->nodeFrom, edge->nodeTo}, edge});
                nodes.at(edge->nodeFrom)->edges.insert(edge);
                nodes.at(edge->nodeTo)->edges.insert(edge);
                return 1;
            } else {
                auto oldedge = edges.at(pair);

                // merge prediction
                bool need_to_check = false;
                if (oldedge->GetLabel() == Edge::Same()) {
                    need_to_check = true;
                }
#ifdef COMPILE_WITH_GRAPHPRED
                oldedge->mFeatures = edge->mFeatures;
                oldedge->UpdatePrediction(edge->labelProp, mConfigPslam->use_fusion);
#endif
                /// Check the instance of the same part
                if (need_to_check && oldedge->GetLabel() != Edge::Same()) {
                    // there was a "same part" and now isn't. Need to fix the instance idx
                    if (!HasSamePart(*nodes.at(oldedge->nodeFrom)))
                        nodes.at(oldedge->nodeFrom)->instance_idx = oldedge->nodeFrom;
                    if (!HasSamePart(*nodes.at(oldedge->nodeTo)))
                        nodes.at(oldedge->nodeTo)->instance_idx = oldedge->nodeTo;
                }

                return 2;
            }
        }
    }catch (std::out_of_range& e)
    {
        SCLOG(ERROR) << e.what();
    }
    throw std::runtime_error("should not come here\n");
    return -1;
}

int Graph::AddEdge(int from, int to, const EdgePtr &edge){
    auto edge_ = std::make_shared<PSLAM::Edge>(*edge);
    edge_->nodeFrom=from;
    edge_->nodeTo=to;
    return AddEdge(edge_);
}

void Graph::Clear() {
    {
        std::unique_lock<std::mutex> lock(this->mMutNode);
        nodes.clear();
    }
    {
        std::unique_lock<std::mutex> lock(this->mMutEdge);
        edges.clear();
    }
    nodes_to_update.clear();
}

void Graph::RemoveSurfelFromNode(const SurfelPtr &surfel){
    const int index_in_graph = surfel->index_in_graph;
    const int oldLabel = surfel->GetLabel();

    // remove the surfel from the old node (old label) and add it to the new node (new label)
    if(nodes.find(oldLabel) != nodes.end()) {
        NodePtr node;
        {
            std::unique_lock<std::mutex> lock(mMutNode);
            node = nodes.at(oldLabel);
        }
        if (node->surfels.find(index_in_graph) == node->surfels.end()) {
            SCLOG(WARNING) << "Tried to update a surfel in node["<<oldLabel<<"] while the surfel does not exist in the node.";
            return;
        }

        // remove
        node->Remove(index_in_graph);

        // if current node has no more surfels, delete this node
        if (node->GetPointSize() == 0) {
            RemoveNode(oldLabel);
            // remove this node from the update list
            if(nodes_to_update.find(oldLabel) != nodes_to_update.end()) nodes_to_update.erase(oldLabel);
        } else {
            nodes_to_update.insert(oldLabel);
        }
    }
}

void Graph::RemoveNode(int idx) {
    try {
        NodePtr node;
        {
            std::unique_lock<std::mutex> lock(this->mMutNode);
            node = nodes.at(idx);
            nodes.erase(idx);

            std::unique_lock<std::mutex> lock2(this->mMutThis);
            nodes_to_update.erase(idx);
        }

        // remove edge related to this edge
        for (const auto &edge:node->edges) {
            auto pair = std::make_pair(edge->nodeFrom, edge->nodeTo);
            RemoveEdge(pair);
        }
    } catch (std::out_of_range &e) {
        SCLOG(ERROR) << e.what();
    }
}

void Graph::RemoveEdge(const std::pair<int,int> &pair){
    try {
        EdgePtr edge;
        {
            std::unique_lock<std::mutex> lock(this->mMutEdge);
            // erase edge from global edge map
            if (edges.find(pair) == edges.end()) {
                SCLOG(WARNING) << "An edge exist in a node but not in the global edge container.";
                return;
            }
            edge = edges.at(pair);
            edges.erase(pair);
        }

        // remove it from target node
        {
            std::unique_lock<std::mutex> lock(this->mMutNode);
            if (nodes.find(edge->nodeTo) != nodes.end())
                nodes.at(edge->nodeTo)->RemoveEdge(edge);

            if (nodes.find(edge->nodeFrom) != nodes.end())
                nodes.at(edge->nodeFrom)->RemoveEdge(edge);
        }
    }catch (std::out_of_range& e)
    {
        SCLOG(ERROR) << e.what();
    }
}

void Graph::UpdateSelectedNodes(const std::unordered_set<int> &filtered_selected_nodes, const size_t time,
                                const bool force) {
    SCLOG(VERBOSE) << "Update selected surfels.";
    for (auto node_idx : filtered_selected_nodes) {
        if (node_idx == 0 ) continue;
        NodePtr node;
        {
            std::unique_lock<std::mutex> lock(mMutNode);
            node = this->nodes.at(node_idx);
            if (node->surfels.size() < (size_t)mConfigPslam->filter_num_node) continue;
        }
        if(mbThread)
            mPools->runTask( std::bind(&PSLAM::Node::UpdateSelectedNode, node.get(), time, mConfigPslam->filter_num_node, mConfigPslam->n_pts, force) );
        else
            node->UpdateSelectedNode(time, mConfigPslam->filter_num_node, mConfigPslam->n_pts, force);
    }
    SCLOG(VERBOSE) << "selected surfels updated.";
}


void Graph::Wait(){
    SCLOG(VERBOSE) << "wait update properties to finish";
    if(mbThread) mPools->waitWorkComplete();
}