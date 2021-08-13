#include "node.h"

// #include "inseg_lib/surfel.h"
#include <ORUtils/Logging.h>
#include <random>
#include <algorithm>
using namespace PSLAM;

Node::Node(int label):
idx(label), instance_idx(0), mDebug(false), time_stamp(0),lastUpdatePropertySize(0),
mbNeedUpdateNodeFeature(false),mIdxCounter(0) {
    last_class_predicted = Unknown();
    instance_idx=idx;
    centroid.setZero();
    pos_sum.setZero();
    bbox_max.setZero();
    bbox_min.setZero();
}

int Node::Add(const SurfelPtr &surfel) {
    assert(surfel->GetLabel() == this->idx);
    /*Find Idx*/
    int idx_;
    {
        Lock lock(mMutNode);
        if(!mqFreeIndices.empty()) {
            idx_ = mqFreeIndices.front();
            mqFreeIndices.pop();
        } else {
            idx_ = mIdxCounter++;
        }
    }

    size_t size;
    {
        Lock lock(mMutSurfel);
        surfels.insert( {idx_, surfel} );
        size = surfels.size();
    }

    /*Update Properties*/
    {
        Lock lock(mMutNode);
        pos_sum += surfel->pos;
        centroid = pos_sum / size;
        if(size == 1){
            bbox_min=bbox_max=surfel->pos;
        } else
            for(size_t j=0;j<3;++j) {
                if (surfel->pos[j] > bbox_max[j]) {
                    bNeedCheckNeighbor=true;
                    bbox_max[j] = surfel->pos[j];
                }
                if (surfel->pos[j] < bbox_min[j]) {
                    bNeedCheckNeighbor=true;
                    bbox_min[j] = surfel->pos[j];
                }
            }
    }
    return idx_;
}

void Node::Remove(const int index_in_graph_old) {
    size_t size;
    SurfelPtr surfel;
    {
        Lock lock_surfel(mMutSurfel);
        size = surfels.size();
        if(surfels.find(index_in_graph_old) == surfels.end())
            throw std::runtime_error("Trying to remove surfel but it doesn't exist.");
        assert(surfels.at(index_in_graph_old)->label == this->idx);
        surfel = surfels.at(index_in_graph_old);
        surfels.erase(index_in_graph_old);
    }

    Lock lock_node(mMutNode);
    pos_sum -= surfel->pos;
    centroid = pos_sum / size;
    bool need_recal_bbox = surfel->pos==bbox_min || surfel->pos ==bbox_max;

    mqFreeIndices.emplace(index_in_graph_old);
    if(need_recal_bbox) {
        Lock lock_surfel(mMutSurfel);
        for (size_t i=0;i<size;++i){
            auto surfel_ = surfels.at(i);
            if(i==0){
                bbox_max = bbox_min = surfel_->pos;
                continue;
            }
            for (size_t j = 0; j < 3; ++j) {
                if (surfel_->pos[j] > bbox_max[j]) bbox_max[j] = surfel_->pos[j];
                if (surfel_->pos[j] < bbox_min[j]) bbox_min[j] = surfel_->pos[j];
            }
        }
        bNeedCheckNeighbor = true;
    }
}

void Node::Update(const int label,
                   const int index,
                   const Eigen::Vector3f& pos,
                   const Eigen::Vector3f& normal,
                   const std::array<unsigned char, 3>& color) {
    throw std::runtime_error("not implemented.");
}

void Node::RemoveEdge(const EdgePtr &edge){
    Lock lock(mMutEdge);
    if (edges.find(edge) == edges.end()) return;
    edges.erase(edge);
}

void Node::UpdatePrediction(const std::map<std::string, float> &pd, const std::map<std::string,std::pair<size_t, size_t>> &sizeAndEdge, bool fusion) {
    Lock lock(mMutPred);
    for(const auto &pair:pd) {
        auto name = pair.first;
        auto value = pair.second;

        if(mClsProb.find(name) == mClsProb.end()) {
            mClsProb[name] = value;
            if (sizeAndEdge.find(name) == sizeAndEdge.end())
                SCLOG(ERROR) << "cannot find " << name << " in sizeAndEdge";
            mSizeAndEdge[name] = sizeAndEdge.at(name);
            mClsWeight[name] = 1;
        }
        else {
            if(fusion) {
                static const float max_w = 100;
                const static float new_w = 1;
                const auto &old_value = mClsProb.at(name);
                const auto &old_w = mClsWeight.at(name);
                mClsProb.at(name) = (old_value * old_w + value * new_w) / (old_w + new_w);
                mClsWeight.at(name) = std::min(max_w, mClsWeight.at(name) + new_w);
            } else {
                // simply over-write
                mClsProb[name] = value;
            }
        }
    }
    // find the newest with maximum prob
    if(mClsProb.empty()) return;
    {
        float max_v=mClsProb.begin()->second;
        std::string max_label = GetLabel();
        for(const auto& pair:mClsProb){
            if(pair.second>=max_v){
                max_v = pair.second;
                max_label = pair.first;
            }
        }
        Lock lock_pd (mMutPDLabel);
        last_class_predicted = max_label;
    }
}

bool Node::CheckConnectivity(Node *nodeP, float margin, bool modify){
    int idx_s = this->idx;
    int idx_t = nodeP->idx;
    if(idx_s == idx_t) return true;
    bool has_out = false;
    {
        Lock lock_this(this->mMutNode);
        for(size_t m=0;m<3;++m){
            if (bbox_min[m]-margin > nodeP->bbox_max[m]+margin) has_out = true;
            if (nodeP->bbox_min[m]-margin > bbox_max[m]+margin) has_out = true;
        }
        if(modify) this->bNeedCheckNeighbor = nodeP->bNeedCheckNeighbor = false;
    }
    if(!modify) return !has_out;

    Lock lock(mMutNN);
    if (has_out) {
        if(neighbors.find(idx_t) != neighbors.end()) neighbors.erase(idx_t);
        if(nodeP->neighbors.find(idx_s) != nodeP->neighbors.end())
            nodeP->neighbors.erase(idx_s);
    } else {
        neighbors.insert(idx_t);
        nodeP->neighbors.insert(idx_s);
    }
    return !has_out;
}

int Node::GetPointSize() {
    Lock lock (mMutSurfel);
    return surfels.size();
}

void Node::UpdateSelectedNode(const size_t time, const size_t filter_size, const size_t num_pts, bool force) {
    if(mbNeedUpdateNodeFeature) return;
    const auto& last_size = lastUpdatePropertySize; // this will be updated in UpdatePrediction
    size_t new_size;
    {
        Lock lock (mMutSurfel);
        new_size = surfels.size();
    }
    static float threshold_size = 0.1;
    static float threshold_time = 60;
    const float changes = abs(float(new_size)-float(last_size))/ float(new_size);
    bool should_update = false;
    should_update |= changes >= threshold_size;
    should_update |= (time - time_stamp) > threshold_time;
    if(force) should_update = true;
    if( !should_update) return;

    Lock lock(mMutSelected);
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator

    selected_surfels.clear();
    selected_surfels.reserve(num_pts);

    if (this->surfels.size() < filter_size) {
        return;
    } else {
        size_t valid_pts = 0;
        if (!mDebug) {
            std::vector<size_t> indices(this->surfels.size(), 0);
            std::iota(indices.begin(), indices.end(), 0);
            std::shuffle(indices.begin(), indices.end(), gen);
            for (const auto &i_idx : indices) {
                if (surfels.find(i_idx) == surfels.end()) continue;
                if (!surfels.at(i_idx)->is_stable || !surfels.at(i_idx)->is_valid) continue;
                valid_pts++;
                if (selected_surfels.size() < num_pts)
                    this->selected_surfels.emplace_back(surfels.at(i_idx));
                if (valid_pts >= filter_size && selected_surfels.size() == num_pts) break;
            }
        } else {
            for (const auto &surfel:surfels) {//TODO: remove me
                if (!surfel.second->is_stable || !surfel.second->is_valid) continue;
                valid_pts++;
                if (selected_surfels.size() < num_pts)
                    this->selected_surfels.emplace_back(surfel.second);
                if (valid_pts >= filter_size && selected_surfels.size() == num_pts) break;
            }
        }
        if(valid_pts < filter_size || selected_surfels.size() != num_pts){
            selected_surfels.clear();
            return;
        }
    }
    lock.unlock();

    if (mDebug)
        for(size_t i=0;i<1;++i) {//TODO: remove me
            Eigen::Vector3f color;
            color.x() = float(selected_surfels.at(i)->color[2])/255*2.f-1;
            color.y() = float(selected_surfels.at(i)->color[1])/255*2.f-1;
            color.z() = float(selected_surfels.at(i)->color[0])/255*2.f-1;
            SCLOG(DEBUG) << selected_surfels.at(i)->pos.transpose() << " " << color.transpose() << " " << selected_surfels.at(i)->normal.transpose();
        }

    mCentroid.setZero();
    mStd.setZero();
    for(size_t i=0;i<selected_surfels.size();++i){
        const auto &surfel = selected_surfels.at(i);
        mCentroid+=surfel->pos;

        if(i==0){
            mBBox_max=mBBox_min=surfel->pos;
        } else {
            for(size_t j=0;j<3;++j) {
                if (surfel->pos[j] > mBBox_max[j]) {
                    mBBox_max[j] = surfel->pos[j];
                }
                if (surfel->pos[j] < mBBox_min[j]) {
                    mBBox_min[j] = surfel->pos[j];
                }
            }
        }
    }
    mCentroid/=selected_surfels.size();

    for(auto & selected_surfel : selected_surfels) {
        for (size_t d = 0; d < 3; ++d)
            mStd[d] += pow(selected_surfel->pos[d] - mCentroid[d], 2);
    }
    for (size_t d = 0; d < 3; ++d)
        mStd[d] = std::sqrt(mStd[d] / (selected_surfels.size() - 1.f) );

    //TODO: remove me
    if (mDebug)
    {
        float volume = 1;
        auto dim = (mBBox_max-mBBox_min)*1e-3;
        for (size_t d = 0; d < 3; ++d) volume *= dim[d];
        float length = *std::max_element(dim.begin(), dim.end());
        size_t valid_surfels = 0;
        for(const auto& surfel:surfels)
            if(surfel.second->is_valid && surfel.second->is_stable) valid_surfels++;
    }

    mbNeedUpdateNodeFeature=true;
    lastUpdatePropertySize = new_size;
    time_stamp = time;
}

const std::string Node::GetLabel() const {
    Lock lock(mMutPDLabel);
    return last_class_predicted;
}
