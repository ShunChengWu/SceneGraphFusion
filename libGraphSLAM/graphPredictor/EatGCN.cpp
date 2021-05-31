//
// Created by sc on 12/18/20.
//
#include "EatGCN.h"
#include <cassert>
using namespace PSLAM;

EatGCN::EatGCN(std::string path, Ort::MemoryInfo *memoryInfo, bool verbose) : PSLAM::ONNX::OnnxModelBase(std::move(path),
                                                                                                               memoryInfo),bVerbose(verbose) {
            mOp2Name[OP_ENC_OBJ] = "obj_pnetenc";
            mOp2Name[OP_ENC_REL] = "rel_pnetenc";
            mOp2Name[OP_CLS_OBJ] = "obj_pnetcls";
            mOp2Name[OP_CLS_REL] = "rel_pnetcls";
            mGOp2Name[OP_GCN_ATTEN] = "edgeatten_MultiHeadedEdgeAttention";
            mGOp2Name[OP_GCN_PROP] = "edgeatten_prop";
        }

std::vector<Ort::Value> EatGCN::ComputeGCN(GCN_OP_NAME name, size_t level, const std::vector<float *> &data,
                                   const std::vector<std::vector<int64_t>> &dims) {
    auto model_name = mGOp2Name.at(name);
    auto gcn_name = "gcn_" + std::to_string(level) + "_" + model_name;
    if (mModelParams.at(gcn_name)->inputName.size() != data.size())
        throw std::runtime_error("input size mismatch!");
    std::vector<Ort::Value> inputs;
    for (size_t i = 0; i < data.size(); ++i) {
        inputs.push_back(CreateTensor(data[i], dims[i]));
    }

    return Run(gcn_name, inputs);
}

std::vector<Ort::Value> EatGCN::ComputeGCN(GCN_OP_NAME name, size_t level, const std::vector<Ort::Value> &inputs) {
    auto model_name = mGOp2Name.at(name);
    auto gcn_name = "gcn_" + std::to_string(level) + "_" + model_name;
    if (mModelParams.at(gcn_name)->inputName.size() != inputs.size())
        throw std::runtime_error("input size mismatch!");
    return Run(gcn_name, inputs);
}

std::vector<Ort::Value>
EatGCN::Compute(OP_NAME name, const std::vector<float *> &data, const std::vector<std::vector<int64_t>> &dims) {
    auto model_name = mOp2Name.at(name);
    if (mModelParams.at(model_name)->inputName.size() != data.size())
        throw std::runtime_error("input size mismatch!");
    std::vector<Ort::Value> inputs;
    for (size_t i = 0; i < data.size(); ++i)
        inputs.push_back(CreateTensor(data[i], dims[i]));
    return Run(model_name, inputs);
}

std::vector<Ort::Value> EatGCN::Compute(OP_NAME name, std::vector<Ort::Value> &inputs) {
    auto model_name = mOp2Name.at(name);
    if (mModelParams.at(model_name)->inputName.size() != inputs.size())
        throw std::runtime_error("input size mismatch!");
    return Run(model_name, inputs);
}

std::tuple<MemoryBlock2D, MemoryBlock2D> EatGCN::Run(
        const MemoryBlock3D &input_nodes,
        const MemoryBlock2D &descriptor,
        const MemoryBlock2D &edge_index) {
    Ort::Value obj_f(nullptr), rel_f(nullptr);
    int64_t dim_obj_f = mParams.at("dim_o_f").int_value();
    int64_t dim_rel_f = mParams.at("dim_r_f").int_value();
    int64_t n_edge = edge_index.mDims.y;
    int64_t n_node = input_nodes.mDims.x;
    int64_t d_pts = input_nodes.mDims.y;
    int64_t n_pts = input_nodes.mDims.z;
    int64_t d_edge = descriptor.mDims.y;
    bool source_to_target = false;
    auto edge_descriptor = DataUtil::compute_edge_descriptor(descriptor, edge_index, source_to_target);
    if (bVerbose) {
        PrintVector("Edges", edge_index.Get<int64_t>(),
                    std::vector<unsigned long>{edge_index.mDims.x, edge_index.mDims.y});
        PrintVector("descriptor", descriptor.Get<float>(),
                    std::vector<unsigned long>{descriptor.mDims.x, descriptor.mDims.y});
        // print first node
        PrintVector("input_nodes", input_nodes.Get<float>(),
                    std::vector<size_t>{input_nodes.mDims.y, input_nodes.mDims.z});
        PrintVector("edge_descriptor", edge_descriptor.Get<float>(),
                    std::vector<unsigned long>{edge_descriptor.mDims.x, edge_descriptor.mDims.y});
        for (size_t i = 0; i < edge_descriptor.mDims.y; ++i) {
            std::cerr << edge_descriptor.at<float>(0, i) << " ";
        }
        std::cerr << "\n";
    }

    std::vector<float *> rel_inputs = {static_cast<float *>(edge_descriptor.data())};
    std::vector<std::vector<int64_t>> rel_input_sizes = {{n_edge, d_edge, 1}};
    auto out_enc_rel = Compute(PSLAM::EatGCN::OP_ENC_REL, rel_inputs, rel_input_sizes);
    if (bVerbose) {
        PrintTensorShape(out_enc_rel[0], "enc_rel");
        std::cerr << "\n";
        PrintVector("out_enc_rel", out_enc_rel[0].GetTensorMutableData<float>(),
                    std::vector<int64_t>{out_enc_rel[0].GetTensorTypeAndShapeInfo().GetShape()[0],
                                         out_enc_rel[0].GetTensorTypeAndShapeInfo().GetShape()[1]});
    }
//            {
    std::vector<float *> obj_inputs = {reinterpret_cast<float *>(input_nodes.data())};
    std::vector<std::vector<int64_t>> obj_input_sizes = {{n_node, d_pts, n_pts}};
    auto out_enc_obj = Compute(PSLAM::EatGCN::OP_ENC_OBJ, obj_inputs, obj_input_sizes);
    size_t dim_obj_feature = out_enc_obj[0].GetTensorTypeAndShapeInfo().GetShape()[1];

    if (bVerbose) {
        PrintVector("obj_feature", out_enc_obj[0].GetTensorMutableData<float>(),
                    std::vector<int64_t>{out_enc_obj[0].GetTensorTypeAndShapeInfo().GetShape()[0],
                                         out_enc_obj[0].GetTensorTypeAndShapeInfo().GetShape()[1]});
    }
    /// Concat descriptor to the object feature
    auto out = ConcatObjFeature(n_node, dim_obj_feature, out_enc_obj[0], descriptor);

    Ort::Value obj_feature = CreateTensor(out.Get<float>(),
                                          {static_cast<long>(out.mDims.x), static_cast<int64_t>(out.mDims.y)});
//            }
    obj_f = std::move(obj_feature);
    rel_f = std::move(out_enc_rel[0]);
    //NOTE: If the tensor if not created from onnx computation, if it is created by user. The address of the tensor
    // will be deleted if the owner is deleted. So do not move such self-created tensor into obj_f or rel_f in a loop
    // or use them in a different scope
    if (mParams.at("USE_GCN").bool_value())
        for (size_t l = 0; l < size_t(mParams.at("n_layers").int_value()); ++l) {
            size_t i_i = source_to_target ? 1 : 0;
            size_t i_j = source_to_target ? 0 : 1;

            std::vector<float> obj_f_i = DataUtil::Collect(obj_f.GetTensorMutableData<float>(),
                                                           edge_index.Row<int64_t>(i_i), dim_obj_f, n_edge);
            std::vector<float> obj_f_j = DataUtil::Collect(obj_f.GetTensorMutableData<float>(),
                                                           edge_index.Row<int64_t>(i_j), dim_obj_f, n_edge);
            if (bVerbose) {
                PrintVector("obj_f_i", obj_f_i.data(),
                            std::vector<int64_t>{n_edge, dim_obj_f});
                PrintVector("obj_f_j", obj_f_j.data(),
                            std::vector<int64_t>{n_edge, dim_obj_f});
            }

            std::vector<Ort::Value> inputs;
            inputs.emplace_back(CreateTensor(obj_f_i.data(), {n_edge, dim_obj_f}));
            inputs.emplace_back(std::move(rel_f));
            inputs.emplace_back(CreateTensor(obj_f_j.data(), {n_edge, dim_obj_f}));
            auto output_atten = ComputeGCN(PSLAM::EatGCN::OP_GCN_ATTEN, l, inputs);
            if (bVerbose) {
                PrintVector("output_atten", output_atten[0].GetTensorMutableData<float>(),
                            std::vector<int64_t>{output_atten[0].GetTensorTypeAndShapeInfo().GetShape()[0],
                                                 output_atten[0].GetTensorTypeAndShapeInfo().GetShape()[1]});
                PrintVector("output_edge", output_atten[1].GetTensorMutableData<float>(),
                            std::vector<int64_t>{output_atten[1].GetTensorTypeAndShapeInfo().GetShape()[0],
                                                 output_atten[1].GetTensorTypeAndShapeInfo().GetShape()[1]});
            }

            auto dim_hidden = output_atten[0].GetTensorTypeAndShapeInfo().GetShape()[1];
            auto xx = DataUtil::IndexAggr(output_atten[0].GetTensorMutableData<float>(),
                                          edge_index.Row<int64_t>(i_i), dim_hidden, n_edge, n_node, 1);
            if (bVerbose)
                PrintVector("xx_aggr", xx.data(), std::vector<int64_t>{n_node, dim_hidden});

            auto xxx = DataUtil::Concat(obj_f.GetTensorMutableData<float>(), xx.data(), n_node,
                                        dim_obj_f, dim_hidden,
                                        dim_obj_f, dim_hidden);
            if (bVerbose)
                PrintVector("xxx_concat", xxx.data(), std::vector<int64_t>{n_node, dim_obj_f + dim_hidden});

            std::vector<float *> gcn_nn_inputs = {xxx.data()};
            std::vector<std::vector<int64_t>> gcn_nn_input_size = {{n_node, dim_obj_f + dim_hidden}};
            auto gcn_nn2_outputs = ComputeGCN(PSLAM::EatGCN::OP_GCN_PROP, l, gcn_nn_inputs, gcn_nn_input_size);
            if (bVerbose)
                PrintVector("gcn_nn2_outputs", gcn_nn2_outputs[0].GetTensorMutableData<float>(),
                            std::vector<int64_t>{gcn_nn2_outputs[0].GetTensorTypeAndShapeInfo().GetShape()[0],
                                                 gcn_nn2_outputs[0].GetTensorTypeAndShapeInfo().GetShape()[1]});

            obj_f = std::move(gcn_nn2_outputs[0]);
            rel_f = std::move(output_atten[1]);
            if (l + 1 < size_t(mParams.at("n_layers").int_value())) {
                DataUtil::Relu(obj_f.GetTensorMutableData<float>(), n_node * dim_obj_f);
                DataUtil::Relu(rel_f.GetTensorMutableData<float>(), n_edge * dim_rel_f);
            }
        }

    std::vector<Ort::Value> node_features;
    std::vector<Ort::Value> edge_features;
    node_features.push_back(std::move(obj_f));
    edge_features.push_back(std::move(rel_f));
    auto objcls_prob = Compute(PSLAM::EatGCN::OP_CLS_OBJ, node_features);
    auto relcls_prob = Compute(PSLAM::EatGCN::OP_CLS_REL, edge_features);
    if (bVerbose) {
        PrintVector("objcls_prob", objcls_prob[0].GetTensorMutableData<float>(),
                    std::vector<int64_t>{objcls_prob[0].GetTensorTypeAndShapeInfo().GetShape()[0],
                                         objcls_prob[0].GetTensorTypeAndShapeInfo().GetShape()[1]});
        PrintVector("relcls_prob", relcls_prob[0].GetTensorMutableData<float>(),
                    std::vector<int64_t>{relcls_prob[0].GetTensorTypeAndShapeInfo().GetShape()[0],
                                         relcls_prob[0].GetTensorTypeAndShapeInfo().GetShape()[1]});
    }

    std::tuple<MemoryBlock2D, MemoryBlock2D> outputs;


    assert(objcls_prob[0].GetTensorTypeAndShapeInfo().GetElementCount() == (size_t)
                                                                                   mParams.at("num_classes").int_value() * n_node);
    assert(relcls_prob[0].GetTensorTypeAndShapeInfo().GetElementCount() == (size_t)
                                                                                   mParams.at("num_relationships").int_value() * n_edge);

    auto &output_obj =std::get<0>(outputs);
    auto &output_rel =std::get<1>(outputs);
    output_obj.Resize(MemoryBlock::FLOAT,{static_cast<unsigned long>(n_node),
                                          static_cast<unsigned long>(mParams.at("num_classes").int_value())});
    output_rel.Resize(MemoryBlock::FLOAT, {static_cast<unsigned long>(n_edge),
                                           static_cast<unsigned long>(mParams.at("num_relationships").int_value())});
    memcpy(output_obj.Get<float>(), objcls_prob[0].GetTensorMutableData<float>(), sizeof(float)*output_obj.size());
    memcpy(output_rel.Get<float>(), relcls_prob[0].GetTensorMutableData<float>(), sizeof(float)*output_rel.size());
    return outputs;
}

MemoryBlock2D EatGCN::ConcatObjFeature(size_t n_node, size_t dim_obj_feature, Ort::Value &obj_feature,
                                      const MemoryBlock2D &descriptor) {
    /// Concat descriptor to the object feature
    MemoryBlock2D out(MemoryBlock::DATA_TYPE::FLOAT, {static_cast<unsigned long>(n_node), dim_obj_feature + 8});
    MemoryBlock2D tmp_out_enc_obj(obj_feature.GetTensorMutableData<float>(), MemoryBlock::FLOAT,
                                  {static_cast<unsigned long>(n_node), dim_obj_feature});
    for (size_t n = 0; n < n_node; ++n) {
        MemoryBlock tmp (descriptor.Get<float>(n,3),descriptor.type(),8);
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

void EatGCN::InitModel(){
    pOrtEnv = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "test");
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(4);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    sessionOptions.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    for (auto &pair:mModelParams) {
        mSessions[pair.first] = std::make_unique<Ort::Session>(*pOrtEnv,
                                                               (pth_base + pair.second->model_path).c_str(),
                                                               sessionOptions);
    }
}
