//
// Created by sc on 10/6/20.
//

#ifndef GRAPHSLAM_PARAMLOADER_H
#define GRAPHSLAM_PARAMLOADER_H
#include <string>
#include <json11.hpp>
#include <cstring>
#include <ORUtils/JsonUtil.h>
namespace PSLAM {
    struct ONNX_MODEL_PARAMS {
        std::string model_path;
        std::vector<std::string> inputName_;// this is to store the memory location. the onnx needs const char* format. a direct store of const char * in a vector is problematic.
        std::vector<std::string> outputName_;
        std::vector<const char*> inputName;
        std::vector<const char*> outputName;
        ONNX_MODEL_PARAMS(){}
        ONNX_MODEL_PARAMS(std::string name, std::vector<std::string> in, std::vector<std::string> out){
            Set(std::move(name),std::move(in),std::move(out));
        }
        void Set(std::string name, std::vector<std::string> in, std::vector<std::string> out){
            model_path = std::move(name);
            inputName_ = std::move(in);
            outputName_ = std::move(out);
            for(const auto& s : inputName_) {
                inputName.push_back(&s[0]);
            }

            for(const auto& s : outputName_) {
                char *buff = new char[s.size() + 1];          // allocate memory
                std::strncpy(buff, s.c_str(), s.size() + 1); // copy data to memory
                outputName.push_back(buff);
            }
        }
        ~ONNX_MODEL_PARAMS(){}
    };

    class ParamLoader {
    public:
        std::string pth_base;
        ParamLoader(std::string path, std::string name_args = "args.json"):pth_base(path){
            const std::string pth_onnx = pth_base + name_args;
            auto json = ORUtils::JsonUtils::LoadJson(pth_onnx);
            LoadArgs(mParams, mModelParams, json);

            /// Read labels
            {
                std::fstream file(path+"/"+"classes.txt", std::ios::in);
                if(!file.is_open())SCLOG(ERROR) << "Cannot open class file at path " << path+"/"+"classes.txt";
                std::string line;
                std::vector<std::string> labels;
                while (std::getline(file, line)) labels.push_back(line);

                SCLOG(VERBOSE) << "Labels";
                for(size_t i=0;i<labels.size();++i) {
                    mLabels.insert({i, labels[i]});
                    SCLOG(VERBOSE) << i << ": " << labels[i];
                }
            }
            {
                std::fstream file(path+"/"+"relationships.txt", std::ios::in);
                if(!file.is_open())SCLOG(ERROR) << "Cannot open relationship file at path " << path+"/"+"relationships.txt";
                std::string line;
                std::vector<std::string> labels;
                while (std::getline(file, line)) labels.push_back(line);
                SCLOG(VERBOSE) << "Relationships";
                for(size_t i=0;i<labels.size();++i) {
                    mRelationships.insert({i,labels[i]});
                    SCLOG(VERBOSE) << i << ": " << labels[i];
                }
            }
        }
        std::map<std::string, std::unique_ptr<ONNX_MODEL_PARAMS>> mModelParams;
        std::map<std::string, json11::Json> mParams;
        std::map<size_t,std::string> mLabels, mRelationships;
    private:
        static void LoadArgs(std::map<std::string, json11::Json> &otherParams,
                             std::map<std::string, std::unique_ptr<ONNX_MODEL_PARAMS>> &modelParams, const json11::Json &json, bool verbose=false) {
            for (auto &object:json.object_items()){
                const auto &name = object.first;
                if (name.find("model_") != std::string::npos && object.second.is_object()) {
                    auto t_name = name.substr(6, name.length());
                    if (verbose) printf("name: %s, items: %zu\n", t_name.c_str(), object.second.object_items().size());
                    auto path = object.second.object_items().at("path").string_value();
                    std::vector<std::string> input_names, output_names;
                    for (const auto &name_ : object.second.object_items().at("input").array_items())
                        input_names.push_back(name_.string_value());
                    for (const auto &name_ : object.second.object_items().at("output").array_items())
                        output_names.push_back(name_.string_value());
                    modelParams[t_name].reset(new ONNX_MODEL_PARAMS(path, input_names, output_names));

                    if (verbose)
                        for (const auto &m_obj : object.second.object_items()) {
                            std::cout << m_obj.first << ": ";
                            if (m_obj.second.is_string())
                                std::cout << m_obj.second.string_value() << " ";
                            if (m_obj.second.is_array()) {
                                for (auto &v : m_obj.second.array_items()) {
                                    std::cout << v.string_value() << " ";
                                }
                            }
                            std::cout << "\n";
                        }
                } else if (object.second.is_bool() || object.second.is_string() || object.second.is_number()) {
                    otherParams[name] = object.second;
                } else if (object.second.is_object())
                    LoadArgs(otherParams, modelParams, object.second);
            }
        }
    };
}

#endif //GRAPHSLAM_PARAMLOADER_H
