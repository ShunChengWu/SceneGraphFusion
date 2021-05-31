//
// Created by sc on 10/9/20.
//

#ifndef GRAPHSLAM_ONNXMODELBASE_H
#define GRAPHSLAM_ONNXMODELBASE_H
#include "ParamLoader.h"
#include <onnxruntime_cxx_api.h>
namespace PSLAM {
    namespace ONNX {
        template<typename T>
        static size_t GetTotalSizeFromVector(const std::vector<T> &vec){
            size_t size = 1;
            for (auto &v:vec)
                size*=v;
            return size;
        }

        template<typename T>
        static inline Ort::Value CreateTensor(const Ort::MemoryInfo *memoryInfo, T* data, const std::vector<int64_t> &dims ) {
            return Ort::Value::CreateTensor<T>(*memoryInfo,
                                               data, GetTotalSizeFromVector(dims), dims.data(), dims.size());
        }
        static inline void PrintTensorShape(const std::string &name, Ort::Value &t){
            {
                std::stringstream ss;
                for(auto& v : t.GetTensorTypeAndShapeInfo().GetShape())
                    ss << v << ", ";
                printf("%s shape: %s\n", name.c_str(),ss.str().c_str());
            }
        };

        template<typename T, typename T2>
        static void PrintVector(const std::string &name, const T * data, const std::vector<T2> &dims){
            assert(dims.size()==2);
            std::stringstream ss;
            ss.precision(3);
            ss << "==" << name << "==\n";
            size_t elem_to_show_per_row = 3;
            for(unsigned int i=0;i<dims.at(0);++i){
                if(dims.at(0) > 10) {
                    if(i==3) {
                        ss << "...\n";
                        continue;
                    }
                    if(i>3 && i < dims.at(0)-3) continue;
                }
                if(dims.at(1)>7){
                    for(size_t j=0;j<elem_to_show_per_row;++j){
                        ss <<data[i*dims.at(1)+j] << " ";
                    }
                    ss << "... ";
                    for(size_t j=0;j<elem_to_show_per_row;++j){
                        ss << data[(i+1)*dims.at(1)-elem_to_show_per_row+j] << " ";
                    }
                    ss << std::endl;
                } else {
                    for(size_t j=0;j<size_t(dims.at(1));++j){
                        ss << data[i*dims.at(1)+j] << " ";
                    }
                    ss << std::endl;
                }
            }
            printf("%s\n",ss.str().c_str());
        }

        class OnnxModelBase: public PSLAM::ParamLoader {
        public:
            OnnxModelBase(std::string path, Ort::MemoryInfo *memoryInfo):ParamLoader(std::move(path)), mMemoryInfo(memoryInfo){
                InitModel();
            }
        protected:
            const Ort::MemoryInfo * mMemoryInfo;
            std::unique_ptr<Ort::Env> pOrtEnv;
            std::map<std::string,std::unique_ptr<Ort::Session>> mSessions;

            /**
             * Create Ort model by loop over the mModelParams.
             */
            virtual void InitModel(){
                pOrtEnv = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "test");
                Ort::SessionOptions sessionOptions;
                sessionOptions.SetIntraOpNumThreads(1);
                sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
                sessionOptions.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
                for(auto &pair:mModelParams) {
                    mSessions[pair.first] = std::make_unique< Ort::Session>(*pOrtEnv, (pth_base + pair.second->model_path).c_str(), sessionOptions);
                }
            }

            template<typename T>
            size_t GetTotalSizeFromVector(const std::vector<T> &vec){
                size_t size = 1;
                for (auto &v:vec)
                    size*=v;
                return size;
            }

            inline std::vector<Ort::Value> Run(const std::string &name, const std::vector<Ort::Value> &input_tensor){
                return mSessions.at(name)->Run(Ort::RunOptions{nullptr},
                                               mModelParams.at(name)->inputName.data(), input_tensor.data(), mModelParams.at(name)->inputName.size(),
                                               mModelParams.at(name)->outputName.data(), mModelParams.at(name)->outputName.size());
            }

            template<typename T>
            inline Ort::Value CreateTensor(T* data, const std::vector<int64_t> &dims ) {
                return Ort::Value::CreateTensor<T>(*mMemoryInfo,
                                                   data, GetTotalSizeFromVector(dims), dims.data(), dims.size());
            }

            static void PrintTensorShape(Ort::Value &t, const std::string &name){
                {
                    std::stringstream ss;
                    for(auto& v : t.GetTensorTypeAndShapeInfo().GetShape())
                        ss << v << ", ";
                    std::cerr << name << " shape: " << ss.str();
                }
            };
            template<typename T, typename T2>
            static void PrintVector(const std::string &name, const T * data, const std::vector<T2> &dims){
                assert(dims.size()==2);
                std::cerr << "=="<<name<<"==\n";
                size_t elem_to_show_per_row = 3;
                for(size_t i=0;i<size_t(dims.at(0));++i){
                    if(dims.at(0) > 10) {
                        if(i==3) {
                            std::cerr << "...\n";
                            continue;
                        }
                        if(i>3 && i < (size_t)dims.at(0)-3) continue;
                    }
                    if(dims.at(1)>7){
                        for(size_t j=0;j<elem_to_show_per_row;++j){
                            std::cerr << data[i*dims.at(1)+j] << " ";
                        }
                        std::cerr << "... ";
                        for(size_t j=0;j<elem_to_show_per_row;++j){
                            std::cerr << data[(i+1)*dims.at(1)-elem_to_show_per_row+j] << " ";
                        }
                        std::cerr << std::endl;
                    } else {
                        for(size_t j=0;j<size_t(dims.at(1));++j){
                            std::cerr << data[i*dims.at(1)+j] << " ";
                        }
                        std::cerr << std::endl;
                    }
                }
            }
        };
    }
}


#endif //GRAPHSLAM_ONNXMODELBASE_H
