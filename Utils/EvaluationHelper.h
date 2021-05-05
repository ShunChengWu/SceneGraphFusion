#pragma once

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <numeric>
#include <ORUtils/LogUtil.h>
//#include <CxxTools/PathTool.hpp>

namespace SCSLAM {
    namespace EVALUATION {
        class Logging {
        public:
            Logging()=default;
            ~Logging()= default;
            // Suppress the default copy constructor and assignment operator
            Logging(const Logging&);
            Logging& operator=(const Logging&);


            static std::string GenerateOutputPathAndName(std::string pth_out) {
                return pth_out;
            }

            static void saveTimeInfo(std::map<std::string, std::vector<double>> &times_){
                /// Save Time Information
                for (const std::pair<std::string, double> &time : getWatch.getTimings()) {
                    if(!getWatch.updated(time.first)) continue;
                    getWatch.getUpdateStats()[time.first] = false;

                    if(times_.find(time.first) == times_.end()) {
                        times_[time.first].reserve(1e5); // assume will have this amount of images
                        times_[time.first].push_back(time.second);
                    } else {
                        times_[time.first].push_back(time.second);
                    }
                }
            }

            static bool printResultToFile(const std::string &pth_out, const std::string &filename = "times.csv") {
                std::map<std::string, std::vector<double>> times_;
                for (auto &pair : getWatch.getCTimings()) {
                    const auto &name = pair.first;
                    if(pair.second.second==0) continue;

                    const auto &value= pair.second.first / float(pair.second.second);
                    if(!getWatch.updated(name)) continue;
                    getWatch.getUpdateStats()[name] = false;

                    if(times_.find(name) == times_.end()) {
                        times_[name].reserve(1e5); // assume will have this amount of images
                        times_[name].push_back(value);
                    } else {
                        times_[name].push_back(value);
                    }
                }
                return printResultToFile(times_,pth_out,filename);
            }

            static bool printResultToFile(const std::map<std::string, std::vector<double>> & times_, const std::string &pth_out, const std::string &filename = "times.csv") {
//                checkInputFolder(pth_out);
                const std::string outputFileName = pth_out + "/" + filename;
                std::fstream logFile_(outputFileName, std::ios::out);
                if(!logFile_.is_open()) {
                    SCLOG(ERROR) <<"Unable to open file at " << outputFileName << "\n";
                    return false;
                }

                if(!logFile_.is_open()) return false;
                logFile_ << "Name;Mean;Variance;Standard Deviation;Number of Measurements" << ";\n";
                double mean, var, stdvar;
                for (auto &time : times_) {
                    mean = var = stdvar = 0;
                    if (time.second.size() == 1) {
                        mean = time.second[0];
                        var = stdvar = 0;
                    } else {
                        mean = std::accumulate(time.second.begin(), time.second.end(), 0.f) / time.second.size();
                        for(size_t i=0;i<time.second.size();++i){
                            auto d = mean - time.second[i];
                            var += d*d;
                        }
                        var /= time.second.size();
                        stdvar = std::sqrt(var);
                    }

                    logFile_ << time.first << ";" << std::to_string(mean) << ";" << std::to_string(var) << ";" << std::to_string(stdvar)
                       << ";" << std::to_string(time.second.size()) << "\n";
                }

//                for(auto &time:times_){
//                    if(time.first == "[SLAMWrapper][processOnce]3.slam_->ProcessFrame" ||
//                            time.first == "[SLAM][ProcessFrame]4.Mapping") {
//                        logFile_ << time.first<<"\n";
//                        for(size_t i=0;i<time.second.size();++i){
//                            logFile_ << i << ";" <<time.second[i] << "\n";
//                        }
//                    }
//
//                }

                logFile_.close();
                SCLOG(INFO) << "Runtime saved to " << outputFileName;
                return true;
            }

        private:
//            static bool checkInputFolder(const std::string &pth_out){
//                tools::PathTool pathTool;
//                if(!pathTool.isFolder(pth_out)) {
//                    SCLOG(ERROR) << "expect a folder. Given was not.\n";
//                    return false;
//                }
//                pathTool.check_and_create_folder(pth_out);
//                return true;
//            }
        };
    }
}