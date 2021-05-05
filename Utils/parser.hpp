//
//  Parser.hpp
//
//  Created by Shun-Cheng Wu on 07/11/2018.
//

#ifndef Parser_hpp
#define Parser_hpp

#include <stdio.h>
#include <iostream>
#include <map>
#include <utility>
#include <vector>
#include <sstream>
#include <ORUtils/Logging.h>

#define pkgname(var) pkgVar(#var, var)
#define pkgcname(name,var) pkgVar(name,var)
template <typename T>
std::pair<std::string, T*> pkgVar(std::string name, T* var){
    if(name.find_first_of("&", 0, 1) != std::string::npos) {
        name.substr(1);
        name.erase(name.begin());
    }
    return std::make_pair(name, var);
}
namespace tools {
    class ParserException : public std::exception {
    public:
        explicit ParserException(std::string exception):msg(std::move(exception)){}
        const char* what() const noexcept override {
            return msg.c_str();
        }
    private:
        std::string msg;
        std::ostringstream log;
    };

    class Parser{
        std::map<std::string, std::vector<std::string>> argvs;
        bool hasHelp_;
        struct Command {
            std::string explination;
            std::string value;
            size_t size;
            bool required;
            bool handled;
        };
        std::map<std::string, Command> vRegisterd; // name : helper message
        std::vector<std::string> vOrder;
    public:
        Parser(int argc, char** argv){
            hasHelp_ = false;
            std::string current_parse;
            for (int i=1; i< argc; ++i) {
                std::string str = argv[i];
                if (str.compare(0, 2, "--") == 0){
                    current_parse = str.substr(2, str.size());
                    argvs[current_parse].resize(0);//if size 0, it is an option
                } else {
                    if (!current_parse.empty())
                        argvs[current_parse].push_back(argv[i]);

                }
            }
        };

        template <typename T>
        void addOption(std::pair<std::string, T*> pkg, std::string explaination = "", bool Required = false){
            const std::string& name = pkg.first;
            T* var = pkg.second;
            Command command;
            command.explination = std::move(explaination);
            command.required = Required;
            command.handled = false;
            command.size = 1;

            if(hasHelp_ || find_switch("h") || find_switch("help")){
                hasHelp_ = true;
            } else {
                if(find_parse(name)){
                    process(name, var, command.size);
                    command.handled = true;
                } else if (find_switch(name)) {
                    command.handled = changeSwitchValue(var);
                }
            }
            std::stringstream stn;
            stn << *var;
            command.value = stn.str();
            vRegisterd[name] = command;
            vOrder.push_back(name);
        }
        template <typename T>
        void addOption(std::pair<std::string, T*> pkg, size_t size, std::string explaination = "", bool Required = false){
            const std::string& name = pkg.first;
            T* var = pkg.second;
            Command command;
            command.explination = std::move(explaination);
            command.required = Required;
            command.handled = false;
            command.size = size;

            if(hasHelp_ || find_switch("h") || find_switch("help")){
                hasHelp_ = true;
            } else {
                if(find_parse(name)){
                    process(name, var, command.size);
                    command.handled = true;
                } else if (find_switch(name)) {
                    command.handled = changeSwitchValue(var);
                }
            }
            std::stringstream stn;
            stn << *var;
            command.value = stn.str();
            vRegisterd[name] = command;
            vOrder.push_back(name);
        }

        void addSwitch(const std::pair<std::string, bool*>& pkg, std::string explaination = "", bool Requied = false){
            const std::string& name = pkg.first;
            bool* var = pkg.second;
            Command command;
            command.explination = std::move(explaination);
            command.required = Requied;
            command.handled = false;

            if(hasHelp_ || find_switch("h") || find_switch("help")){
                hasHelp_ = true;
            } else {
                if(find_switch(name)) {
                    *var = !(*var);
                    command.handled = true;
                }
            }
            std::stringstream stn;
            stn << *var;
            command.value = stn.str();
            vRegisterd[name] = command;
            vOrder.push_back(name);
        }

        /// 2: has help 1: good, 0: warning, -1: error
        int showMsg(bool verbose=true){
            int maxLengh = 0;
            for(auto& name:vOrder){
                if(name.size()>(size_t)maxLengh) maxLengh = name.size();
            }
            if(find_switch("h") || find_switch("help")){
                for (auto& name : vOrder) {
                    auto& command = vRegisterd[name];
                    if(command.required)
                        printf("--%-*s \t(REQUIRED) %s (default: %s)\n", maxLengh, name.c_str(), command.explination.c_str(), command.value.c_str());
                    else
                        printf("--%-*s \t%s (default: %s)\n", maxLengh, name.c_str(), command.explination.c_str(), command.value.c_str());
                }
                return 2;
            }
            if(hasNotHandled(true)) return -1;
            if(hasNotRegistered(verbose))return 0;
            if(verbose)
            {
                for (auto& name : vOrder) {
                    auto& command = vRegisterd[name];
                    printf("--%-*s \t%s\n", maxLengh, name.c_str(), command.value.c_str());
                }
            }
            return 1;
        }


        template <typename T>
        int outputLog(T& log){
            size_t maxLengh = 0;
            char text[200];
            for(auto& name:vOrder){
                if(name.size()>maxLengh) maxLengh = name.size();
            }
            for (auto& name : vOrder) {
                auto& command = vRegisterd[name];
                sprintf(text,"--%-*s \t%s\n", maxLengh, name.c_str(), command.value.c_str());
                log << text;
            }
            return 1;
        }

        bool hasHelp(){return hasHelp_;}
    private:
        bool hasNotHandled(bool verbose){
            std::vector<std::string> vNotHandled;
            for (auto& pair : vRegisterd) {
                if(pair.second.required && !pair.second.handled){
                    vNotHandled.push_back(pair.first);
                }
            }
            if(!vNotHandled.empty()){
                if(verbose) {
                    printf("[Error] The following argument(s) should be given. Pass --h for help\n");
                    for (auto &noth : vNotHandled) {
                        printf("\t\t %s \"%s\"\n", noth.c_str(), vRegisterd[noth].explination.c_str());
                    }
                }
                return true;
            }
            return false;
        }
        bool hasNotRegistered(bool verbose){
            std::vector<std::string> vNotRegistered;
            for (auto& arg : argvs) {
                if(vRegisterd.count(arg.first) == 0){
                    vNotRegistered.push_back(arg.first);
                }
            }
            if(!vNotRegistered.empty()){
                if(verbose) {
                    printf("[Warning] Unknown argument(s) given. Please check the input arguments.\n");
                    for (auto &noth : vNotRegistered) {
                        printf("\t\t %s\n", noth.c_str());
                    }
                }
                return true;
            }
            return false;
        }

//        template <typename T, typename ... Types>
//        void process (std::string name, T& var, Types&& ... Args){
//            static const std::size_t Tsize = sizeof...(Types);
//            process(name, var);
//            for (int i=1; i<Tsize; ++i) {
//                process(std::forward<Types>(Args)...);
//            }
//        };

        bool find_parse (const std::string& target){
            if(argvs.count(target)){
                if(!argvs[target].empty())
                    return true;
            }
            return false;
        }

        bool find_switch(const std::string& target){
            if(argvs.count(target)){
                if(argvs[target].empty())
                    return true;
            }
            return false;
        }

        template <typename T>
        void process(const std::string &name, T& t){
            bool found = false;
            for (auto m: argvs) {
                if (m.first == name) {
                    found = true;
                    t.resize(m.second.size());
                    for (size_t i=0; i< m.second.size();++i) t.at(i) = m.second[i];
                }
            }
            if (!found){
                std::cout << "Cannot process command \"" << name << "\""  << std::endl;
            }
        }
        void process(const std::string &name, std::vector<std::string>& t){
            for (auto m: argvs) {
                if (m.first == name) {
                    t.resize(m.second.size());
                    for (size_t i=0; i< m.second.size();++i) t.at(i) = m.second[i];
                }
            }
        }
        void process(const std::string &name, std::vector<int>& t){
            for (auto m: argvs) {
                if (m.first == name) {
                    t.resize(m.second.size());
                    for (size_t i=0; i< m.second.size();++i) t.at(i) = std::stoi(m.second[i]);
                }
            }
        }
        void process(const std::string &name, std::vector<float>& t){
            for (auto m: argvs) {
                if (m.first == name) {
                    t.resize(m.second.size());
                    for (size_t i=0; i< m.second.size();++i) t.at(i) = std::stof(m.second[i]);
                }
            }
        }


        void process( const std::string &name, unsigned int* var, size_t size = 1) {
            for (auto m: argvs) {
                if (m.first == name) {
                    if (m.second.size() != size)
                        SCLOG(WARNING) << "Expect variable [" << m.first << "] to have " << size << " arguments, instread of " << m.second.size() << "\n";
                    for (size_t i = 0; i < size; ++i)
                        var[i] = std::stoi(m.second[i]);
                }
            }
        }
        void process( const std::string &name, std::string* var, size_t size = 1) {
            for (auto m: argvs) {
                if (m.first == name) {
                    if (m.second.size() != size)
                        SCLOG(WARNING) << "Expect variable [" << m.first << "] to have " << size << " arguments, instread of " << m.second.size() << "\n";
                    for (size_t i = 0; i < size; ++i)
                        var[i] = m.second[i];
                }
            }
        }
        void process( const std::string &name, int* var, size_t size = 1) {
            for (auto m: argvs) {
                if (m.first == name) {
                    if (m.second.size() != size)
                        SCLOG(WARNING) << "Expect variable [" << m.first << "] to have " << size << " arguments, instread of " << m.second.size() << "\n";
                    for (size_t i = 0; i < size; ++i)
                        var[i] = std::stoi(m.second[i]);
                }
            }
        }
        void process( const std::string &name, float* var, size_t size = 1) {
            for (auto m: argvs) {
                if (m.first == name) {
                    if (m.second.size() != size)
                        SCLOG(WARNING) << "Expect variable [" << m.first << "] to have " << size << " arguments, instread of " << m.second.size() << "\n";
                    for (size_t i = 0; i < size; ++i)
                        var[i] = std::stof(m.second[i]);
                }
            }
        }
        void process( const std::string &name, bool* var, size_t size = 1) {
            for (auto m: argvs) {
                if (m.first == name) {
                    if (m.second.size() != size)
                        SCLOG(WARNING) << "Expect variable [" << m.first << "] to have " << size << " arguments, instread of " << m.second.size() << "\n";
                    for (size_t i = 0; i < size; ++i)
                        var[i] = std::stoi(m.second[i]) > 0;
                }
            }
        }

        template<class T>
        static bool changeSwitchValue(T* var){
            *var = true;
            //*var = *var > 0? 0:1;
            return true;
        }

        static bool changeSwitchValue(std::string *var) {
            return false;
        }
    };
}

#endif /* Parser_hpp */

