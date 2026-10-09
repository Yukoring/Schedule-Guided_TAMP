#include "cpsat_model.hpp"
#include <iostream>
int main() {
    std::string line;
    while(std::getline(std::cin,line)) {
        try { auto p=grstapse::numericFromJson(nlohmann::json::parse(line));
              std::cout<<grstapse::numericResultJson(grstapse::solveNumericSchedule(p)).dump()<<std::endl; }
        catch(const std::exception& e) { std::cout<<nlohmann::json({{"error",e.what()}}).dump()<<std::endl; }
    }
}
