//
// Created by Kira on 24-4-16.
//
#include "Tools.h"
namespace ArduCV{
    void Console::WriteLine(const std::string& str){
        std::cout<< HIGHLIGHT str CLRST <<"\n";
    }
    void Console::Write(const std::string& str){
        std::cout<< HIGHLIGHT str CLRST;
    }
    void Console::Error(const std::string& str){
        std::cout<< RED str CLRST;
    }
    void Console::Warning(const std::string& str){
        std::cout<< YELLOW str CLRST;
    }
}