//
// Created by Kira on 24-4-16.
//
#ifndef ARDUCV_TOOLS_H
#define ARDUCV_TOOLS_H

#include <iostream>
#include <string>

#define RED     "\033[31m" <<
#define GREEN   "\033[32m" <<
#define YELLOW  "\033[33m" <<
#define BLUE    "\033[34m" <<
#define MAGENTA "\033[35m" <<
#define CYAN    "\033[36m" <<
#define WHITE   "\033[37m" <<

#define HIGHLIGHT BLUE
#define CLRST   << "\033[0m"

namespace ArduCV {
    class Console {
    public:
        static void WriteLine(const std::string &str);

        static void Write(const std::string &str);

        static void Error(const std::string &str);

        static void Warning(const std::string &str);
    };
}

#endif //ARDUCV_TOOLS_H
