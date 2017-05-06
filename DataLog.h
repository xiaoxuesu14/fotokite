//////// header DataLog.h ///////////
#include <string>
#include <fstream>

namespace DataLog
{
    extern const std::string path ;
    extern std::ofstream out ;
    void flush() ;
}