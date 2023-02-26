#include <DataHandler.h>

namespace RCD
{
    DataHandler::DataHandler()
    {
        std::cout<<"DataHandler Constructor"<<std::endl;
    }
    DataHandler::~DataHandler()
    {
        std::cout<<"DataHandler De-Constructor"<<std::endl;
    }
    void DataHandler::test_pointersLogData()
    {
        int b = 5;
        log_data.a = &b;
        std::cout<<log_data.a<< " num is"<< *log_data.a <<std::endl;
    }
}
