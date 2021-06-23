#include <logger.h>
#include <iostream>

CsvLogger::CsvLogger(std::string filename, std::vector<std::string> keys)
{
    std::cout<< "Open "<<filename<<std::endl;

    csv_file.open(filename, std::ofstream::out | std::ofstream::app);
    ncols_ = keys.size();

    for (int i=0;i<ncols_;i++)
    {
        csv_file << keys.at(i);
        if (i < ncols_-1)
            csv_file << ',';
    }
    
    csv_file << "\n";
}

CsvLogger::~CsvLogger()
{
    std::cout<< "close file "<<std::endl;
    csv_file.close();
}

void CsvLogger::writeRows(std::vector<double> data)
{
    if (data.size()!= ncols_)
    {
        std::cout <<"Keys and data have different size"<<std::endl;
    }

    for (int i=0;i<data.size();i++)
    {
        csv_file << data.at(i);
        if (i < data.size() - 1)
            csv_file << ',';
    }
    
    csv_file << "\n";
}