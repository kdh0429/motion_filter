#include <logger.h>
#include <iostream>

CsvLogger::CsvLogger(std::string dirpath, std::string filename, std::vector<std::string> keys)
{
    boost::filesystem::path dir(dirpath);
    std::string full_path;

    if(boost::filesystem::create_directory(dir))
    {
        std::cerr<< "Directory Created: "<<dirpath<<std::endl;
    }

    full_path = dirpath + "/" + filename;
    std::cout<< "Open "<<full_path<<std::endl;
    csv_file.open(full_path, std::ofstream::out | std::ofstream::app);
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