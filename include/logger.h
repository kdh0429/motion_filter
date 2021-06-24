#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

class CsvLogger
{

public:
CsvLogger(std::string dirpath, std::string filename, std::vector<std::string> keys);
~CsvLogger();
void writeRows(std::vector<double> data);

private:

std::ofstream csv_file;
int ncols_;

};


#endif