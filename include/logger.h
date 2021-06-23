#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <vector>

class CsvLogger
{

public:
CsvLogger(std::string filename, std::vector<std::string> columns);
~CsvLogger();
void writeRows(std::vector<double> data);

private:

std::ofstream csv_file;
int ncols_;

};


#endif