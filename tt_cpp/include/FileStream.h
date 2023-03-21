#include <iostream>
#include <fstream>

class FileStream
{
private:
    std::string filePath;

public:
    FileStream(std::string filePath) {this->filePath = filePath;}
    int writeToFile(std::string inputString, bool append);
    int writeToFile_formatted(float x, float y, float theta);
};

int FileStream::writeToFile(std::string inputString, bool append)
{
    std::ofstream file;

    try
    {
        if(append) file.open(filePath, std::ios_base::app);
        else file.open(filePath);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    file.write(inputString.c_str(), inputString.size());
    return 0;
}

int FileStream::writeToFile_formatted(float x, float y, float theta)
{
    std::ofstream file;

    try
    {
        file.open(filePath, std::ios_base::app);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    std::string line_1 = "id=";
    std::string line_2 = "pickup_x(mm)=" + std::to_string(x);
    std::string line_3 = "pickup_y(mm)=" + std::to_string(y);
    std::string line_4 = "pickup_theta(degrees)=" + std::to_string(theta);
    std::string line_5 = "#############################";

    file.write(line_1.c_str(), line_1.size());
    file.write("\n", 1);
    file.write(line_2.c_str(), line_2.size());
    file.write("\n", 1);
    file.write(line_3.c_str(), line_3.size());
    file.write("\n", 1);
    file.write(line_4.c_str(), line_4.size());
    file.write("\n", 1);
    file.write(line_5.c_str(), line_5.size());
    file.write("\n", 1);

    return 0;
}