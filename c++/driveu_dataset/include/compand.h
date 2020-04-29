#ifndef DRIVEU_DATASET_COMPAND_H_
#define DRIVEU_DATASET_COMPAND_H_


#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <cmath>


class Compand {
public:
    Compand(const std::map<int,std::vector<int> > &kneepoints, bool verbose=false);
    virtual ~Compand();

    void processPixel(const ushort &src, ushort &dst);
    bool saveLut(const std::string &file);
    bool loadLut(const std::string &file);

private:
    std::vector<int> compandLUT;

};


class Decompand {
public:
    Decompand();
    Decompand(const std::map<int,std::vector<int> > &kneepoints);
    Decompand(const std::string &file);

    virtual ~Decompand();

    void processPixel(const ushort &src, ushort &dst);
    void processPixel(const ushort &src, unsigned char &dst);
    bool saveLut(const std::string &file);
    bool loadKneepoints(const std::string &file, std::map<int, std::vector<int> > &kneepoints);

private:
    void lutFromKneepoints(const std::map<int,std::vector<int> > &kneepoints);
    std::vector<int> decompandLUT;

};

#endif // DRIVEU_DATASET_COMPAND_H_
