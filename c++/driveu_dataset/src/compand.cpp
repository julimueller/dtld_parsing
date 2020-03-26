#include <compand.h>
#include <limits>
#include <fstream>
#include <sstream>

Decompand::Decompand() { }

Decompand::Decompand(const std::map<int,std::vector<int> > &kneepoints) {
    lutFromKneepoints(kneepoints);
}

Decompand::Decompand(const std::string &file) {

    std::map<int,std::vector<int> > kneepoints;
    if (loadKneepoints(file, kneepoints) == false) {
        return;
    }

    lutFromKneepoints(kneepoints);
}

Decompand::~Decompand() { }


void Decompand::lutFromKneepoints(const std::map<int,std::vector<int> > &kneepoints) {

    // check valid
    if (kneepoints.empty()) {
        return;
    }

    // start is always 0/0
    int decompanded = 0;
    int src_min = 0;
    int dst_min = 0;

    // reserve memory
    decompandLUT.clear();
    const int & max_in = (--kneepoints.end())->first;
    decompandLUT.reserve(max_in);

    int i = 1;
    for (std::map<int,std::vector<int> >::const_iterator p = kneepoints.begin(); p!=kneepoints.end(); ++p) {

        const int& src_max = p->first;
        const int& dst_max = (p->second)[0];
        const int& compression = (p->second)[1];

        for (int src = src_min; src <= src_max; ++src) {
            decompanded = (((src - src_min) * compression) + dst_min);
            if (decompanded > dst_max) {
                decompanded = dst_max;
            }
            decompandLUT.push_back(decompanded);
        }

        std::cout << "Decompanding section " << i << ": SRC " << src_min << " to " << src_max << " ---> DST: " << dst_min << " to " << decompanded << std::endl;
        src_min = src_max+1;
        dst_min = dst_max+1;
        i++;
    }
}

void Decompand::processPixel(const ushort &src, ushort &dst) {
  dst = decompandLUT[src];
}

void Decompand::processPixel(const ushort &src, unsigned char &dst) {
  dst = decompandLUT[src];
}

bool Decompand::saveLut(const std::string &file) {

    std::ofstream outfile(file);
    if( outfile.is_open() ){
        outfile << "# Y\t X" << std::endl;

        for(std::vector<int>::size_type i = 0; i != decompandLUT.size(); i++) {
            outfile << i << " \t " << decompandLUT[i] << std::endl;
        }
        outfile.close();
        return true;
    } else{
        std::cout << "Unable to open file: " << file << std::endl;
        return false;
    }
}

bool Decompand::loadKneepoints(const std::string &file, std::map<int, std::vector<int> > &kneepoints) {

    // open file
    std::ifstream f;
    f.open(file, std::ios_base::in);
    if (!f.is_open()) {
        std::cout << "ERROR while opening decompandig LUT file '" << file << "'!" << std::endl;
        return false;
    }

    std::string line;
    int line_cnt = 1;
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
    int compression = 0;

    // parse file
    while (std::getline(f, line)){
        if (!((line.empty()) || (line[0] == '#'))) {
            std::stringstream points;
            points << line;

            // extract Y and X from line
            points >> x2 >> y2;

            // check out of range
            if ((x2 > std::numeric_limits<unsigned short>::max()) || (y2 > std::numeric_limits<unsigned short>::max())) {
                std::cout << "ERROR while parsing decompandig LUT '" << file << "' (line: " << line_cnt << "). - Kneepoint exceeding unsigned short range (>65535)!" << std::endl;
                return false;
            }


            // checks division by zero and kneepoint order
            if ((x2-x1) <= 0) {
                std::cout << "ERROR while parsing decompandig LUT '" << file << "' (line: " << line_cnt << "). - A Kneepoint has to have a higher x-value than its precessor!" << std::endl;
                return false;
            }

            // check compression ratio (has to be integer)
            if ((y2-y1)%(x2-x1)){
                std::cout << "ERROR while parsing decompanding LUT '" << file << "' (line: " << line_cnt << "). -  Decompanding compression not of type integer. Change kneepoints accordingly.\n" << std::endl;
                return false;
            }

            // calculate compression and save kneepoint
            compression = (y2-y1)/(x2-x1);
            kneepoints[x2] = {y2,compression};

            //std::cout << "Kneepoint " << kneepoints.size() << ": " << x1 << ".." << x2 << ", compression: " << compression << " | mapped to: " << y1 << ".." << y2 << std::endl;

            x1 = x2;
            y1 = y2;
            line_cnt++;
        }
    }

    return true;
}



//=======================================================================================================



Compand::Compand(const std::map<int,std::vector<int> > &kneepoints, bool verbose) {

    // COMPANDING
    compandLUT.clear();

    for (std::map<int,std::vector<int> >::const_iterator p = kneepoints.begin(); p!=kneepoints.end(); ++p) {
        // Check for ushort overflow and non-negativity of kneepoints
        if (p->first > 65535 || p->second[0] > 65535 ){
            std::cout << "ERROR in compand.cpp: At least one decompanding kneepoint exceeds (16bit) unsigned short range (>65535). Adapt kneepoints!" << std::endl;
            return;
        } else if (p->first < 0 || p->second[0] < 0){
            std::cout << "ERROR in compand.cpp: Kneepoints are not allowed to be negative, at least one kneepoint is negative. Adapt kneepoints!" << std::endl;
            return;
        }
    }

    int compSrcMin = 0;
    int compDstMin = 0;
    int i = 1;
    for (std::map<int,std::vector<int> >::const_iterator p = kneepoints.begin(); p!=kneepoints.end(); ++p) {
        int compSrcMax = (p->second)[0];
        int compDstMax = p->first;
        int compression = (p->second)[1];

        if (verbose) {
            std::cout << "Companding section " << i << ": SRC " << compSrcMin << " to " << compSrcMax << " ---> DST: " << compDstMin << " to " << compDstMax << std::endl;
        }

        for (int src = compSrcMin; src <= compSrcMax; src++) {
            compandLUT.push_back( ((src-compSrcMin)/compression) + compDstMin);
        }

        compSrcMin = compSrcMax+1;
        compDstMin = compDstMax+1;
        i++;
    }
    if (verbose) {
        std::cout << "Created a compandingLUT with " << compandLUT.size() << " elements." << std::endl;
    }
}

Compand::~Compand() { }


void Compand::processPixel(const ushort &src, ushort &dst) {
    dst = compandLUT[src];
}

bool Compand::saveLut(const std::string &file) {

    std::ofstream outfile(file);
    if( outfile.is_open() ){
        outfile << "# X\t Y" << std::endl;

        for(std::vector<int>::size_type i = 0; i != compandLUT.size(); i++) {
            outfile << i << " \t " << compandLUT[i] << std::endl;
        }
        outfile.close();
        return true;
    } else{
        std::cout << "Unable to open file: " << file << std::endl;
        return false;
    }
}

bool Compand::loadLut(const std::string &file) {


    return true;
}
