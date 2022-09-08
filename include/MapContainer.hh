#ifndef MapContainer_class
#define MapContainer_class

#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
typedef
  std::vector<Eigen::ArrayXf,Eigen::aligned_allocator<Eigen::ArrayXf>>
  ArrayList;
// typedef tuple<int, int> MAPIDX;
class MapContainer{
    public:
    MapContainer();
    void ReadSpectrum(string name);
    bool ReadMap(string name, ArrayXf& _skinMap, ArrayXf& _lensMap);
    bool SetCArm(int kVp, int rot, int ang);
    void SetDoseRate(const MatrixXf &U, VectorXd &D, double dap);

    // void SetDAP(double _dap) {dap = _dap;}
    // void SetIJK(int _i, int _j, int _k) {i=_i; j=_j; k=_k;}
    // int GetI(){return i;}
    // int GetJ(){return j;}
    // int GetK(){return k;}
    
    // void GetSkinDoseMap(VectorXd& _doseMapS) {_doseMapS = doseMapS;}
    // double GetSkinDose(int idx){return doseMapS(idx);}
    // double GetLensDose(int idx){return doseMapL(idx);}
    // double GetMaxSkin(){return maxSkin;}
    // double GetInvDAPperNPS(){return invDAP;}
    // Vector3d GetIsoCenter(){return isoCenter;}
    private:
    Matrix3f CarmRotation(double rot, double ang)
    {
        return
            (AngleAxisf(rot/180.*M_PI, Vector3f(0, 1, 0))*
             AngleAxisf(ang/180.*M_PI, Vector3f(1, 0, 0))).matrix();
    }
    //variables
    string mapDir, specDir;
    RowVector3f base; //lowest (X, Y, Z)
    RowVector3i numGrid; //number of grids (X, Y, Z)
    int totalNum;
    double gridConv;
    // int i, j, k, baseIdx;
    ArrayList skinMapList, lensMapList;
    ArrayXf skinMap, lensMap;
    Matrix3f cArm_IT;
    
    // double dap;
    int rot0, ang0, kVp0;
    map<int, double> spec;
    // vector<double> doseMapS, doseMapL;
    // double maxSkin, invDAP;
    // int leftE, rightE;
    // map<MAPIDX, int> mapList;
    // map<int, double> mapDAPs;
    // sVector3d isoCenter;
};

#endif