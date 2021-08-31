#ifndef MapContainer_class
#define MapContainer_class

#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
typedef tuple<int, int> MAPIDX;
class MapContainer{
    public:
    MapContainer();
    void ReadMapList();
    void ReadMap(string name);
    void SetIJK(int _i, int _j, int _k) {i=_i; j=_j; k=_k;}
    int GetI(){return i;}
    int GetJ(){return j;}
    int GetK(){return k;}
    double GetSkinDose(int idx){return doseMapS[idx];}
    double GetLensDose(int idx){return doseMapL[idx];}
    double GetMaxSkin(){return maxSkin;}
    double GetInvDAPperNPS(){return invDAP;}
    Vector3d GetIsoCenter(){return isoCenter;}
    private:
    //variables
    int i, j, k;
    vector<double> doseMapS, doseMapL;
    double maxSkin, invDAP;
    map<MAPIDX, int> mapList;
    map<int, double> mapDAPs;
    Vector3d isoCenter;
};

#endif