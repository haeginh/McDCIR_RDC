#ifndef PhantomAnimator_class
#define PhantomAnimator_class

#include <iostream>
#include <ctime>
#include <functions.h>
#include <functional>

//#include "bodytracking.hh"

#include <igl/readTGF.h>
#include <igl/writeTGF.h>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>
#include <igl/writeMESH.h>
#include <igl/readMESH.h>
#include <igl/readPLY.h>
#include <igl/directed_edge_parents.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/boundary_conditions.h>
#include <igl/directed_edge_parents.h>
#include <igl/directed_edge_orientations.h>
#include <igl/deform_skeleton.h>
#include <igl/forward_kinematics.h>
#include <igl/doublearea.h>
#include <igl/dqs.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/Timer.h>
#include <igl/mat_max.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

class Viewer;
class PhantomAnimator{
//functions
public:
    PhantomAnimator();
    PhantomAnimator(string prefix);
    ~PhantomAnimator();

    bool ReadFiles(string prefix);
    bool Initialize();
    string CalibrateTo(string name);
    void Animate(RotationList vQ, const MatrixXd &C_disp, MatrixXd &C_new, bool calibChk = true);
    void Animate(RotationList vQ, MatrixXd &V_new);

    void GetMeshes(MatrixXd &_V, MatrixXi &_F, MatrixXd &_C, MatrixXi &_BE){
        _V = V; _F = F; _C = C; _BE = BE;
    }
    MatrixXd GetV(){ return V; }
    MatrixXd GetU(){ return U; }
    MatrixXi GetF(){ return F; }
    MatrixXd GetC(){ return C; }
    MatrixXd GetC_calib(){ return C_calib; }
    MatrixXi GetBE(){ return BE; }
    ArrayXd GetWSkin(){return W_avgSkin;}
    MatrixXd GetUapron(){return U_apron;}
    VectorXi GetOutApron(){return outApron;}
    ArrayXd GetApronMask(){return apronMask;}
    MatrixXi GetFapron(){return F_apron;}
    // map<int, double> GetWLens(){return lensWeight;}
    RotationList GetAlignRot() {return alignRot;}
    void GetEyeIdx(int &R, int&L){R = rightEye; L = leftEye;}
   
public:
    bool ReadProfileData(string fileName);
    bool WriteProfileData(string fileName);
    int AddProfile(map<int, double> calibLengths, Vector3d eyeL_pos, Vector3d eyeR_pos, string name){
        auto iter = profileIDs.insert(make_pair(name, jointLengths.size()));
        jointLengths.push_back(calibLengths);
        eyeL_vec.push_back(eyeL_pos);
        eyeR_vec.push_back(eyeR_pos);
        return distance(profileIDs.begin(), iter.first);
    }
    vector<string> GetProfileNames()
    {
        vector<string> names;
        for(auto iter:profileIDs) names.push_back(iter.first);
        return names;
    }
    bool AlreadyExists(string name)
    {
        if(profileIDs.find(name)==profileIDs.end()) return false;
        else return true;
    }
    map<string, int> profileIDs;
    vector<map<int, double>> jointLengths;
    vector<Vector3d> eyeR_vec, eyeL_vec;

//variables
private:
    MatrixXd C, V, U, Wj, V_apron, U_apron, Wj_apron;
    MatrixXi BE, T, F, F_apron;
    VectorXi P;
    ArrayXd W_avgSkin;
    // map<int, double> lensWeight;
    MatrixXd V_calib, C_calib, V_calib_apron;
    int rightEye, leftEye;
    // vector<int> eyeIDs;
    // vector<int> eye2ply;
    RotationList alignRot;
    vector<map<int, double>> cleanWeights, cleanWeightsApron;

    //tmp
    VectorXi outApron;
    ArrayXd apronMask;
    map<int, double> lengths;
};

#endif