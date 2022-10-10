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

static vector<string> BFlist = {"M26", "M29", "M31", "M33", "M36", "F26", "F29", "F32", "F35", "F39"};

// class Viewer;
class PhantomAnimator{
public:
    PhantomAnimator()
    :useApron(true), irrTime(0)
    {
        phantomDir = getenv("DCIR_PHANTOM_DIR");
    }
    ~PhantomAnimator(){ Clear(); }

    bool LoadPhantom(string phantomName, bool isMale);
    bool LoadPhantomWithWeightFiles(string phantomName, bool isMale);
    bool CalibrateTo(map<int, double> jl, RowVector3d eyeL_pos, RowVector3d eyeR_pos);
    void Clear(){
        V.resize(0, 0); 
        U.resize(0, 0);
        F.resize(0, 0);
        C.resize(0, 0);
        BE.resize(0, 0);
        F_apron.resize(0, 0);
        V_apron.resize(0, 0);
        U_apron.resize(0, 0);
    }

    void Animate(RotationList vQ, const MatrixXd &C_disp, MatrixXd &C_new, bool withApron = false);
    // void Animate(RotationList vQ, MatrixXd &V_new);

    void GetMeshes(MatrixXd &_V, MatrixXi &_F, MatrixXd &_C, MatrixXi &_BE){
        _V = V; _F = F; _C = C; _BE = BE;
    }

    RotationList GetAlignRot() {return alignRot;}

    //debug
    VectorXd GetWeight(int id)
    {
        VectorXd Wvec(cleanWeights.size());
        for(int i=0;i<cleanWeights.size();i++)
        {
            if(cleanWeights[i].find(id)==cleanWeights[i].end()) Wvec(i) = 0;
            else Wvec(i) = cleanWeights[i][id];
        }
        return Wvec;
    }

//variables
    MatrixXd C, V, U, Wj, V_apron, U_apron, Wj_apron;
    MatrixXi BE, T, F, F_apron;   
private:
    bool Initialize();

    VectorXi P;
    RowVectorXd W_avgSkin;
    RotationList alignRot;
    vector<map<int, double>> cleanWeights, cleanWeightsApron;
    vector<vector<int>> endC;

    //tmp
    ArrayXd apronMask;
    map<int, double> lengths;
public:
    bool useApron;
    double irrTime;
    VectorXi outApron;
    RowVectorXd lHandW, rHandW;
    string phantomDir;
    //dose
    VectorXd accD, rateD; //Gy, Gy/s
    Vector2d accD_eye, rateD_eye;
    void SetDose(VectorXd _doseRate, Vector2d _eyeDose, double mSec){
        if(useApron){
            VectorXd dose = VectorXd::Zero(V.rows());
            igl::slice_into(_doseRate, outApron, dose);
            _doseRate = dose;
        }
        rateD = _doseRate;// / mSec * 1000; 
        accD += _doseRate * mSec * 0.001;
        rateD_eye = _eyeDose;// / mSec * 1000;
        accD_eye += _eyeDose * mSec * 0.001;
        irrTime += mSec * 0.001;
    }
    void SetZeroDose(){
        rateD = VectorXd::Zero(V.rows()); 
        rateD_eye = Vector2d::Zero();
    }
    double GetAvgSkinDoseRate(){
        if(rateD.rows()>0) return W_avgSkin*rateD;
        else return 0;
    }
    double GetRightHandDoseRate(){
        if(rateD.rows()>0) return rHandW*rateD;
        else return 0;
    }
    double GetRightHandAccDose(){
        if(rateD.rows()>0) return rHandW*accD;
        else return 0;
    }
    double GetLeftHandDoseRate(){
        if(rateD.rows()>0) return lHandW*rateD;
        else return 0;
    }
    double GetLeftHandAccDose(){
        if(rateD.rows()>0) return lHandW*accD;
        else return 0;
    }
    double GetAvgAccSkinDose(){
        if(accD.rows()>0) return W_avgSkin*accD;
        else return 0;
    }
    double GetMaxSkinDoseRate(){
        if(rateD.rows()>0) return rateD.maxCoeff();
        else return 0;
    }
    double GetMaxAccSkinDose(){
        if(accD.rows()>0) return accD.maxCoeff();
        else return 0;
    }
    void ClearDose(){
        accD = VectorXd::Zero(V.rows());
        rateD = accD;
        rateD_eye = Vector2d::Zero();
        accD_eye = rateD_eye;
    }

    // jp==
    // map<int, >
};

#endif