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
    string Calibrate(map<int, double> calibLengths, Vector3d eyeL_pos, Vector3d eyeR_pos, int calibFrame);
    void Animate(RotationList vQ, const MatrixXd &C_disp, MatrixXd &C_new, MatrixXd &V_new, bool calibChk = true);

    void GetMeshes(MatrixXd &_V, MatrixXi &_F, MatrixXd &_C, MatrixXi &_BE){
        _V = V; _F = F; _C = C; _BE = BE;
    }
    MatrixXd GetV(){ return V; }
    MatrixXi GetF(){ return F; }
    MatrixXd GetC(){ return C; }
    MatrixXd GetC_calib(){ return C_calib; }
    MatrixXi GetBE(){ return BE; }
    RotationList GetAlignRot() {return alignRot;}
   
private:
//variables
private:
    MatrixXd C, V, W, Wj;
    MatrixXi BE, T, F;
    VectorXi P;
    MatrixXd V_calib, C_calib;
    vector<int> eye2ply;
    RotationList alignRot;
    vector<map<int, double>> cleanWeights;

    //tmp
    map<int, double> lengths;
};

#endif