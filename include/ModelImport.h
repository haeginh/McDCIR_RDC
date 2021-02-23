#ifndef MODELIMPORT_HH
#define MODELIMPORT_HH

#include "G4Tet.hh"
#include <Eigen/Geometry>
#include "functions.h"

class ModelImport
{
public:
    ModelImport(G4String modelName);
    G4bool RecvInitData();
    void PostureChange(const RotationList &vQ, const std::vector<Vector3d> &vT);

private:
    G4int vNum;
    std::vector<G4ThreeVector>  V, V_calib, U;
    std::vector<std::map<int, double>> W;
    Eigen::MatrixXi F, T;
    std::vector<G4Tet*> tetVec;

    std::vector<G4ThreeVector> V_G4;
    std::map<int,G4ThreeVector> offsetV;
};

#endif // MODELIMPORT_HH
