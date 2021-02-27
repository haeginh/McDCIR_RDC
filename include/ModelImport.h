#ifndef MODELIMPORT_HH
#define MODELIMPORT_HH

#include "G4Tet.hh"
#include <Eigen/Geometry>
#include "functions.h"
#include "ClientSocket.h"

class ModelImport
{
public:
    ModelImport(ClientSocket*);
    G4bool RecvInitData();
    void PostureChange(const RotationList &vQ, const std::vector<Vector3d> &vT);

private:
    ClientSocket* socket;
    std::vector<G4ThreeVector>  U;
    std::vector<std::map<int, double>> W;
    Eigen::MatrixXi F, T;
    MatrixXd V, V_calib, Wj;
    std::vector<G4Tet*> tetVec;

    std::vector<G4ThreeVector> V_G4;
    std::map<int,G4ThreeVector> offsetV;
};

#endif // MODELIMPORT_HH
