#ifndef MODELIMPORT_HH
#define MODELIMPORT_HH

#include "G4Tet.hh"
#include "G4Material.hh"
#include <Eigen/Geometry>
#include "functions.h"
#include "ClientSocket.h"

class ModelImport
{
public:
    ModelImport(ClientSocket*);
    G4bool RecvInitData();
    void PostureChange(const RotationList &vQ, const std::vector<Vector3d> &vT, bool calib);

    G4Tet* GetTetrahedron(G4int i) {return tetVec[i];}
    G4int GetNumOfTet() {return tetVec.size();}
    G4int GetNumOfFace() {return F.rows();}
    Vector3d GetCenter() {return center;}
    Vector3d GetSize() {return size;}
    G4Material* GetMaterial(G4int i) {return matMap[i];}
    G4int GetMaterialIdx(G4int i){
        if(i<T.rows()) return 148; //RST
        else return 121; //skin
    }
    G4int GetNonTargetNum(){return nonTargetNum;}
    void GenerateOffset();
private:
    void ReadMatFile(G4String filename);

    ClientSocket* socket;
    //std::vector<G4ThreeVector>  U;
    std::vector<std::map<int, double>> W;
    Eigen::MatrixXi F, T;
    MatrixXd V,V_calib, Wj, U;
    std::vector<G4Tet*> tetVec, tetVec_offset;
    Vector3d center, size;

    std::vector<G4ThreeVector> V_G4;
    std::map<G4int,G4ThreeVector> offsetV;

    std::map<G4int, G4Material*> matMap;
    std::map<G4int, std::vector<G4int>> adjacentFaces;

    /* |<--skin_d(t)-->0|<-- skin_d -->1|*/
    std::vector<G4ThreeVector> U_layer0,U_layer1;
    std::map<int, int> u2o;
    G4int nonTargetNum;
};

#endif // MODELIMPORT_HH
