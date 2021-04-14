#ifndef MODELIMPORT_HH
#define MODELIMPORT_HH

#include "G4Tet.hh"
#include "G4Material.hh"
#include <Eigen/Geometry>
#include "functions.h"
#include "ClientSocket.h"
#include <set>
#include "G4SystemOfUnits.hh"
class ModelImport
{
public:
    ModelImport(ClientSocket*);
    G4bool RecvInitData();
    void ResetCalibData(MatrixXd calib_M){
        V_calib = V+Wj*calib_M*cm;
    }
    void ResetCalibData(){
        V_calib = V+Wj*calib_M0*cm;
    }

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
    std::set<G4int> GetEyeFaces(){return eyeFaces;}
private:
    void ReadMatFile(G4String filename);

    ClientSocket* socket;
    //std::vector<G4ThreeVector>  U;
    std::vector<std::map<int, double>> W;
    Eigen::MatrixXi F, T;
    MatrixXd V,V_calib, Wj, U, calib_M0;
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

    //eye
    std::set<G4int> eyeFaces;

    //degen
private:
    void ChkDegenAndSetVertices(G4ThreeVector a, G4ThreeVector b, G4ThreeVector c, G4ThreeVector d, G4Tet* tet){
        G4double vol6;
        if (ChkDegenTet2(a, b, c, d, vol6)) {
            G4cout << "  degenerated tet!! --> " <<flush;
            double move;
            move = FixDegenTet3(a, b, c, d, vol6);
            G4cout << "fixed (moved "<<move<<" cm)" <<G4endl;
        }
        tet->SetVertices(a,b,c,d);
    }
    G4double FixDegenTet3(G4ThreeVector& anchor, G4ThreeVector& p2, G4ThreeVector& p3, G4ThreeVector& p4, G4double &vol6);
    G4bool ChkDegenTet2(G4ThreeVector p0, G4ThreeVector p1, G4ThreeVector p2, G4ThreeVector p3, G4double& vol6);

};

#endif // MODELIMPORT_HH
