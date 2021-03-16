#ifndef MODELIMPORT_HH
#define MODELIMPORT_HH

#include "G4Tet.hh"
#include "G4Material.hh"

class ModelImport
{
public:
    ModelImport();
    G4bool RecvInitData();

    G4Tet* GetTetrahedron(G4int i) {return tetVec[i];}
    G4int GetNumOfTet() {return tetVec.size();}
    G4Material* GetMaterial(G4int i) {return matMap[i];}
    G4int GetMaterialIdx(G4int i){
        return matVec[i];
    }
    G4ThreeVector GetHalfSize(){return size;}
    G4ThreeVector GetCenter(){return center;}

private:
    void ReadPhantomFile(G4String filename);
    G4ThreeVector center, size;

    std::vector<G4Tet*> tetVec;
    std::vector<G4int>  matVec;
    std::map<G4int, G4Material*> matMap;
};

#endif // MODELIMPORT_HH
