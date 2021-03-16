#include "ModelImport.hh"
#include "G4SystemOfUnits.hh"
#include "G4NistManager.hh"

ModelImport::ModelImport()
{
    ReadPhantomFile("patient");
    matMap[1] = G4NistManager::Instance()->FindOrBuildMaterial("G4_TISSUE_SOFT_ICRP");
    matMap[2] = G4NistManager::Instance()->FindOrBuildMaterial("G4_LUNG_ICRP");
}

void ModelImport::ReadPhantomFile(G4String filename){
    //simple patient phantom file consists of 1(tissue) and 2(lung) mats.
    std::vector<G4ThreeVector> nodeVec;
    std::ifstream ifsNode(filename+".node");
    G4int numOfNodes; G4String dump; G4double x, y, z;
    G4ThreeVector max, min;
    ifsNode>>numOfNodes>>dump>>dump>>dump;
    for(G4int i=0;i<numOfNodes;i++){
        ifsNode>>dump>>x>>y>>z;
        nodeVec.push_back(G4ThreeVector(x,y,z)*cm);
        if(max.x()<x)max.setX(x);
        else if(min.x()>x)min.setX(x);
        if(max.y()<y)max.setY(y);
        else if(min.y()>y)min.setY(y);
        if(max.z()<z)max.setZ(z);
        else if(min.z()>z)min.setZ(z);
    }ifsNode.close();

    center = (min+max)*0.5*cm;
    size   = (max-min)*0.5*cm;
    G4cout<<"Center: "<<center/cm<<" cm"<<G4endl;
    G4cout<<"Size  : "<<size  /cm<<" cm"<<G4endl;

    std::ifstream ifsEle(filename+".ele");
    G4int numOfEle, a, b, c, d, id;
    ifsEle>>numOfEle>>dump>>dump;
    for(G4int i=0;i<numOfEle;i++){
        ifsEle>>dump>>a>>b>>c>>d>>id;
        tetVec.push_back(new G4Tet("tet",nodeVec[a]-center,nodeVec[b]-center,nodeVec[c]-center,nodeVec[d]-center));
        matVec.push_back(id);
    }
}
