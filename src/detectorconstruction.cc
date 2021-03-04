//
// ********************************************************************
// * License and Disclaimer                                           *
// *                                                                  *
// * The  Geant4 software  is  copyright of the Copyright Holders  of *
// * the Geant4 Collaboration.  It is provided  under  the terms  and *
// * conditions of the Geant4 Software License,  included in the file *
// * LICENSE and available at  http://cern.ch/geant4/license .  These *
// * include a list of copyright holders.                             *
// *                                                                  *
// * Neither the authors of this software system, nor their employing *
// * institutes,nor the agencies providing financial support for this *
// * work  make  any representation or  warranty, express or implied, *
// * regarding  this  software system or assume any liability for its *
// * use.  Please see the license in the file  LICENSE  and URL above *
// * for the full disclaimer and the limitation of liability.         *
// *                                                                  *
// * This  code  implementation is the result of  the  scientific and *
// * technical work of the GEANT4 collaboration.                      *
// * By using,  copying,  modifying or  distributing the software (or *
// * any work based  on the software)  you  agree  to acknowledge its *
// * use  in  resulting  scientific  publications,  and indicate your *
// * acceptance of all terms of the Geant4 Software license.          *
// ********************************************************************
//
// DetectorConstruction.cc
// \file   MRCP_GEANT4/External/src/DetectorConstruction.cc
// \author Haegin Han
//

#include "detectorconstruction.hh"
#include "G4PSEnergyDeposit.hh"

DetectorConstruction::DetectorConstruction(ModelImport* _tetData, ClientSocket* _sock)
:worldPhysical(0), container_logic(0), tetData(_tetData), tetLogic(0), sock(_sock)
{
    fMessenger = new DetectorMessenger(this);
}

DetectorConstruction::~DetectorConstruction()
{
    delete tetData;
}

G4VPhysicalVolume* DetectorConstruction::Construct()
{
    SetupWorldGeometry();
    ConstructPhantom();

   // PrintPhantomInformation();
    return worldPhysical;
}

void DetectorConstruction::SetupWorldGeometry()
{
    // Define the world box (size: 10*10*10 m3)
    //
    G4double worldXYZ = 10. * m;
    G4Material* air = G4NistManager::Instance()->FindOrBuildMaterial("G4_AIR");

    G4VSolid* worldSolid
      = new G4Box("worldSolid", worldXYZ/2, worldXYZ/2, worldXYZ/2);

    G4LogicalVolume* worldLogical
      = new G4LogicalVolume(worldSolid,air,"worldLogical");

    worldPhysical
      = new G4PVPlacement(0,G4ThreeVector(), worldLogical,"worldPhysical", 0, false,0,false);

    // Define the phantom container (10-cm margins from the bounding box of phantom)
    //
    container_sol = new G4Box("phantomBox", tetData->GetSize()(0) + 1.*cm,
                                                    tetData->GetSize()(1) + 1.*cm,
                                                    tetData->GetSize()(2) + 1.*cm);

    container_logic = new G4LogicalVolume(container_sol, air, "phantomLogical");

    container_phys = new G4PVPlacement(0, G4ThreeVector(), container_logic, "PhantomPhysical",
                                       worldLogical, false, 0);
    container_logic->SetOptimisation(TRUE);
    container_logic->SetSmartless( 0.5 ); // for optimization (default=2)
}

void DetectorConstruction::ConstructPhantom()
{
    // Define the tetrahedral mesh phantom as a parameterised geometry
    //
    // solid and logical volume to be used for parameterised geometry
    G4VSolid* tetraSolid = new G4Tet("TetSolid",
                                G4ThreeVector(),
                                G4ThreeVector(1.*cm,0,0),
                                G4ThreeVector(0,1.*cm,0),
                                G4ThreeVector(0,0,1.*cm));

    G4Material* tissue = G4NistManager::Instance()->FindOrBuildMaterial("G4_ADIPOSE_TISSUE_ICRP");
    //G4Material* vacuum = G4NistManager::Instance()->FindOrBuildMaterial("G4_Galactic");
    tetLogic = new G4LogicalVolume(tetraSolid, tissue, "TetLogic");

    // physical volume (phantom) constructed as parameterised geometry
    new G4PVParameterised("wholePhantom",tetLogic,container_logic,
                          kUndefined, tetData->GetNumOfTet(),
                          new TetParam(tetData));
}

void DetectorConstruction::ConstructSDandField()
{
    // Define detector (Phantom SD) and scorer (eDep)
    //
    G4MultiFunctionalDetector* MFDet = new G4MultiFunctionalDetector("phantomSD");
    G4SDManager::GetSDMpointer()->AddNewDetector(MFDet);

    // scorer for energy depositon in each organ
    G4VPrimitiveScorer* edep = new G4PSEnergyDeposit("edep");
    MFDet->RegisterPrimitive(edep);
    SetSensitiveDetector(tetLogic,MFDet);
}

void DetectorConstruction::SetUpNewFrame(const RotationList &vQ, const std::vector<Vector3d> &vT, bool calib = false){
    tetData->PostureChange(vQ, vT, calib);
    Vector3d size = tetData->GetSize()*0.5;
    Vector3d center = tetData->GetCenter();
    container_sol->SetXHalfLength(size(0)+10*cm);
    container_sol->SetYHalfLength(size(1)+10*cm);
    container_sol->SetZHalfLength(size(2)+10*cm);
    G4ThreeVector trans = G4ThreeVector(center(0),center(1),center(2));
    container_phys->SetTranslation(trans);
}

#include "G4RunManager.hh"
void DetectorConstruction::NextFrame(){
    array<double, 155> pack;
    sock->RecvDoubleBuffer(pack.data(),155);

    G4cout<<"calculate frame #"<<(int)pack[0]<<G4endl;
    RotationList vQ;
    vector<Vector3d> vT;
    for(int i=0;i<22;i++){
        vQ.push_back(Quaterniond(pack[4*i+1],pack[4*i+2],pack[4*i+3],pack[4*i+4]));
        vT.push_back(Vector3d(pack[3*i+89], pack[3*i+90], pack[3*i+91])*cm);
    }
    bool calib(false);
    if(pack[0]<0) calib = true;
    SetUpNewFrame(vQ,vT, calib);
    G4RunManager::GetRunManager()->GeometryHasBeenModified();
}
//void DetectorConstruction::PrintPhantomInformation()
//{
//    // print brief information on the imported phantom
//    G4cout<< G4endl;
//    G4cout.precision(3);
//    G4cout<<"   Phantom name               "<<tetData->GetPhantomName() << " TET phantom"<<G4endl;
//    G4cout<<"   Phantom size               "<<phantomSize.x()<<" * "<<phantomSize.y()<<" * "<<phantomSize.z()<<" mm3"<<G4endl;
//    G4cout<<"   Phantom box position (min) "<<phantomBoxMin.x()<<" mm, "<<phantomBoxMin.y()<<" mm, "<<phantomBoxMin.z()<<" mm"<<G4endl;
//    G4cout<<"   Phantom box position (max) "<<phantomBoxMax.x()<<" mm, "<<phantomBoxMax.y()<<" mm, "<<phantomBoxMax.z()<<" mm"<<G4endl;
//    G4cout<<"   Number of tetrahedrons     "<<nOfTetrahedrons<<G4endl<<G4endl;
//}
