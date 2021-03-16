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
//

#include "detectorconstruction.hh"


DetectorConstruction::DetectorConstruction(ModelImport* _tetData)
:worldPhysical(0), container_logic(0), tetData(_tetData)
{
    det_transform = G4Transform3D(G4RotationMatrix(), G4ThreeVector(0,-42.8,60)*cm);
    fMessenger = new DetMessenger(this);
}

DetectorConstruction::~DetectorConstruction()
{
    delete tetData;
    delete fMessenger;
}

G4VPhysicalVolume* DetectorConstruction::Construct()
{
    SetupWorldGeometry();
    ConstructPhantom();

    //detector Geom.
    G4Box* det = new G4Box("det", 42*0.5*cm, 15*0.5*cm, 52*0.5*cm);
    det_phys = new G4PVPlacement(det_transform,
                                 new G4LogicalVolume(det, G4NistManager::Instance()->FindOrBuildMaterial("G4_W"),"det_log"),
                                 "det_phy", worldPhysical->GetLogicalVolume(), false, 0, false);

    return worldPhysical;
}

void DetectorConstruction::SetupWorldGeometry()
{
    // Define the world box (size: 10*10*10 m3)
    //
    G4double worldHalfZ = 2*m;
    G4double worldXY = 4 * m;
    G4Material* air = G4NistManager::Instance()->FindOrBuildMaterial("G4_AIR");

    G4VSolid* worldSolid
      = new G4Box("worldSolid", worldXY/2, worldXY/2, worldHalfZ);

    G4LogicalVolume* worldLogical
      = new G4LogicalVolume(worldSolid,air,"worldLogical");

    worldPhysical
      = new G4PVPlacement(0,G4ThreeVector(), worldLogical,"worldPhysical", 0, false,0,false);

    // Define the phantom container (1-cm margins from the bounding box of phantom)
    //
    container_sol = new G4Box("phantomBox", tetData->GetHalfSize().x() + 1.*cm,
                                            tetData->GetHalfSize().y() + 1.*cm,
                                            tetData->GetHalfSize().z() + 1.*cm);

    container_logic = new G4LogicalVolume(container_sol, air, "phantomLogical");

    container_phys = new G4PVPlacement(0, tetData->GetCenter(), container_logic, "PhantomPhysical",
                                       worldLogical, false, 0);
//    container_logic->SetOptimisation(TRUE);
//    container_logic->SetSmartless( 0.5 ); // for optimization (default=2)
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
    G4LogicalVolume* tetLogic = new G4LogicalVolume(tetraSolid, tissue, "TetLogic");

    // physical volume (phantom) constructed as parameterised geometry
    new G4PVParameterised("wholePhantom",tetLogic,container_logic,
                          kUndefined, tetData->GetNumOfTet(),
                          new TetParam(tetData));
}

void DetectorConstruction::ConstructSDandField()
{
}

