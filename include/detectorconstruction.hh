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
// DetectorConstruction.hh
// \file   MRCP_GEANT4/External/include/DetectorConstruction.hh
// \author Haegin Han
//

#ifndef DetectorConstruction_h
#define DetectorConstruction_h 1

#include "G4VUserDetectorConstruction.hh"

#include <cmath>

#include "globals.hh"

#include "G4Material.hh"
#include "G4NistManager.hh"

#include "G4Box.hh"
#include "G4Tet.hh"
#include "G4LogicalVolume.hh"
#include "G4PVPlacement.hh"
#include "G4PVParameterised.hh"

#include "G4SDManager.hh"
#include "G4MultiFunctionalDetector.hh"
#include "G4SystemOfUnits.hh"
#include "G4GeometryManager.hh"
#include "G4RunManager.hh"

#include "detmessenger.hh"

#include "ModelImport.hh"
#include "tetparam.hh"

class DetMessenger;
class DetectorConstruction : public G4VUserDetectorConstruction
{
public:
    DetectorConstruction(ModelImport* tetData);
    virtual ~DetectorConstruction();

    virtual G4VPhysicalVolume* Construct();
    virtual void ConstructSDandField();

    void SetPatient(G4ThreeVector _rot, G4ThreeVector trans){
        G4double angle = _rot.mag();
        G4RotationMatrix rot; rot.setAxis(_rot/angle); rot.setTheta(angle*degree);
        transform = G4Transform3D(rot, trans);//-G4ThreeVector(0,0,worldHalfZ));

        if(container_phys){
            G4GeometryManager::GetInstance()->OpenGeometry();
            delete container_phys;
            container_phys = new G4PVPlacement(transform, container_logic, "PhantomPhysical",
                                               worldPhysical->GetLogicalVolume(), false, 0);
        }
    }

    void SetDetector(G4RotationMatrix rot, G4ThreeVector trans){
        det_transform = G4Transform3D(rot, trans);//-G4ThreeVector(0,0,worldHalfZ));

        if(det_phys){
            G4GeometryManager::GetInstance()->OpenGeometry();
            auto det_log = det_phys->GetLogicalVolume();
            delete det_phys;
            det_phys = new G4PVPlacement(det_transform, det_log, "PhantomPhysical",
                                               worldPhysical->GetLogicalVolume(), false, 0);
            //G4GeometryManager::GetInstance()->CloseGeometry();
            G4RunManager::GetRunManager()->GeometryHasBeenModified();
        }
    }
//    G4double GetWorldHalfZ(){return worldHalfZ;}

private:
    void SetupWorldGeometry();
    void ConstructPhantom();
    //void PrintPhantomInformation();

    G4VPhysicalVolume* worldPhysical;
    G4Box*             container_sol;
    G4LogicalVolume*   container_logic;
    G4VPhysicalVolume* container_phys;
    G4VPhysicalVolume* det_phys;
    G4Transform3D transform, det_transform;

    ModelImport*    tetData;
    DetMessenger*   fMessenger;


//    G4double worldHalfZ;
};

#endif
