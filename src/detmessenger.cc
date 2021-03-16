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
// \author Haegin Han
//

#include "G4UIdirectory.hh"
#include "G4UIcmdWith3Vector.hh"
#include "G4UIcmdWith3VectorAndUnit.hh"
#include "G4UIcmdWithADoubleAndUnit.hh"
#include "G4UIcmdWithoutParameter.hh"

#include "detmessenger.hh"
#include "G4GeometryManager.hh"

DetMessenger::DetMessenger(DetectorConstruction* _det)
:G4UImessenger(), fDet(_det)
{
    fDetDir = new G4UIdirectory("/det/");

    fRotCmd = new G4UIcmdWith3Vector("/det/rot", this);
    fRotCmd->SetGuidance("unit axis vector * angle in degree");

    fIsoCmd = new G4UIcmdWith3VectorAndUnit("/det/isoC", this);
    fIsoCmd->SetDefaultUnit("cm");

    //primary values
    isocenter = G4ThreeVector(0,0,60)*cm;
    source = G4ThreeVector(0,81,0)*cm;
    detY = -35.3*cm;
    detMinDir = G4ThreeVector(-30.61*cm*0.5,detY,-39.54*cm*0.5)-source;
    detXdir = G4ThreeVector(30.61*cm, 0, 0);
    detZdir = G4ThreeVector(0, 0, 39.54*cm);
}

DetMessenger::~DetMessenger() {
    delete fDetDir;
    delete fRotCmd;
    delete fIsoCmd;
}

void DetMessenger::SetNewValue(G4UIcommand* command, G4String newValue)
{
    if(command == fRotCmd){
        G4ThreeVector vec = fRotCmd->GetNew3VectorValue(newValue);
        double ang = vec.mag() * deg;
        G4ThreeVector axis = vec.unit();
        rot = G4RotationMatrix();
        rot.rotate(ang, axis);
        G4ThreeVector dir = rot*(detMinDir+(detXdir+detZdir)*0.5);
        G4cout<<isocenter + dir.unit()*(detY+7.5*cm)<<G4endl;
        fDet->SetDetector(rot, isocenter + dir.unit()*(-detY+7.5*cm));
    }
    else if(command == fIsoCmd){
        isocenter = fIsoCmd->GetNew3VectorValue(newValue);
        G4ThreeVector dir = rot*(detMinDir+(detXdir+detZdir)*0.5);
        fDet->SetDetector(rot, isocenter + dir.unit()*(detY+7.5*cm));
    }
}

