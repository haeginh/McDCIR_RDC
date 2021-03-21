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
// TETPrimaryGeneratorAction.cc
// \file   MRCP_GEANT4/External/src/TETPrimaryGeneratorAction.cc
// \author Haegin Han
// \update
// \


#include "primarygeneratoraction.hh"
#include "G4ParticleTable.hh"
#include "G4ParticleDefinition.hh"
#include "G4RunManager.hh"
#include "detectorconstruction.hh"
#include "G4PhysicalVolumeStore.hh"

PrimaryGeneratorAction::PrimaryGeneratorAction()
    :worldHalfZ(2*m), useSpec(false) // supposed to be get from det.

{
    fParticleGun = new G4ParticleGun(1);
    fMessenger   = new PrimaryMessenger(this);

    source = G4ThreeVector(0,81,0)*cm;
    isocenter = G4ThreeVector(0,0,60)*cm;
    SetSource(rot);
    G4ParticleDefinition* gamma
      = G4ParticleTable::GetParticleTable()->FindParticle("gamma");
    fParticleGun->SetParticleDefinition(gamma);
    fParticleGun->SetParticleEnergy(50*keV);

    detY = -35.3*cm;
    detMinDir = G4ThreeVector(-30.61*cm*0.5,detY,-39.54*cm*0.5)-source;
    detXdir = G4ThreeVector(30.61*cm, 0, 0);
    detZdir = G4ThreeVector(0, 0, 39.54*cm);
}

PrimaryGeneratorAction::~PrimaryGeneratorAction()
{
    delete fParticleGun;
    delete fMessenger;
}

void PrimaryGeneratorAction::GeneratePrimaries(G4Event* anEvent)
{
    if(useSpec)
        fParticleGun->SetParticleEnergy(SampleAnEnergySG());
    fParticleGun->SetParticleMomentumDirection(SampleADirection());
    fParticleGun->GeneratePrimaryVertex(anEvent);
}


