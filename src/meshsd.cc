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
//
/// \file MeshSD.cc
/// \brief Implementation of the MeshSD class

#include "meshsd.hh"
#include "G4HCofThisEvent.hh"
#include "G4Step.hh"
#include "G4ThreeVector.hh"
#include "G4SDManager.hh"
#include "G4ios.hh"
#include "G4SystemOfUnits.hh"
#include "G4ParticleTable.hh"

MeshSD::MeshSD(const G4String& name, G4int i, G4int j, G4int k, G4double cellVol)
 : G4VSensitiveDetector(name), ni(i), nj(j), nk(k)
{
  collectionName.insert("doseS");
  collectionName.insert("doseE");

  energyVec = {0,0.01,0.015,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.1,0.15,
               0.2,0.3,0.4,0.5,0.511,0.6,0.662,0.8,1};
  skinDvec = {0,2.62,1.29,0.789,0.425,0.302,0.258,0.249,0.257,0.276,0.333,
              0.524,0.732,1.04,1.26,1.44,1.46,1.61,1.70,1.91,2.18};
  lensDvec = {0,0.296,0.555,0.503,0.335,0.255,0.228,0.228,0.244,0.268,0.330,
              0.526,0.754,1.16,1.61,2.03,2.15,2.45,2.75,3.20,3.92};
  G4double coeff = 1E-12*(joule/kg)*cm2/cellVol;
  for(size_t i=0;i<energyVec.size();i++){
      energyVec[i] *= MeV;
      skinDvec[i] *= coeff;
      lensDvec[i] *= coeff;
  }
  gamma
    = G4ParticleTable::GetParticleTable()->FindParticle("gamma");
}

MeshSD::~MeshSD()
{
}

void MeshSD::Initialize(G4HCofThisEvent* hce)
{
  fHitsMapS = new G4THitsMap<G4double>(SensitiveDetectorName,
                                       GetCollectionName(0));
  fHitsMapE = new G4THitsMap<G4double>(SensitiveDetectorName,
                                       GetCollectionName(1));

  hce->AddHitsCollection(GetCollectionID(0), (G4VHitsCollection*)fHitsMapS);
  hce->AddHitsCollection(GetCollectionID(1), (G4VHitsCollection*)fHitsMapE);
}

G4bool MeshSD::ProcessHits(G4Step* step, G4TouchableHistory*)
{
  if(step->GetTrack()->GetParticleDefinition()!=gamma) return false;
  G4double length = step->GetStepLength();
  if ( length == 0. ) return FALSE;
  G4double energy = step->GetPreStepPoint()->GetKineticEnergy();
  if ( energy == 0. ) return FALSE;

  G4double skinD, lensD;
  CalculateDoses(energy, skinD, lensD);
  skinD *= length;
  lensD *= length;

  // Add values
  G4int k = step->GetPreStepPoint()->GetTouchable()->GetCopyNumber(0);
  G4int j = step->GetPreStepPoint()->GetTouchable()->GetCopyNumber(1);
  G4int i = step->GetPreStepPoint()->GetTouchable()->GetCopyNumber(2);
  G4int idx = i*nk*nj+j*nk+k;
  fHitsMapS->add(idx, skinD);
  fHitsMapE->add(idx, lensD);

  return true;
}

void MeshSD::EndOfEvent(G4HCofThisEvent*)
{
  if ( verboseLevel>1 ) {

  }
}

void MeshSD::CalculateDoses(G4double energy, G4double &skinDose, G4double &lensDose){
    for(size_t i=1;i<energyVec.size();i++){
        if(energy>energyVec[i]) continue;
        G4double intpltn = (energy-energyVec[i-1])/(energyVec[i]-energyVec[i-1]);
        skinDose = skinDvec[i-1] + (skinDvec[i]-skinDvec[i-1])*intpltn;
        lensDose = lensDvec[i-1] + (lensDvec[i]-lensDvec[i-1])*intpltn;
        return;
    }

    G4Exception("MeshSD::CalculateDoses", "", JustWarning,
                G4String(std::to_string(energy/MeV)+" MeV! Larger than "+std::to_string(energyVec.back()/MeV)+" MeV"));
    skinDose = skinDvec.back();
    lensDose = lensDvec.back();
}
