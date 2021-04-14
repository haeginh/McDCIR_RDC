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

  energyVec = {0,0.002,0.005,0.01,0.015,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.1,0.15};
  skinDvec  = {0,2.96637,22.6288,7.18402,3.18922,1.85468,0.810165,0.488154,0.370343,0.345776,0.341135,0.367115,0.426512,0.682618};
  lensDvec  = {0,0.276,0.689,1.38,1.98,1.52,0.833,0.563,0.460,0.437,0.446,0.468,0.555,0.842};
  G4double coeff = 1E-12*(joule/kg)*cm2/cellVol;
  for(size_t i=0;i<energyVec.size();i++){
      energyVec[i] *= MeV;
      skinDvec[i] *= coeff;
      lensDvec[i] *= coeff;
  }
  gamma
    = G4ParticleTable::GetParticleTable()->FindParticle("gamma");

  G4ThreeVector start(-1.5*m+2.5*cm,-1.5*m+2.5*cm,0+2.5*cm);
  G4double unit = 5*cm;
  //manual
  for(G4int i=0;i<60;i++){
      for(G4int j=0;j<60;j++){
          for(G4int k=0;k<60;k++){
               G4int idx = i*nk*nj+j*nk+k;
               G4ThreeVector pos = start + unit*G4ThreeVector(i,j,k);
               if(pos.getZ()>90*cm) {blockedVox[idx]=false; continue;}
               if(pos.getY()<15*cm) {blockedVox[idx]=false; continue;}
               if(pos.getX()<-18*cm) {blockedVox[idx]=false; continue;}
               if(pos.getX()>47*cm) {blockedVox[idx]=false; continue;}
               blockedVox[idx] = true;
          }
      }
  }

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

  G4int k = step->GetPreStepPoint()->GetTouchable()->GetCopyNumber(0);
  G4int j = step->GetPreStepPoint()->GetTouchable()->GetCopyNumber(1);
  G4int i = step->GetPreStepPoint()->GetTouchable()->GetCopyNumber(2);
  G4int idx = i*nk*nj+j*nk+k;
  if(blockedVox[idx]) return false;

  G4double length = step->GetStepLength();
  if ( length == 0. ) return FALSE;
  G4double energy = step->GetPreStepPoint()->GetKineticEnergy();
  if ( energy == 0. ) return FALSE;

  G4double skinD, lensD;
  CalculateDoses(energy, skinD, lensD);
  skinD *= length;
  lensD *= length;

  // Add values
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
