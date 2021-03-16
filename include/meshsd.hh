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
/// \file MeshSD.hh
/// \brief Definition of the MeshSD class

#ifndef MeshSD_h
#define MeshSD_h 1

#include "G4VSensitiveDetector.hh"

#include "G4THitsMap.hh"

#include <vector>

class G4Step;
class G4HCofThisEvent;

class MeshSD : public G4VSensitiveDetector
{
  public:
    MeshSD(const G4String& name, G4int i, G4int j, G4int k, G4double cellVol);
    virtual ~MeshSD();

    virtual void   Initialize(G4HCofThisEvent* hitCollection);
    virtual G4bool ProcessHits(G4Step* step, G4TouchableHistory* history);
    virtual void   EndOfEvent(G4HCofThisEvent* hitCollection);

  private:
    void CalculateDoses(G4double energy, G4double &skinDose, G4double &lensDose);

  private:
    G4THitsMap<G4double>* fHitsMapS;
    G4THitsMap<G4double>* fHitsMapE;
    G4int    ni, nj, nk;
    std::vector<G4double> energyVec;
    std::vector<G4double> skinDvec, lensDvec;
    G4ParticleDefinition* gamma;
};

#endif

