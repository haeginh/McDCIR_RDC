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
/// \file electromagnetic/TestEm5/src/StackingAction.cc
/// \brief Implementation of the StackingAction class
//
// $Id: StackingAction.cc 76464 2013-11-11 10:22:56Z gcosmo $
//
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

#include "stackingaction.hh"
#include "G4Track.hh"
#include "G4Electron.hh"
#include "G4SDManager.hh"
#include "G4THitsMap.hh"
#include "G4Event.hh"
#include "G4EventManager.hh"

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

StackingAction::StackingAction()
{
    totalEnergyDepositID = G4SDManager::GetSDMpointer()->GetCollectionID("dap/dose");
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

StackingAction::~StackingAction()
{
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

G4ClassificationOfNewTrack
StackingAction::ClassifyNewTrack(const G4Track* aTrack)
{
    if (aTrack->GetParentID() == 0) return fUrgent;
    //if (aTrack->GetDefinition() != G4Electron::Electron()) return fUrgent;
//    const G4Event* aEvent = G4EventManager::GetEventManager()->GetConstCurrentEvent();
    G4HCofThisEvent* HCE = G4EventManager::GetEventManager()->GetConstCurrentEvent()->GetHCofThisEvent();
    if (!HCE) return fUrgent;

    G4int CopyNo = aTrack->GetTouchable()->GetCopyNumber();

    if(CopyNo==1000){
        G4HCofThisEvent* HCE = G4EventManager::GetEventManager()->GetConstCurrentEvent()->GetHCofThisEvent();
        totalEnergyDepositID = G4SDManager::GetSDMpointer()->GetCollectionID("meshSD/doseS");
        G4THitsMap<G4double>* eventTotalEnergyDeposit= (G4THitsMap<G4double>*)(HCE->GetHC(totalEnergyDepositID));
        G4double edep = aTrack->GetKineticEnergy();
        eventTotalEnergyDeposit->add(CopyNo,edep);
    }
    return fKill ;
}

