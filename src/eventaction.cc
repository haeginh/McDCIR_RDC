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

#include "eventaction.hh"
#include "runaction.hh"

#include "G4Event.hh"
#include "G4SDManager.hh"
#include "G4HCofThisEvent.hh"
#include "G4THitsMap.hh"

EventAction::EventAction(RunAction* runAction, ModelImport* _tetmodel)
 : G4UserEventAction(),
   fRunAction(runAction), tetmodel(_tetmodel), fCollID(-1), nonTargetNo(-1)
{}

EventAction::~EventAction()
{ }

void EventAction::BeginOfEventAction(const G4Event*)
{ }

void EventAction::EndOfEventAction(const G4Event* evt )
{
   //Hits collections
  G4HCofThisEvent* HCE = evt->GetHCofThisEvent();
  if(!HCE) return;

  // Get hits collections IDs
  if (fCollID< 0) {
    G4SDManager* SDMan = G4SDManager::GetSDMpointer();
    fCollID  = SDMan->GetCollectionID("phantomSD/edep");
    nonTargetNo = tetmodel->GetNonTargetNum();
    G4double divide3 = 1./3.;
    for(int i=nonTargetNo; i<tetmodel->GetNumOfTet();i++){
        skinIDs[i] = floor((double)(i-nonTargetNo) * divide3);
    }
  }

  G4THitsMap<G4double>* evtMap =
                     (G4THitsMap<G4double>*)(HCE->GetHC(fCollID));

  std::map<G4int,G4double*>::iterator itr;
  bool first(true);
  for (itr = evtMap->GetMap()->begin(); itr != evtMap->GetMap()->end(); itr++) {
    G4int copyNb  = (itr->first);
    if(copyNb<nonTargetNo) continue;
    G4double edep = *(itr->second);
    fRunAction->SumDose(skinIDs[copyNb], edep);
    if(first){
      fRunAction->CountEvent();
      first = false;
    }
  }
}
