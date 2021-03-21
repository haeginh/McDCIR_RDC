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

#include "run.hh"

Run::Run()
:G4Run(), dap(0)
{
    fCollID_skin
    = G4SDManager::GetSDMpointer()->GetCollectionID("meshSD/doseS");
    fCollID_lens
    = G4SDManager::GetSDMpointer()->GetCollectionID("meshSD/doseE");
    fCollID_dap
    = G4SDManager::GetSDMpointer()->GetCollectionID("dap/dose");
}

Run::~Run()
{}

void Run::RecordEvent(const G4Event* event)
{
    // Hits collections
    //
    G4HCofThisEvent* HCE = event->GetHCofThisEvent();
    if(!HCE) return;

    auto doseMapS =
            *static_cast<G4THitsMap<G4double>*>(HCE->GetHC(fCollID_skin))->GetMap();
    auto doseMapE =
            *static_cast<G4THitsMap<G4double>*>(HCE->GetHC(fCollID_lens))->GetMap();

    for(auto itr:doseMapS){
        doseMap[itr.first].first += *itr.second;
        doseMap[itr.first].second += *doseMapE[itr.first];
    }
    auto dapMap =
            *static_cast<G4THitsMap<G4double>*>(HCE->GetHC(fCollID_dap))->GetMap();
    if(dapMap.find(1000)!=dapMap.end()) dap += *dapMap[1000];

}

void Run::Merge(const G4Run* run)
{
    const Run* localRun = static_cast<const Run*>(run);
    // merge the data from each thread
    auto localMap = localRun->doseMap;

    for(auto itr : localMap){
        doseMap[itr.first].first  += itr.second.first;
        doseMap[itr.first].second += itr.second.second;
    }
    dap += localRun->dap;
    G4Run::Merge(run);
}
