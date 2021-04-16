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

#include "runaction.hh"
#include "primarygeneratoraction.hh"

#include "G4RunManager.hh"
#include "G4Run.hh"
#include "G4AccumulableManager.hh"
#include "G4UnitsTable.hh"
#include "G4SystemOfUnits.hh"
#include "G4NistManager.hh"

RunAction::RunAction()
 : G4UserRunAction()
{
    const G4double milligray = 1.e-3*gray;
    const G4double microgray = 1.e-6*gray;
    const G4double nanogray  = 1.e-9*gray;
    const G4double picogray  = 1.e-12*gray;

    new G4UnitDefinition("milligray", "milliGy" , "Dose", milligray);
    new G4UnitDefinition("microgray", "microGy" , "Dose", microgray);
    new G4UnitDefinition("nanogray" , "nanoGy"  , "Dose", nanogray);
    new G4UnitDefinition("picogray" , "picoGy"  , "Dose", picogray);
}

RunAction::~RunAction()
{}

G4Run* RunAction::GenerateRun()
{
    // generate run
    fRun = new Run();
    return fRun;
}

void RunAction::BeginOfRunAction(const G4Run* run)
{
  G4cout << "### Run " << run->GetRunID() << " start." << G4endl;
  nps=run->GetNumberOfEventToBeProcessed();
  G4RunManager::GetRunManager()->SetPrintProgress(int(nps*0.1));
}

void RunAction::EndOfRunAction(const G4Run* )
{
    if(!IsMaster()) return;
    auto doseMapS=fRun->GetDoseMapS();
    auto doseMapL=fRun->GetDoseMapL();
    std::vector<G4double> resultMap(doseMapS->size());
    std::ofstream ofs("./doseMaps/"+std::to_string(fRun->GetRunID())+".map", std::ios::binary);
    std::transform(doseMapS->begin(), doseMapS->end(), resultMap.begin(), [&](G4double d)->G4double{return d/(G4double)nps/gray;});
    ofs.write((char*) (&resultMap[0]), 60*60*60*sizeof(G4double));
    std::transform(doseMapL->begin(), doseMapL->end(), resultMap.begin(), [&](G4double d)->G4double{return d/(G4double)nps/gray;});
    ofs.write((char*) (&resultMap[0]), 60*60*60*sizeof(G4double));
    ofs.close();
    //dose map unit: (/cm2)
//    G4cout<<"dose of DAP meter: "<<G4BestUnit(fRun->GetDap()/nps,"Dose")<<G4endl;
}
