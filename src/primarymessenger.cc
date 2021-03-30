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
// TETPrimaryMessenger.cc
// \author Haegin Han
//

#include "G4UIdirectory.hh"
//#include "G4UIcmdWith3Vector.hh"
//#include "G4UIcmdWith3VectorAndUnit.hh"
//#include "G4UIcmdWithADoubleAndUnit.hh"
//#include "G4UIcmdWithoutParameter.hh"
#include "G4UIcmdWithAString.hh"

#include "primarymessenger.hh"

PrimaryMessenger::PrimaryMessenger(PrimaryGeneratorAction* _primary)
:G4UImessenger(), fPrimary(_primary)
{
    fBeamDir = new G4UIdirectory("/beam/");
    fPhaseSpace = new G4UIcmdWithAString("/beam/phase", this);

//    fRotCmd = new G4UIcmdWith3Vector("/beam/rot", this);
//    fRotCmd->SetGuidance("unit axis vector * angle in degree");

//    fSourceCmd = new G4UIcmdWith3VectorAndUnit("/beam/source", this);
//    fSourceCmd->SetDefaultUnit("cm");

//    fAngleCmd = new G4UIcmdWithADoubleAndUnit("/beam/angle", this);
//    fAngleCmd->SetDefaultUnit("deg");

//    fTransCmd = new G4UIcmdWith3VectorAndUnit("/beam/trans", this);
//    fTransCmd->SetDefaultUnit("cm");

//    fCloseCmd = new G4UIcmdWithoutParameter("/beam/close", this);
}

PrimaryMessenger::~PrimaryMessenger() {
    delete fBeamDir;
    delete fPhaseSpace;
//    delete fRotCmd;
//    delete fSourceCmd;
//    delete fAngleCmd;
//    delete fTransCmd;
//    delete fCloseCmd;
}

#include "G4ParticleTable.hh"
void PrimaryMessenger::SetNewValue(G4UIcommand* command, G4String newValue)
{
    if(command == fPhaseSpace){
        fPrimary->ReadPhaseSpace(newValue);
//        std::ifstream ifs(newValue, std::ios::in | std::ios::binary);
//        G4int num;
//        ifs.read((char*) &num, sizeof(G4int));
//        G4cout<<"Read data for "<<num<<" particles"<<G4endl;
//        particles.clear(); particles.resize(num);
//        energies.clear(); energies.resize(num);
//        positions.clear(); positions.resize(num);
//        directions.clear(); directions.resize(num);
//        ifs.read((char*) &particles[0], num*sizeof(G4int));
//        ifs.read((char*) &energies[0], num*sizeof(G4double));
//        ifs.read((char*) &positions[0], num*sizeof(G4ThreeVector));
//        ifs.read((char*) &directions[0], num*sizeof(G4ThreeVector));
//        ifs.close();

//        particleMap.clear();
//        std::vector<int> sorted(particles);
//        std::sort(sorted.begin(),sorted.end());
//        sorted.erase(std::unique(sorted.begin(),sorted.end()),sorted.end());
//        for(G4int i:sorted) particleMap[i]=G4ParticleTable::GetParticleTable()->FindParticle(i);
    }
    //    if(command == fRotCmd){
    //        G4ThreeVector vec = fRotCmd->GetNew3VectorValue(newValue);
    //        double ang = vec.mag() * deg;
    //        G4ThreeVector axis = vec.unit();
    //        rot = G4RotationMatrix();
    //        rot.rotate(ang, axis);
    //    }
    //    else if(command == fSourceCmd){
    //        fPrimary->SetDefaultSource(fSourceCmd->GetNew3VectorValue(newValue));
    //    }
    //    else if(command == fAngleCmd){
    //        fPrimary->SetAngle(fAngleCmd->GetNewDoubleValue(newValue));
    //    }
    //    else if(command == fTransCmd){
    //        trans = fTransCmd->GetNew3VectorValue(newValue);
    //    }
    //    else if(command == fCloseCmd){
    //        fPrimary->SetSource(rot,trans);
    //    }
}

