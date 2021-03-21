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
#include "G4UIcmdWith3Vector.hh"
#include "G4UIcmdWith3VectorAndUnit.hh"
#include "G4UIcmdWithADoubleAndUnit.hh"
#include "G4UIcmdWithoutParameter.hh"
#include "G4UIcmdWithAString.hh"

#include "primarymessenger.hh"
#include "G4GeometryManager.hh"

PrimaryMessenger::PrimaryMessenger(PrimaryGeneratorAction* _primary)
:G4UImessenger(), fPrimary(_primary)
{
    fBeamDir = new G4UIdirectory("/beam/");

    fRotCmd = new G4UIcmdWith3Vector("/beam/rot", this);
    fRotCmd->SetGuidance("unit axis vector * angle in degree");

    fTransCmd = new G4UIcmdWith3VectorAndUnit("/beam/isoC", this);
    fTransCmd->SetDefaultUnit("cm");

    fSpecCmd = new G4UIcmdWithAString("/beam/spec", this);
}

PrimaryMessenger::~PrimaryMessenger() {
    delete fBeamDir;
    delete fRotCmd;
    delete fTransCmd;
    delete fSpecCmd;
}

void PrimaryMessenger::SetNewValue(G4UIcommand* command, G4String newValue)
{
    if(command == fRotCmd){
        G4ThreeVector vec = fRotCmd->GetNew3VectorValue(newValue);
        double ang = vec.mag() * deg;
        G4ThreeVector axis = vec.unit();
        rot = G4RotationMatrix();
        rot.rotate(ang, axis);
        fPrimary->SetSource(rot);
    }
    else if(command == fTransCmd){
        fPrimary->SetSourceTrans(fTransCmd->GetNew3VectorValue(newValue));
    }
    else if(command == fSpecCmd){
        std::ifstream ifs(newValue);
        G4double e, pdf;
        std::vector<std::pair<G4double, G4double>> sourceGen;
        while(ifs>>e>>pdf){
            sourceGen.push_back(std::make_pair(pdf, e));
        }
        std::sort(sourceGen.begin(),sourceGen.end(),std::greater<>());
        std::vector<G4double> randCDF, energySamples;
        for(auto iter:sourceGen){
            if(randCDF.empty()) randCDF.push_back(iter.first);
            else randCDF.push_back(iter.first+randCDF.back());
            energySamples.push_back(iter.second*keV);
        }
        G4double sum = randCDF.back();
        for(G4double &rands:randCDF) rands/=sum;
        fPrimary->SetSourceGen(randCDF, energySamples);
    }
}

