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
// TETPrimaryGeneratorAction.hh
// \file   MRCP_GEANT4/External/include/TETPrimaryGeneratorAction.hh
// \author Haegin Han
//

#ifndef PrimaryGeneratorAction_h
#define PrimaryGeneratorAction_h 1

#include "G4VUserPrimaryGeneratorAction.hh"
#include "globals.hh"
#include "G4Event.hh"
#include "G4ParticleGun.hh"
#include "G4SystemOfUnits.hh"
#include "primarymessenger.hh"
#include "G4RotationMatrix.hh"
#include "G4RandomDirection.hh"
#include "G4ParticleTable.hh"
class PrimaryMessenger;

class PrimaryGeneratorAction : public G4VUserPrimaryGeneratorAction
{
  public:
    PrimaryGeneratorAction();
    virtual ~PrimaryGeneratorAction();

  public:
    virtual void   GeneratePrimaries(G4Event* anEvent);

    void ReadPhaseSpace(G4String file){
        std::ifstream ifs(file, std::ios::in | std::ios::binary);
        G4int num;
        ifs.read((char*) &num, sizeof(G4int));
        G4cout<<"Read data for "<<num<<" particles"<<G4endl;
        particles.clear(); particles.resize(num);
        energies.clear(); energies.resize(num);
        positions.clear(); positions.resize(num);
        directions.clear(); directions.resize(num);
        ifs.read((char*) &particles[0], num*sizeof(G4int));
        ifs.read((char*) &energies[0], num*sizeof(G4double));
        ifs.read((char*) &positions[0], num*sizeof(G4ThreeVector));
        ifs.read((char*) &directions[0], num*sizeof(G4ThreeVector));
        ifs.close();

        particleMap.clear();
        std::vector<int> sorted(particles);
        std::sort(sorted.begin(),sorted.end());
        sorted.erase(std::unique(sorted.begin(),sorted.end()),sorted.end());
        for(G4int i:sorted) particleMap[i]=G4ParticleTable::GetParticleTable()->FindParticle(i);
    }
//    void SetAngle(G4double angle){
//        cosTheta = cos(angle);
//    }
//    void SetDefaultSource(G4ThreeVector pos){
//        source = pos;
//    }
//    void SetSource(G4RotationMatrix _rot, G4ThreeVector trans){
//        rot = _rot;
//        fParticleGun->SetParticlePosition(rot*source + trans);
//    }
//    G4ThreeVector SampleADirection(){
//        return rot*G4RandomDirection(cosTheta);
//    }

  private:
    G4ParticleGun*       fParticleGun;
    PrimaryMessenger*    fMessenger;
    G4double cosTheta;

    std::vector<G4int> particles;
    std::map<G4int, G4ParticleDefinition*> particleMap;
    std::vector<G4double> energies;
    std::vector<G4ThreeVector> positions, directions;
//    G4RotationMatrix rot;
//    G4ThreeVector source;
};

#endif

