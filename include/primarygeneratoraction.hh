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
#include "Randomize.hh"
#include "G4VPhysicalVolume.hh"
#include "G4GeometryManager.hh"
#include <vector>

class PrimaryMessenger;

class PrimaryGeneratorAction : public G4VUserPrimaryGeneratorAction
{
  public:
    PrimaryGeneratorAction();
    virtual ~PrimaryGeneratorAction();

  public:
    virtual void   GeneratePrimaries(G4Event* anEvent);

    void SetSource(G4RotationMatrix _rot){
        rot = _rot;
        fParticleGun->SetParticlePosition(rot*source + isocenter);
    }
    void SetSourceTrans(G4ThreeVector _trans) {isocenter = _trans;}
    G4ThreeVector SampleADirection(){
        return rot*(detMinDir+(detXdir*G4UniformRand()+detZdir*G4UniformRand()));
    }
    G4RotationMatrix GetRot(){return rot;}
    G4ThreeVector GetDetPos(){
        G4ThreeVector dir = rot*(detMinDir+(detXdir+detZdir)*0.5);
        return isocenter + dir.unit()*(detY+7.5*cm);
    }
    void SetSourceGen(std::vector<G4double> _randCDF,std::vector<G4double> _energySamples){
        randCDF = _randCDF; energySamples = _energySamples;
    }
    G4double SampleAnEnergySG(){
        G4double rand = G4UniformRand();
        for(size_t i=0;i<randCDF.size();i++){
            if(randCDF[i]<rand) continue;
            return energySamples[i];
        }
    }

  private:
    G4ParticleGun*       fParticleGun;
    PrimaryMessenger*    fMessenger;
    G4double cosTheta;
    G4RotationMatrix rot;
    G4ThreeVector source;
    G4double tranZ;
    G4double worldHalfZ;

    G4double detY;
    G4ThreeVector detMinDir, detXdir, detZdir;
    G4ThreeVector sourcePos, isocenter;

    //source Gen
    G4bool useSpec;
    std::vector<G4double> randCDF;
    std::vector<G4double> energySamples;
};

#endif

