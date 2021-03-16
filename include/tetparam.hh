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
// TetParam.hh
// \file   MRCP_GEANT4/External/include/TetParam.hh
// \author Haegin Han
//

#ifndef TetParam_h
#define TetParam_h 1

#include "ModelImport.hh"

#include "globals.hh"
#include "G4VPVParameterisation.hh"
#include "G4VSolid.hh"
#include "G4Material.hh"
#include "G4VisAttributes.hh"

#include <map>

class G4VPhysicalVolume;

class TetParam : public G4VPVParameterisation
{
  public:
    TetParam(ModelImport* tetData);
    virtual ~TetParam();

    virtual G4VSolid* ComputeSolid(
                   const G4int copyNo, G4VPhysicalVolume* );

    virtual void ComputeTransformation(
                   const G4int,G4VPhysicalVolume*) const;

    virtual G4Material* ComputeMaterial(const G4int copyNo,
                                        G4VPhysicalVolume* phy,
                                        const G4VTouchable*);

  private:
    ModelImport*                       tetData;
};

#endif
