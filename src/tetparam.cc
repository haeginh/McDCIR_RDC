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
// TetParam.cc
// \file   MRCP_GEANT4/External/src/TetParam.cc
// \author Haegin Han
//

#include "tetparam.hh"
#include "G4LogicalVolume.hh"
#include "G4VisExecutive.hh"
#include "G4RunManager.hh"

TetParam::TetParam(ModelImport* _tetData)
: G4VPVParameterisation(), tetData(_tetData)
{
}

TetParam::~TetParam()
{}

G4VSolid* TetParam::ComputeSolid(
                   const G4int copyNo, G4VPhysicalVolume* )
{
    // return G4Tet*
    return tetData->GetTetrahedron(copyNo);
}

void TetParam::ComputeTransformation(
                   const G4int,G4VPhysicalVolume*) const
{}

//G4Material* TetParam::ComputeMaterial(const G4int copyNo,
//                                            G4VPhysicalVolume*,
//                                      const G4VTouchable* )
//{
//    return tetData->GetMaterial(tetData->GetMaterialIdx(copyNo));
//}


