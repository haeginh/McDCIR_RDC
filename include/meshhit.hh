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

#ifndef MeshHit_h
#define MeshHit_h 1

#include "G4VHit.hh"
#include "G4THitsCollection.hh"
#include "G4Allocator.hh"
#include "G4Threading.hh"

class MeshHit : public G4VHit
{
  public:
    MeshHit();
    MeshHit(const MeshHit&);
    virtual ~MeshHit();

    // operators
    const MeshHit& operator=(const MeshHit&);
    G4bool operator==(const MeshHit&) const;

    inline void* operator new(size_t);
    inline void  operator delete(void*);

    // methods from base class
    virtual void Draw() {}
    virtual void Print();

    // methods to handle data
    void Add(G4double skin, G4double lens);

    // get methods
    G4double GetSkinD() const;
    G4double GetLensD() const;

  private:
    G4double skinDose;
    G4double lensDose;
};


using MeshHitsCollection = G4THitsCollection<MeshHit>;

extern G4ThreadLocal G4Allocator<MeshHit>* MeshHitAllocator;

inline void* MeshHit::operator new(size_t)
{
  if (!MeshHitAllocator) {
    MeshHitAllocator = new G4Allocator<MeshHit>;
  }
  void *hit;
  hit = (void *) MeshHitAllocator->MallocSingle();
  return hit;
}

inline void MeshHit::operator delete(void *hit)
{
  if (!MeshHitAllocator) {
    MeshHitAllocator = new G4Allocator<MeshHit>;
  }
  MeshHitAllocator->FreeSingle((MeshHit*) hit);
}

inline void MeshHit::Add(G4double skin, G4double lens) {
  skinDose += skin;
  lensDose += lens;
}

inline G4double MeshHit::GetSkinD() const {
  return skinDose;
}

inline G4double MeshHit::GetLensD() const {
  return lensDose;
}


#endif
