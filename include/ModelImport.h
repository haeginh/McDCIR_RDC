#ifndef MODELIMPORT_HH
#define MODELIMPORT_HH

#include "G4Tet.hh"

class ModelImport
{
public:
    ModelImport(G4String modelName);
    G4bool RecvInitData();

private:
};

#endif // MODELIMPORT_HH
