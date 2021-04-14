#include "ModelImport.hh"
#include <iostream>
#include <string>
#include <array>

//G4
#include "G4RunManagerFactory.hh"
#include "detectorconstruction.hh"
#include "parallelmesh.hh"
#include "actioninitialization.hh"
#include "runaction.hh"
#include "FTFP_BERT.hh"
#include "G4PhysListFactory.hh"
#include "G4StepLimiterPhysics.hh"
#include "G4ParallelWorldPhysics.hh"

#include "G4VisExecutive.hh"
#include "G4UIExecutive.hh"
#include "G4UImanager.hh"
#include "G4Timer.hh"
using namespace std;
int main ( int argc, char** argv)
{
    // Choose the Random engine
    //
    G4Random::setTheEngine(new CLHEP::RanecuEngine);
    G4Random::setTheSeed(time(0));

    auto* runManager =
            G4RunManagerFactory::CreateRunManager(G4RunManagerType::Default);
    G4UIExecutive* ui = 0;
    if(argc==1)   ui = new G4UIExecutive(argc, argv);


    G4VisManager* visManager = new G4VisExecutive("Quiet");
    visManager->Initialize();
    G4UImanager* UImanager = G4UImanager::GetUIpointer();
    //generate socket for each source
    DetectorConstruction* det = new DetectorConstruction(new ModelImport());
    auto parallel = new ParallelMesh("MeshTally");
    det->RegisterParallelWorld(parallel);
    runManager->SetUserInitialization(det);
    G4VModularPhysicsList* physicsList = new FTFP_BERT;
    //    physicsList->RegisterPhysics(new G4StepLimiterPhysics());
    physicsList->RegisterPhysics(new G4ParallelWorldPhysics("MeshTally"));
    runManager->SetUserInitialization(physicsList);
    runManager->SetUserInitialization(new ActionInitialization());

    if(ui){
        UImanager->ApplyCommand("/control/execute init_vis.mac");
        UImanager->ApplyCommand("/control/execute gui.mac");
        ui->SessionStart();
    }
    else{
        UImanager->ApplyCommand("/control/execute " + G4String(argv[1]));
    }

    delete runManager;
    delete visManager;
    return 0;
}

