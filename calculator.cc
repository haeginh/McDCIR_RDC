#include "ClientSocket.h"
#include "SocketException.h"
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
    int port = 30303;
    auto* runManager =
            G4RunManagerFactory::CreateRunManager(G4RunManagerType::Default);
    G4UIExecutive* ui = 0;
    if(argc==2)  runManager->SetNumberOfThreads(atoi(argv[1]));
    else         ui = new G4UIExecutive(argc, argv);

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

    runManager->Initialize();

        try{

            string listenerIP("localhost"); int listenerPort(30303);
//            cout<<"Listener IP: "; cin>>listenerIP;
//            cout<<"Listener port: "; cin>>listenerPort;
            ClientSocket vis(listenerIP, listenerPort);

            G4String data;
            vis<<"v1_cal"; vis>>data;
            if(data.substr(0,3)=="chk")
                G4cout<<"Visualizer successfully connected!"<<G4endl;
            else
                G4cout<<"WARNING>> Wrong signal: "<<data<<G4endl;

            G4int ijkData[3];
            parallel->GetIJK(ijkData[0], ijkData[1], ijkData[2]);
            vis.SendIntBuffer(ijkData, 3);

            bool isFirst(true);
            while (true){
                G4bool cont(true);
                if(isFirst){
                    UImanager->ApplyCommand("/beam/spec 80kVp.txt");
                    isFirst = false;
                }else
                    while(cont){ //command (U-define: /beam/rot x y z)
                        G4cout<<"command>>"<<std::flush; char buff[100];
                        G4cin.getline(buff, sizeof(buff));
                        G4String command=buff;
                        if(command.back()!='\\') cont = false;
                        else command = command.substr(0, command.length()-1);
                        stringstream ss(command); G4String start; ss>>start;
                        if(start=="patient"){
                            G4ThreeVector pos, rot;
                            ss>>pos>>rot;
                            det->SetPatient(pos, rot);
                        }else if(command=="tracker"){
                            G4cout<<"Tracker rotation in vector: "<<std::flush;
                            char trackerBuff[100]; G4cin.getline(trackerBuff, sizeof(trackerBuff));
                            command = trackerBuff;
                            UImanager->ApplyCommand("/beam/rot "+command);
                            UImanager->ApplyCommand("/det/rot "+command);
                        }else UImanager->ApplyCommand(command);
                    }
                G4Timer timer; timer.Start();
                runManager->BeamOn(100000);
                timer.Stop(); G4cout<<"Run time: "<<timer.GetRealElapsed()<<"s"<<G4endl;
                timer.Start();
                const RunAction* runAction
                        = static_cast<const RunAction*>(runManager->GetUserRunAction());
                const std::map<G4int, std::pair<G4double, G4double>>* doseVec = runAction->GetDoseMap();
                double buff[180], buff2[180]; int buffInt[180];
                G4cout<<"Send "<<doseVec->size()<< " dose data..."<<std::flush;
                G4int numOfData = doseVec->size();
                G4int numOfPack = floor(numOfData / 180)+1;

                string dump; dump.clear();
                vis.SendIntBuffer(&numOfData,1);
                vis>>dump;
                auto iter = doseVec->begin();
                for(int i=0, n=0;i<numOfPack;i++){
                    for(int j=0;j<180 && n<numOfData;j++, n++, iter++){
                        buffInt[j] = iter->first;
                        buff[j] = iter->second.first;
                        buff2[j] = iter->second.second;
                    }
                    vis.SendIntBuffer(buffInt,180); vis>>dump;
                    vis.SendDoubleBuffer(buff,180); vis>>dump;
                    vis.SendDoubleBuffer(buff2,180);vis>>dump;
                }
                timer.Stop();
                G4cout<<timer.GetRealElapsed()<<"s"<<G4endl;
            }
        }
        catch ( SocketException& e )
        {
            std::cout << "Exception was caught:" << e.description() << "\n";
        }
//    }
//    catch ( SocketException& e )
//    {
//        std::cout << "Exception was caught:" << e.description() << "\n";
//    }

    //   delete runManager;
    return 0;
}

