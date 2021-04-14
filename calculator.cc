#include "ClientSocket.h"
#include "SocketException.h"
#include "ModelImport.hh"
#include <iostream>
#include <string>
#include <array>

//G4
#include "G4RunManagerFactory.hh"
#include "detectorconstruction.hh"
#include "actioninitialization.hh"
#include "runaction.hh"
#include "FTFP_BERT.hh"
#include "G4PhysListFactory.hh"
#include "G4StepLimiterPhysics.hh"

#include "G4VisExecutive.hh"
#include "G4UIExecutive.hh"
#include "G4UImanager.hh"
#include "G4Timer.hh"
using namespace std;
int main ( int argc, char** argv)
{
    auto* runManager =
      G4RunManagerFactory::CreateRunManager(G4RunManagerType::Default);
    G4UIExecutive* ui = 0;
    if(argc==2)  runManager->SetNumberOfThreads(atoi(argv[1]));
    else         ui = new G4UIExecutive(argc, argv);
    G4VModularPhysicsList* physicsList = new FTFP_BERT;
    physicsList->RegisterPhysics(new G4StepLimiterPhysics());
    runManager->SetUserInitialization(physicsList);

    G4VisManager* visManager = new G4VisExecutive("Quiet");
    visManager->Initialize();
    G4UImanager* UImanager = G4UImanager::GetUIpointer();

    try{
        ClientSocket* client_socket;
        client_socket = new ClientSocket ("166.104.155.142", 30303 );
        G4int calID;
        (*client_socket)<<"v2_cal"; client_socket->RecvIntBuffer(&calID,1);
        cout<<"Activated as Cal #"<<calID<<endl;

        ModelImport* modelImport = new ModelImport(client_socket);
        modelImport->RecvInitData();
        DetectorConstruction* det = new DetectorConstruction(modelImport, client_socket);

        runManager->SetUserInitialization(det);
        runManager->SetUserInitialization(new ActionInitialization(modelImport));

        if(ui){
            UImanager->ApplyCommand("/control/execute init_vis.mac");
            UImanager->ApplyCommand("/control/execute gui.mac");
            ui->SessionStart();
        }else{
           // UImanager->ApplyCommand("/tracking/verbose 2");
            runManager->Initialize();
            G4String phaseSpace = "/home/hurel/RemoteCal_cal/100kVp1E7_shield.bin";
            G4int num;
            std::ifstream ifs(phaseSpace, std::ios::in | std::ios::binary);
            ifs.read((char*)&num, sizeof(G4int)); ifs.close();
            G4cout<<"nps per frame: "<<num <<G4endl;
            UImanager->ApplyCommand("/beam/phase "+phaseSpace);
            runManager->BeamOn(1);
//            UImanager->ApplyCommand("/gun/particle gamma");
//            UImanager->ApplyCommand("/gun/energy 50 keV");
            const RunAction* runAction
              = static_cast<const RunAction*>(runManager->GetUserRunAction());
            G4int numOfPack = floor(modelImport->GetNumOfFace() / 180.)+1;
            while(true){
                array<double, 155> pack;
                client_socket->RecvDoubleBuffer(pack.data(),155);
                if(pack[0]==0){
                    if(pack[1]==1){
                        G4cout<<"Receive recorded data"<<endl;
                        (*client_socket)<<"chk";
                        MatrixXd calib_M(24,3);
                        client_socket->RecvDoubleBuffer(calib_M.data(),72);
                        (*client_socket)<<"chk";
                        modelImport->ResetCalibData(calib_M);
                        continue;
                    }
                    else if(pack[1]==-1){
                        (*client_socket)<<"chk";
                        G4cout<<"Finishing calculating recorded data"<<endl;
                        modelImport->ResetCalibData();
                        continue;
                    }
                }
                G4cout<<"Frame signal: "<<pack[0]<<G4endl;
                RotationList vQ;
                vector<Vector3d> vT;
                for(int i=0;i<22;i++){
                    vQ.push_back(Quaterniond(pack[4*i+1],pack[4*i+2],pack[4*i+3],pack[4*i+4]));
                    vT.push_back(Vector3d(pack[3*i+89], pack[3*i+90], pack[3*i+91])*cm);
                }
                bool calib(false);
                if(pack[0]<0) calib = true;
                det->SetUpNewFrame(vQ,vT, calib);
                runManager->GeometryHasBeenModified();
                runManager->BeamOn(num*0.001);
                const std::vector<G4Accumulable<G4double>*>* doseVec = runAction->GetDoseVec();
                double buff[180]; string dump;
                G4Timer timer; timer.Start();
                G4cout<<"Send dose data..."<<std::flush;
                for(int i=0, n=0;i<numOfPack;i++){
                    for(int j=0;j<180 && n<modelImport->GetNumOfFace();j++, n++)
                        buff[j] = (*doseVec)[n]->GetValue();
                    client_socket->SendDoubleBuffer(buff,180);
                    if(n<modelImport->GetNumOfFace())(*client_socket)>>dump;
                }timer.Stop();
                G4cout<<timer.GetRealElapsed()<<"s"<<G4endl;
            }
        }
    }
    catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\n";
    }

 //   delete runManager;
  return 0;
}

