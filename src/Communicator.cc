#include "Communicator.hh"
#include "PhantomAnimator.hh"
#include "RDCWindow.hh"

Communicator &Communicator::Instance()
{
    static Communicator Communicator;
    return Communicator;
}

bool Communicator::StartServer(int port)
{
    serverPORT = port;
    struct sockaddr_in serverAddress;
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(port);
    // server socket generation
    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) // SOCK_DGRAM: UDP
    {
        cout << "Server: cannot open socket" << endl;
        return false;
    }
    else
        cout << "Server: open socket -> success!" << endl;
    struct timeval optVal = {0, 50};
    int optLen = sizeof(optVal);
    setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &optVal, optLen);
    // bind
    if (bind(server_fd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    {
        cout << "Server: cannot bind local address" << endl;
        return false;
    }
    else
        cout << "Server: bind local address -> success!" << endl;
    isListening = true;
    cout << "Server: start listening" << endl;
    listening = thread([this]()
                       {
            igl::Timer timer;
            int client_fd;
            struct sockaddr_in clientAddress;
            socklen_t client_addr_size;
            ssize_t receivedBytes;
            int server_fd = this->server_fd;
            int graceHalt(0);
            float readBuff[500];
            char* msg="stop";
            while(this->isListening||graceHalt++<this->GetWorkerNum()*10){
                timer.start();
                struct sockaddr_in connectSocket;
                socklen_t connectSocketLength = sizeof(connectSocket);
                getpeername(client_fd, (struct sockaddr*)&clientAddress, &connectSocketLength);
                // char clientIP[sizeof(clientAddress.sin_addr)+1] = {0};
                receivedBytes = recvfrom(server_fd, readBuff, 500*4, 0, (struct sockaddr*)&clientAddress, &client_addr_size);
                if(receivedBytes>0){
                    int id = (int)readBuff[0];
                    for(int i=0;i<500;i++) ofs<<readBuff[i]<<"\t";
         
                    if(!this->isListening||this->workerData.find(id)==this->workerData.end()){
                        sendto(server_fd, msg, 4, 0, (struct  sockaddr*)&clientAddress, sizeof(clientAddress));
                        lastStamp.erase(id);
                    }
                    else if(this->lastStamp.find(id)==this->lastStamp.end())//for the first time
                    {
                        // if(this->GetSocketOpt(id)<0) sendto(server_fd, msg, 4, 0, (struct  sockaddr*)&clientAddress, sizeof(clientAddress));
                        cout<<"WORKER"<<id<<": hello!"<<endl;
                        Affine3d aff = Affine3d::Identity();
                        aff.rotate(Quaterniond(readBuff[5], readBuff[2], readBuff[3], readBuff[4]).normalized().toRotationMatrix().transpose());
                        aff.translate(-Vector3d(readBuff[6], readBuff[7], readBuff[8]));
                        lastStamp[id] = clock();
                        get<2>(workerData[id]) = aff;
                        // workerData[id] = WORKER(string(inet_ntoa(clientAddress.sin_addr)), (int)readBuff[1], aff);
                    }
                    else
                    {
                        Affine3d glassAff = Affine3d::Identity();
                        map<int, Body> bodyMap;
                        // MatrixXd jointC(24, 3);
                        // RotationList posture(18);
                        lastStamp[id] = clock();
                        int pos(1);
                        int opt = readBuff[pos++];
                        if(opt & 2)
                        {
                        }
                        if(opt & 4)
                        {   
                            Quaterniond q(readBuff[pos++], readBuff[pos++], readBuff[pos++], readBuff[pos++]);
                            Vector3d t(readBuff[pos++], readBuff[pos++], readBuff[pos++]);
                            glassAff.translate(t);
                            glassAff.rotate(q);
                            glassAff = get<2>(workerData[readBuff[0]]) * glassAff;
                        }
                        if(opt & 8)
                        {
                            int num = readBuff[pos++];
                            for(int body=0;body<num;body++)
                            {
                                int bodyID = readBuff[pos++];
                                Body bodyStruct;
                                for(int i=0;i<bodyStruct.posture.size();i++) 
                                    bodyStruct.posture[i] = get<2>(workerData[readBuff[0]]).rotation() *
                                                            Quaterniond(readBuff[pos++], readBuff[pos++], readBuff[pos++], readBuff[pos++]); //*   alignRot[i];
                                for(int i=0;i<24;i++)
                                {
                                    bodyStruct.jointC(i, 0) = readBuff[pos++];
                                    bodyStruct.jointC(i, 1) = readBuff[pos++];
                                    bodyStruct.jointC(i, 2) = readBuff[pos++];
                                    // jointC(i, 0) = readBuff[pos++];
                                    // jointC(i, 1) = readBuff[pos++];
                                    // jointC(i, 2) = readBuff[pos++];
                                }
                                bodyStruct.jointC = (bodyStruct.jointC.rowwise().homogeneous()*get<2>(workerData[readBuff[0]]).matrix().transpose()).rowwise().hnormalized();
                                bodyMap[bodyID] = bodyStruct;
                            }
                        }

                        //change frame data -> TODO: set mutex!!
                        if(opt&4) current.glass_aff = glassAff;
                        if(opt&8){
                            //add new frame only if posture data is IN.
                            RDCWindow::Instance().postureUpdated = true;
                            current.bodyMap = bodyMap;
                        }
                    }                               
                }
                timer.stop();
                double time = timer.getElapsedTime();
                if(time<0.05) sleep(0.05-time);
            } });

    return true;
}
// bool Communicator::EstablishConnection(string ip, int port)
// {
// char readBuff[500];
// char sendBuff[500];
// struct sockaddr_in serverAddress, clientAddress;
// int server_fd, client_fd;
// socklen_t client_addr_size;
// ssize_t receivedBytes;
// ssize_t sentBytes;

// socklen_t clientAddressLength = 0;
// memset(&serverAddress, 0, sizeof(serverAddress));
// memset(&clientAddress, 0, sizeof(clientAddress));

// serverAddress.sin_family = AF_INET;
// serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
// serverAddress.sin_port = htons(port);

// //server socket generation
// if((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) //SOCK_DGRAM: UDP
// {
//     cout<<"Server: cannot open socket"<<endl;
// }
// //bind
// if(bind(server_fd, (struct sockaddr *)&serverAddress, sizeof(serverAddress))<0)
// {
//     cout<<"Server: cannot bind local address"<<endl;
// }

// while(1)
// {
//     //check client IP
//     struct sockaddr_in connectSocket;
//     socklen_t connectSocketLength = sizeof(connectSocket);
//     getpeername(client_fd, (struct sockaddr*)&clientAddress, &connectSocketLength);
//     // char clientIP
//     client_addr_size = sizeof(clientAddress);
//     receivedBytes = recvfrom(server_fd, readBuff, 500, 0, (struct sockaddr*)&clientAddress, &client_addr_size);

// }
/* TCP/IP
//bitwise operation (motion, glass, bed, c-arm): ex) motion + bed = 10
fd_set tmp;
server = new ServerSocket(port);
FD_ZERO(&readfds);
FD_SET(server->GetSocket(), &readfds);
max_sd = server->GetSocket();

//read remote program info.
ifstream ifs("RemoteRun.txt");
vector<pair<string, int>> clients;
while (!ifs.eof())
{
    string dump;
    getline(ifs, dump);
    if (dump.substr(0, 1) == "#" || dump.empty())
        continue;
    stringstream ss(dump);
    string client;
    int option;
    ss >> client >> option;
    clients.push_back(make_pair(client, option));
    if(option<0) system(("ssh " + client + " \"3_ocr "+ip+" "+to_string(port)+"\" &").c_str());
    else system(("ssh " + client + " \"2_tracker "+ip+" "+to_string(port)+" " + to_string(option) + "\" &").c_str());
}

cout << "number of sockets to connect: " << clients.size() << endl;
array<double, 155> pack;
while (client_sockets.size() < clients.size())
{
    tmp = readfds;
    sel_timeout.tv_sec = 1;
    sel_timeout.tv_usec = 0;
    int activity = select(max_sd + 1, &tmp, NULL, NULL, &sel_timeout);
    if (activity < 0 && errno != EINTR)
        cout << "" << flush;
    if (FD_ISSET(server->GetSocket(), &tmp))
    {
        ServerSocket *_sock = new ServerSocket();
        server->accept(*_sock);
        int sd = _sock->GetSocket();
        FD_SET(sd, &readfds);
        if (sd > max_sd)
            max_sd = sd;

        cout << "new connection from "
             << inet_ntoa(_sock->GetAdrrInfo().sin_addr)
             << " (sock #" << sd << ") / opt: ";
        (*_sock) << "welcome socket #" + to_string(sd) + "!";

        _sock->RecvDoubleBuffer(pack.data(), 8);
        int tracking_id = (int)pack[0];
        sock_opts[sd] = tracking_id;
        client_sockets[sd] = _sock;
        if (tracking_id<0)
        {
            cout<<"ocr"<<endl;
            // string str;
            // (*_sock)>>str;
            // stringstream ss(str);
            // for(int i=0;i<pack[1];i++)
            // {
            //     ss>>str;
            //     labels.push_back(str);
            // }
            values.resize(pack[1], 0.f);
        }
        else
        {
            if (tracking_id & 8)
                cout << "motion ";
            if (tracking_id & 4)
                cout << "glass ";
            if (tracking_id & 2)
                cout << "bed ";
            if (tracking_id & 1)
                cout << "c-arm ";

            Affine3d aff = Affine3d::Identity();
            aff.rotate(Quaterniond(pack[4], pack[1], pack[2], pack[3]).normalized().toRotationMatrix().transpose());
            aff.translate(-Vector3d(pack[5], pack[6], pack[7]));
            sock_coord[sd] = aff;
            cout << endl;

            if (tracking_id & 8)
            {
                cout << "-> Sending alignRot/BE data.." << flush;
                _sock->SendDoubleBuffer(initPack.data(), BE_ROWS * 6);
                cout << "success" << endl;
            }

            string msg;
            *_sock >> msg;
        }
    }
}
*/
// }

void Communicator::InitializeDataSet()
{
    current.glass_aff = Affine3d::Identity();
    // current.posture.resize(BE_ROWS);
    // current.jointC.resize(24, 3);
    // current.bodyIn = false;
}

Communicator::~Communicator()
{
    if (isListening)
        CloseServer();

    stopSignal = true;
    mainLoop.join();

    // for (auto client : client_sockets)
    //     delete client.second;
    // delete server;
}

void Communicator::SetInitPack(RotationList vQ, MatrixXi BE)
{
    for (int i = 0; i < BE_ROWS; i++)
    {
        initPack[6 * i] = vQ[i].w();
        initPack[6 * i + 1] = vQ[i].x();
        initPack[6 * i + 2] = vQ[i].y();
        initPack[6 * i + 3] = vQ[i].z();
        initPack[6 * i + 4] = BE(i, 0);
        initPack[6 * i + 5] = BE(i, 1);
    }
}

// void Communicator::Initialization()
// {

// }

void Communicator::BodyCalibration(int sock_id, string newProfile, PhantomAnimator *phantom)
{
    int signal(-1); // signal for calib.
    // client_sockets[sock_id]->SendIntBuffer(&signal, 1);
    cout << "Start calibration in socket #" << sock_id << endl;

    array<double, 155> pack;
    // client_sockets[sock_id]->RecvDoubleBuffer(pack.data(), 155);
    cout << "Get calibration data.." << flush;
    map<int, double> calibLengths;
    Vector3d eyeL_pos(0, 0, 0), eyeR_pos(0, 0, 0);
    int i = 0;
    for (; i < 155; i += 2)
    {
        if (initPack[i] < 0)
            break;
        calibLengths[initPack[i]] = initPack[i + 1];
    }
    eyeL_pos = Vector3d(initPack[i + 1], initPack[i + 2], initPack[i + 3]);
    eyeR_pos = Vector3d(initPack[i + 4], initPack[i + 5], initPack[i + 6]);
    int calibFrame = initPack[i + 7];
    cout << "done" << endl;

    for (i = 0; i < BE_ROWS; i++)
    {
        if (calibLengths.find(i) == calibLengths.end())
            continue;
        calibLengths[i] /= (double)calibFrame * 10;
    }
    eyeR_pos /= (double)calibFrame * 10;
    eyeL_pos /= (double)calibFrame * 10;

    phantom->AddProfile(calibLengths, eyeL_pos, eyeR_pos, newProfile);
    phantom->WriteProfileData("profile.txt");
    // (*client_sockets[sock_id]) << "Profile data for " + string(newProfile) + " was successfully transmitted!";
}

void Communicator::StartMainLoop()
{
    // for (auto client : client_sockets)
    // {
    //     string msg;
    //     (*client.second) >> msg;
    //     cout << msg << endl;
    //     int signal(1);
    //     client.second->SendIntBuffer(&signal, 1);
    // }

    // fd_set tmp;
    // array<float, 375> pack;
    // int dataNum = (24 + BE_ROWS) * 4;
    // mainLoop = thread([&]()
    //                   {
    //     while(1)
    //     {
    //         if(stopSignal) break;
    //         fd_set tmp = readfds;
    //         sel_timeout.tv_sec = 1;
    //         sel_timeout.tv_usec = 0;
    //         int activity = select(max_sd + 1, &tmp, NULL, NULL, &sel_timeout);
    //         //exceptions
    //         //if (activity < 0 && errno != EINTR) continue;
    //         if (FD_ISSET(server->GetSocket(), &tmp))
    //         {
    //             ServerSocket _sock;
    //             server->accept(_sock);
    //             _sock << "connection refused!";
    //         }

    //         for (auto sid : sock_opts)
    //         {
    //             if (!FD_ISSET(sid.first, &tmp))
    //                 continue;
    //             array<float, 375> pack;
    //             recv(sid.first, (void*) pack.data(), 375*4, 0);

    //             int capture_opt = pack[374];
    //             int signal(1);
    //             send(sid.first, (void*) &signal, 4, 0);

    //             if (capture_opt < 0) //ocr - diff. protocol
    //             {
    //                 // m.lock();
    //                 for(int i=0;i<values.size();i++) values[i] = pack[i];
    //                 // m.unlock();
    //                 continue;
    //             }
    //             if (capture_opt & 2) //bed
    //             {
    //             }
    //             if (capture_opt & 4) //glass
    //             {
    //                 Quaterniond q;
    //                 Vector3d t;
    //                 q.x() = pack[0]; q.y() = pack[1]; q.z() = pack[2]; q.w() = pack[3];
    //                 t(0) = pack[4]; t(1) = pack[5]; t(2) = pack[6];

    //                 Affine3d aff = Affine3d::Identity();
    //                 aff.translate(t);
    //                 aff.rotate(q);
    //                 current.glass_aff = sock_coord[sid.first] * aff;
    //             }
    //             if (capture_opt & 8) //motion (7-)
    //             {
    //                 current.bodyIn = true;
    //                 MatrixXd tmp(24, 3);
    //                 for(int i=0;i<24;i++)
    //                 {
    //                     //if(!pack[7+i]) continue; --> do not care reliability (in case particular joint is not reliable forever)
    //                     int n=31+i*3;
    //                     tmp(i,0) = pack[n];
    //                     tmp(i,1) = pack[n+1];
    //                     tmp(i,2) = pack[n+2];
    //                 }
    //                 current.jointC = (tmp.rowwise().homogeneous()*sock_coord[sid.first].matrix().transpose()).rowwise().hnormalized();
    //                 if(record) ofs<<current.jointC.row(0)<<" ";
    //                 for(int i=0;i<BE_ROWS;i++)
    //                 {
    //                     int n=103+i*4;
    //                     current.posture[i] = sock_coord[sid.first].rotation() * Quaterniond(pack[n], pack[n + 1], pack[n + 2], pack[n + 3]);
    //                     if(record) ofs<<current.posture[i].w()<<" "<<current.posture[i].x()<<" "<<current.posture[i].y()<<" "<<current.posture[i].z()<<" ";
    //                 }
    //                 if(record) ofs<<endl;
    //             }
    //             if (sid.second & 1) //c-arm
    //             {
    //             }
    //         }
    //     } });
    return;
}