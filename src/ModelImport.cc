#include "ModelImport.hh"
#include "igl/per_face_normals.h"
#include "G4Timer.hh"
#include "G4SystemOfUnits.hh"
#include "G4NistManager.hh"

ModelImport::ModelImport(ClientSocket* _sock)
    :socket(_sock), nonTargetNum(0)
{
//    igl::readMESH(modelName+".mesh",V,T,F);
//    igl::readDMAT(modelName+".dmat",W);
    ReadMatFile("test");
}

G4bool ModelImport::RecvInitData(){
    G4Timer timer; timer.Start();
    G4String data; (*socket)>>data;
    std::stringstream ss(data);
    int vNum, tNum, fNum, eNum;
    ss>>data>>vNum>>tNum>>fNum>>eNum;
    if(data.substr(0,4)!="init") G4cout<<"Wrong signal: "<<data<<G4endl;
    G4cout<<data<<G4endl;
    G4cout<<"Get data for "<<vNum<<" vertices...";
    V.resize(vNum,3); Wj.resize(vNum,24);
    W.clear();
    G4double epsilon(1e-5);
    G4String chk("chk");
    (*socket)<<chk;
    for(int i=0;i<vNum;i++){
        array<double, 49> packet;
        socket->RecvDoubleBuffer(packet.data(),49);
        for(int j=0;j<3;j++) V(i,j) = packet[j];

        G4double sum(0);
        std::map<int, double> W_map;
        for(G4int j=3;j<25;j++){
            if(packet.data()[j]<epsilon) continue;
            W_map[j-3] = packet[j];
            sum+=packet[j];
        }

        for(auto &iter:W_map)
            iter.second/=sum;
        W.push_back(W_map);

        for(int j=25;j<49;j++)
            Wj(i,j-25) = packet[j];
        (*socket)<<chk;
    }
    V *= cm;
    Vector3d min = V.colwise().minCoeff().transpose();
    Vector3d max = V.colwise().maxCoeff().transpose();
    center = (min+max)*0.5;
    size = max-min;

    timer.Stop(); double vTime = timer.GetRealElapsed();
    G4cout<<vTime<<G4endl;
    igl::normalize_row_sums(Wj,Wj);
    timer.Start();

    cout<<"Get data for "<<tNum<<" tetrahedrons..."<<flush;
    tetVec.clear();
    T.resize(tNum,4);
    for(int i=0;i<tNum;i++){
        array<int, 4> t;
        socket->RecvIntBuffer(t.data(),4);
        tetVec.push_back(new G4Tet("tet",
                                   G4ThreeVector(V(t.data()[0],0),V(t.data()[0],1),V(t.data()[0],2)),
                                   G4ThreeVector(V(t.data()[1],0),V(t.data()[1],1),V(t.data()[1],2)),
                                   G4ThreeVector(V(t.data()[2],0),V(t.data()[2],1),V(t.data()[2],2)),
                                   G4ThreeVector(V(t.data()[3],0),V(t.data()[3],1),V(t.data()[3],2))));
        for(int j=0;j<4;j++) T(i,j) = t.data()[j];
        (*socket)<<chk;
    }timer.Stop(); double tTime = timer.GetRealElapsed();
    G4cout<<tTime<<G4endl; timer.Start();

    cout<<"Get data for "<<fNum<<" faces..."<<flush;
    F.resize(fNum,3);

    adjacentFaces.clear();
    for(int i=0;i<fNum;i++){
        array<int, 3> f;
        socket->RecvIntBuffer(f.data(),3);
        for(int j=0;j<3;j++){
            F(i,j) = f.data()[j];
            adjacentFaces[f.data()[j]].push_back(i);
        }
        (*socket)<<chk;
    }
    int count(0);
    for(auto iter:adjacentFaces) u2o[iter.first] = count++;
    timer.Stop(); double fTime = timer.GetRealElapsed();
    G4cout<<fTime <<" (outerV: "<<adjacentFaces.size()<<")"<<G4endl;

    if(eNum>180) G4cout<<"Chk eye packet!!!!!!"<<G4endl;
    array<int, 180> eyeArray;
    socket->RecvIntBuffer(eyeArray.data(),180);
    eyeFaces.clear();
    for(int i=0;i<eNum;i++) eyeFaces.insert(eyeArray[i]);
    G4cout<<"total: "<<vTime+fTime+tTime<<G4endl;

    cout<<"Generate Skin Layers..."<<flush; timer.Start();
    U=V;
    GenerateOffset();
    /* |<--skin_d(t)-->0|<-- skin_d -->1|*/
    //layers except for target
    for(int i=0;i<F.rows();i++){
        tetVec.push_back(new G4Tet("tet",
                                   U_layer0[u2o[F(i,0)]],U_layer0[u2o[F(i,1)]],
                                   U_layer0[u2o[F(i,2)]],U_layer1[u2o[F(i,1)]])); //0,1,2,1'
        tetVec.push_back(new G4Tet("tet",
                                   U_layer0[u2o[F(i,0)]],U_layer1[u2o[F(i,1)]],
                                   U_layer0[u2o[F(i,2)]],U_layer1[u2o[F(i,0)]])); //0,1',2,0'
        tetVec.push_back(new G4Tet("tet",
                                   U_layer1[u2o[F(i,0)]],U_layer1[u2o[F(i,2)]],
                                   U_layer0[u2o[F(i,2)]],U_layer1[u2o[F(i,1)]])); //0',2',2,1'
    }nonTargetNum = tetVec.size();
    //target layer
    for(int i=0;i<F.rows();i++){
        tetVec.push_back(new G4Tet("tet",
                                   G4ThreeVector(U(F(i,0),0),U(F(i,0),1),U(F(i,0),2)),
                                   G4ThreeVector(U(F(i,1),0),U(F(i,1),1),U(F(i,1),2)),
                                   G4ThreeVector(U(F(i,2),0),U(F(i,2),1),U(F(i,2),2)),
                                   U_layer0[u2o[F(i,1)]])); // 0,1,2,1'
        tetVec.push_back(new G4Tet("tet",
                                   G4ThreeVector(U(F(i,0),0),U(F(i,0),1),U(F(i,0),2)),
                                   U_layer0[u2o[F(i,1)]],
                                   G4ThreeVector(U(F(i,2),0),U(F(i,2),1),U(F(i,2),2)),
                                   U_layer0[u2o[F(i,0)]])); // 0,1',2,0'
        tetVec.push_back(new G4Tet("tet",
                                   U_layer0[u2o[F(i,0)]],
                                   U_layer0[u2o[F(i,2)]],
                                   G4ThreeVector(U(F(i,2),0),U(F(i,2),1),U(F(i,2),2)),
                                   U_layer0[u2o[F(i,1)]])); // 0',2',2,1'
    }
    timer.Stop();
    G4cout<<timer.GetRealElapsed()<<G4endl;

    MatrixXd calib_M(24,3);
    socket->RecvDoubleBuffer(calib_M.data(),72);
    G4cout<<"Recieved calib data"<<G4endl;
    V_calib = V+Wj*calib_M*cm;
    return true;
}

void ModelImport::ReadMatFile(G4String filename){
    G4Material* tissue = G4NistManager::Instance()->FindOrBuildMaterial("G4_ADIPOSE_TISSUE_ICRP");
    matMap[121] = tissue;
    matMap[148] = tissue;
}

void ModelImport::PostureChange(const RotationList &vQ, const std::vector<Vector3d> &vT, bool calib){
    using namespace std;
    G4Timer timer; timer.Start();

    // Convert quats + trans into dual parts
    vector<Eigen::Quaterniond> vD(vQ.size());
    for(size_t c = 0;c<vQ.size();c++)
    {
      const Eigen::Quaterniond  & q = vQ[c];
      vD[c].w() = -0.5*( vT[c](0)*q.x() + vT[c](1)*q.y() + vT[c](2)*q.z());
      vD[c].x() =  0.5*( vT[c](0)*q.w() + vT[c](1)*q.z() - vT[c](2)*q.y());
      vD[c].y() =  0.5*(-vT[c](0)*q.z() + vT[c](1)*q.w() + vT[c](2)*q.x());
      vD[c].z() =  0.5*( vT[c](0)*q.y() - vT[c](1)*q.x() + vT[c](2)*q.w());
    }

    // Loop over vertices
    if(calib) U = V_calib;
    else U = V;
    for(size_t i = 0;i<U.rows();i++)
    {
      Eigen::Quaterniond b0(0,0,0,0);
      Eigen::Quaterniond be(0,0,0,0);
      Eigen::Quaterniond vQ0;
      bool first(true);
      // Loop over handles
      for(auto iter:W[i])
      {
          if(first){
              b0.coeffs() = iter.second * vQ[iter.first].coeffs();
              be.coeffs() = iter.second * vD[iter.first].coeffs();
              vQ0 = vQ[iter.first];
              first = false;
              continue;
          }
          if( vQ0.dot( vQ[iter.first] ) < 0.f ){
              b0.coeffs() -= iter.second * vQ[iter.first].coeffs();
              be.coeffs() -= iter.second * vD[iter.first].coeffs();
          }else{
              b0.coeffs() += iter.second * vQ[iter.first].coeffs();
              be.coeffs() += iter.second * vD[iter.first].coeffs();
          }
      }
      Eigen::Quaterniond ce = be;
      ce.coeffs() /= b0.norm();
      Eigen::Quaterniond c0 = b0;
      c0.coeffs() /= b0.norm();
      // See algorithm 1 in "Geometric skinning with approximate dual quaternion
      // blending" by Kavan et al
      Vector3d v = U.row(i);
      Vector3d d0 = c0.vec();
      Vector3d de = ce.vec();
      Eigen::Quaterniond::Scalar a0 = c0.w();
      Eigen::Quaterniond::Scalar ae = ce.w();
      //Vector3d u =  v + 2*d0.cross(d0.cross(v) + a0*v) + 2*(a0*de - ae*d0 + d0.cross(de));
      U.row(i) =  v + 2*d0.cross(d0.cross(v) + a0*v) + 2*(a0*de - ae*d0 + d0.cross(de));
    }
    Vector3d min = U.colwise().minCoeff().transpose();
    Vector3d max = U.colwise().maxCoeff().transpose();
    center = (min+max)*0.5;
    size = max-min;
    U = U.rowwise() - center.transpose();

    GenerateOffset();
    for(int i=0;i<T.rows();i++){
        ChkDegenAndSetVertices(G4ThreeVector(U(T(i,0),0),U(T(i,0),1),U(T(i,0),2)),
                               G4ThreeVector(U(T(i,1),0),U(T(i,1),1),U(T(i,1),2)),
                               G4ThreeVector(U(T(i,2),0),U(T(i,2),1),U(T(i,2),2)),
                               G4ThreeVector(U(T(i,3),0),U(T(i,3),1),U(T(i,3),2)),
                               tetVec[i]);
    }
    int count = T.rows();
    for(int i=0;i<F.rows();i++){
        ChkDegenAndSetVertices(U_layer0[u2o[F(i,0)]],U_layer0[u2o[F(i,1)]],
                               U_layer0[u2o[F(i,2)]],U_layer1[u2o[F(i,1)]], tetVec[count+i*3]); //0,1,2,1'
        ChkDegenAndSetVertices(U_layer0[u2o[F(i,0)]],U_layer1[u2o[F(i,1)]],
                               U_layer0[u2o[F(i,2)]],U_layer1[u2o[F(i,0)]],tetVec[count+i*3+1]); //0,1',2,0'
        ChkDegenAndSetVertices(U_layer1[u2o[F(i,0)]],U_layer1[u2o[F(i,2)]],
                               U_layer0[u2o[F(i,2)]],U_layer1[u2o[F(i,1)]],tetVec[count+i*3+2]); //0',2',2,1'
    }
    count = T.rows() + F.rows()*3;
    for(int i=0;i<F.rows();i++){
        ChkDegenAndSetVertices(G4ThreeVector(U(F(i,0),0),U(F(i,0),1),U(F(i,0),2)),
                               G4ThreeVector(U(F(i,1),0),U(F(i,1),1),U(F(i,1),2)),
                               G4ThreeVector(U(F(i,2),0),U(F(i,2),1),U(F(i,2),2)),
                               U_layer0[u2o[F(i,1)]],tetVec[count+i*3]); // 0,1,2,1'
        ChkDegenAndSetVertices(G4ThreeVector(U(F(i,0),0),U(F(i,0),1),U(F(i,0),2)),
                               U_layer0[u2o[F(i,1)]],
                               G4ThreeVector(U(F(i,2),0),U(F(i,2),1),U(F(i,2),2)),
                               U_layer0[u2o[F(i,0)]],tetVec[count+i*3+1]); // 0,1',2,0'
        ChkDegenAndSetVertices(U_layer0[u2o[F(i,0)]],
                               U_layer0[u2o[F(i,2)]],
                               G4ThreeVector(U(F(i,2),0),U(F(i,2),1),U(F(i,2),2)),
                               U_layer0[u2o[F(i,1)]],tetVec[count+i*3+2]); // 0',2',2,1'
    }
    timer.Stop(); G4cout<<"posture deformation: "<<timer.GetRealElapsed()<<" s"<<G4endl;
}

void ModelImport::GenerateOffset(){
    U_layer0.clear(); U_layer1.clear();
    double skin_d = 50*um; double lens_d = 3*mm;
    /* |<--skin_d(t)-->0|<-- skin_d -->1|*/
    MatrixXd F_normals;
    igl::per_face_normals(U, F, F_normals);
    VectorXd planeD0(F.rows()), planeD1(F.rows());
    for(int i=0;i<F.rows();i++){
        Vector3d p0 = U.row(F(i,0))+F_normals.row(i)*skin_d;
        planeD0(i) = p0.dot(F_normals.row(i));
        Vector3d p1;
        if(eyeFaces.find(i)==eyeFaces.end())
            p1 = U.row(F(i,0))+F_normals.row(i)*skin_d*2;
        else
            p1 = U.row(F(i,0))+F_normals.row(i)*lens_d*2;

        planeD1(i) = p1.dot(F_normals.row(i));
    }
    for(auto iter:adjacentFaces){
        MatrixXd A(iter.second.size(),3);
        VectorXd B0(iter.second.size()), B1(iter.second.size());
        for(int i=0;i<A.rows();i++){
            A.row(i) = F_normals.row(iter.second[i]);
            B0(i) = planeD0(iter.second[i]);
            B1(i) = planeD1(iter.second[i]);
        }
        MatrixXd AAA = (A.transpose()*A).inverse()*A.transpose();
        Vector3d v0 = AAA*B0;
        Vector3d v1 = AAA*B1;
        U_layer0.push_back(G4ThreeVector(v0(0),v0(1),v0(2)));
        U_layer1.push_back(G4ThreeVector(v0(0),v1(1),v1(2)));
    }    
}

G4double ModelImport::FixDegenTet3(G4ThreeVector& anchor, G4ThreeVector& p2, G4ThreeVector& p3, G4ThreeVector& p4, G4double &vol6)
{
    vector<G4ThreeVector> pointVec = { anchor, p2, p3, p4 };
    vector<G4ThreeVector> upVector;
    for (int i = 0; i < 4; i++) {
        G4ThreeVector up = (pointVec[(i + 2) % 4] - pointVec[(i + 1) % 4]).cross(pointVec[(i + 3) % 4] - pointVec[(i + 1) % 4]);
        up = up.unit();
        if (up.dot(pointVec[i] - pointVec[(i + 1) % 4]) < 0) {
            up = -up;
        }
        upVector.push_back(up);
    }
    double minHeight(DBL_MAX);
    vector<G4ThreeVector> midVector;
    vector<G4ThreeVector> cenVector;
    int minP(-1);
    for (int i = 0; i < 4; i++) {
        G4ThreeVector u = pointVec[(i + 1) % 4] - pointVec[i];
        G4double height = -upVector[i].dot(u);
        G4ThreeVector proj = pointVec[i] - upVector[i] * height;
        G4ThreeVector mid = (pointVec[(i + 1) % 4] + pointVec[(i + 2) % 4] + pointVec[(i + 3) % 4]) / 3.;
        midVector.push_back(mid - proj);
        cenVector.push_back(mid + upVector[i] * height);
        if (height > minHeight) continue;
        minHeight = height;
        minP = i;
    }

    G4bool degen = true;
    G4double totalMove(0.);
    G4double move = pow(vol6, 1. / 3.)*0.0001;
    if (move > 0.0001) move = 0.00001;
    while (degen) {
        pointVec[minP] += upVector[minP] * move;
        if ((cenVector[minP] - pointVec[minP]).dot(midVector[minP]) > 0) {
            pointVec[minP] += midVector[minP] * move;
            totalMove += move * 1.41421356237;
        }
        else totalMove += move;
        degen = ChkDegenTet2(pointVec[0], pointVec[1], pointVec[2], pointVec[3], vol6);
    }
    anchor = pointVec[0];
    p2 = pointVec[1];
    p3 = pointVec[2];
    p4 = pointVec[3];
    return totalMove;
}

G4bool ModelImport::ChkDegenTet2(G4ThreeVector p0, G4ThreeVector p1, G4ThreeVector p2, G4ThreeVector p3, G4double& vol6) {
    double hmin = 4.e-9;

    //Calculate volume
    vol6 = std::abs((p1 - p0).cross(p2 - p0).dot(p3 - p0));

    //calculate face areas squared
    double ss[4];
    ss[0] = ((p1 - p0).cross(p2 - p0)).mag2();
    ss[1] = ((p2 - p0).cross(p3 - p0)).mag2();
    ss[2] = ((p3 - p0).cross(p1 - p0)).mag2();
    ss[3] = ((p2 - p1).cross(p3 - p1)).mag2();

    //find face with max area
    int k=0;
    for(int i=0;i<4;++i){if(ss[i]>ss[k]) k=i;}

    //check: vol^2 / s^2 <= hmin^2
    return (vol6*vol6 <= ss[k]*hmin*hmin);
}
