#include "ModelImport.h"
#include "igl/readMESH.h"
#include "igl/readDMAT.h"
#include "G4Timer.hh"

ModelImport::ModelImport(ClientSocket* _sock)
    :socket(_sock)
{
//    igl::readMESH(modelName+".mesh",V,T,F);
//    igl::readDMAT(modelName+".dmat",W);
}

G4bool ModelImport::RecvInitData(){
    G4Timer timer; timer.Start();
    G4String data; (*socket)>>data;
    std::stringstream ss(data);
    int vNum, tNum, fNum;
    ss>>vNum>>tNum>>fNum;
    G4cout<<data<<G4endl;
    G4cout<<"Get data for "<<vNum<<" vertices...";
    V.resize(vNum,3); Wj.resize(vNum,24);
    W.clear();
    G4double epsilon(1e-5);
    for(int i=0;i<vNum;i++){
        array<double, 3> p;
        array<double, 22> w;
        array<double, 24> wj;
        socket->RecvDoubleBuffer(p.data(),3);
        for(int j=0;j<3;j++) V(i,j) = p.data()[j];
        socket->RecvDoubleBuffer(w.data(),22);
        G4double sum(0);
        std::map<int, double> W_map;
        for(G4int j=0;j<22;j++){
            if(w.data()[j]<epsilon) continue;
            W_map[j] = w.data()[j];
            sum+=w.data()[j];
        }
        for(auto &iter:W_map) iter.second/=sum;
        W.push_back(W_map);

        socket->RecvDoubleBuffer(wj.data(),24);
        for(int j=0;j<24;j++) Wj(i,j) = wj.data()[j];

    } timer.Stop(); double vTime = timer.GetRealElapsed();
    G4cout<<vTime<<G4endl;
    igl::normalize_row_sums(Wj,Wj);
    timer.Start();

    cout<<"Get data for "<<tNum<<" tetrahedrons..."<<flush;
    tetVec.clear();
    for(int i=0;i<tNum;i++){
        array<int, 4> t;
        socket->RecvIntBuffer(t.data(),4);
        tetVec.push_back(new G4Tet("tet",
                                   G4ThreeVector(V(t.data()[0],0),V(t.data()[0],1),V(t.data()[0],2)),
                                   G4ThreeVector(V(t.data()[1],0),V(t.data()[1],1),V(t.data()[1],2)),
                                   G4ThreeVector(V(t.data()[2],0),V(t.data()[2],1),V(t.data()[2],2)),
                                   G4ThreeVector(V(t.data()[3],0),V(t.data()[3],1),V(t.data()[3],2))));
    }timer.Stop(); double tTime = timer.GetRealElapsed();
    G4cout<<tTime<<G4endl; timer.Start();

    cout<<"Get data for "<<fNum<<" faces..."<<flush;
    F.resize(fNum,3);
    for(int i=0;i<fNum;i++){
        array<int, 3> f;
        socket->RecvIntBuffer(f.data(),3);
        for(int j=0;j<3;j++)F(i,j) = f.data()[j];
    }timer.Stop(); double fTime = timer.GetRealElapsed();
    G4cout<<fTime<<G4endl; timer.Start();
    G4cout<<"total: "<<vTime+fTime+tTime<<G4endl;
    MatrixXd calib_M(24,3);
    socket->RecvDoubleBuffer(calib_M.data(),72);
    G4cout<<"Recieved calib data"<<G4endl;
    V_calib = V+Wj*calib_M;
    return true;
}


void ModelImport::PostureChange(const RotationList &vQ, const std::vector<Vector3d> &vT){
    using namespace std;

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
    U.clear();
    for(size_t i = 0;i<V.rows();i++)
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
      Vector3d v = V.row(i);
      Vector3d d0 = c0.vec();
      Vector3d de = ce.vec();
      Eigen::Quaterniond::Scalar a0 = c0.w();
      Eigen::Quaterniond::Scalar ae = ce.w();
      Vector3d u =  v + 2*d0.cross(d0.cross(v) + a0*v) + 2*(a0*de - ae*d0 + d0.cross(de));
      U.push_back(G4ThreeVector(u(0),u(1),u(2)));
    }
}
