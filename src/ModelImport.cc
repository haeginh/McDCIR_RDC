#include "ModelImport.h"
#include "igl/readMESH.h"
#include "igl/readDMAT.h"

ModelImport::ModelImport(G4String modelName)
{
//    igl::readMESH(modelName+".mesh",V,T,F);
//    igl::readDMAT(modelName+".dmat",W);
}

G4bool ModelImport::RecvInitData(){

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
  #pragma omp parallel for if (vNum>10000)
    for(G4int i = 0;i<vNum;i++)
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
      Vector3d v(V[i].getX(),V[i].getY(),V[i].getZ());
      Vector3d d0 = c0.vec();
      Vector3d de = ce.vec();
      Eigen::Quaterniond::Scalar a0 = c0.w();
      Eigen::Quaterniond::Scalar ae = ce.w();
      Vector3d u =  v + 2*d0.cross(d0.cross(v) + a0*v) + 2*(a0*de - ae*d0 + d0.cross(de));
      U.push_back(G4ThreeVector(u(0),u(1),u(2)));
    }
}
