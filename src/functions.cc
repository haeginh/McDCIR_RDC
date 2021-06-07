#include "functions.h"

MatrixXd GenerateBonePoints(MatrixXd C, MatrixXi BE, double interval){
    MatrixXd output=C;
    for(int i=0;i<BE.rows();i++){
        Vector3d c0 = C.row(BE(i,0)).transpose();
        Vector3d c1 = C.row(BE(i,1)).transpose();
        Vector3d dir = (c1-c0).normalized();
        double l = (c1-c0).norm();
        int num = floor(l/interval);
        double interval1 = l/(double)num;
        int prevSize = output.rows();
        output.conservativeResize(prevSize+num-1,NoChange);
        for(double intvl=1;intvl<num;intvl++)
            output.row(prevSize+intvl-1) = (c0+dir*interval1*intvl).transpose();
    }
    return output;
}
map<tuple<int,int,int>,vector<int>> GenerateGrid(MatrixXd V, double size){
    map<tuple<int,int,int>,vector<int>> grid;
    for(int i=0;i<V.rows();i++){
        int x = floor(V(i,0)*size+0.5);
        int y = floor(V(i,1)*size+0.5);
        int z = floor(V(i,2)*size+0.5);
        auto key = make_tuple(x,y,z);
        if(grid.find(key)==grid.end()) grid[key]={};
        grid[key].push_back(i);
    }
    return grid;
}

map<int, map<int, double>> GenerateBarycentricCoord(MatrixXd V_f, MatrixXi T_f, MatrixXd V){
    map<tuple<int,int,int>,vector<int>> grid = GenerateGrid(V);
    map<int, map<int, double>> baryCoords;
    double epsl(-1e-5);
    for(int n=0;n<T_f.rows();n++){
        vector<Vector3d> tet;
        Vector3d max(-1e10,-1e10,-1e10), min(1e10,1e10,1e10);
        for(int e=0;e<4;e++){
            tet.push_back(V_f.row(T_f(n,e)).transpose());
            max(0) = max(0)>tet[e](0)? max(0):tet[e](0);
            max(1) = max(1)>tet[e](1)? max(1):tet[e](1);
            max(2) = max(2)>tet[e](2)? max(2):tet[e](2);
            min(0) = min(0)<tet[e](0)? min(0):tet[e](0);
            min(1) = min(1)<tet[e](1)? min(1):tet[e](1);
            min(2) = min(2)<tet[e](2)? min(2):tet[e](2);
        }
        double invVol6 = 1./(tet[1]-tet[0]).cross(tet[2]-tet[0]).dot(tet[3]-tet[0]);
        int i_max = floor(max(0)+0.5);int i_min = floor(min(0)+0.5);
        int j_max = floor(max(1)+0.5);int j_min = floor(min(1)+0.5);
        int k_max = floor(max(2)+0.5);int k_min = floor(min(2)+0.5);
        for(int i=i_min;i<i_max+1;i++){
            for(int j=j_min;j<j_max+1;j++){
                for(int k=k_min;k<k_max+1;k++){
                    auto key = make_tuple(i,j,k);
                    for(int idx:grid[key]){
                        if(baryCoords.find(idx)!=baryCoords.end()) continue;
                        Vector3d v = V.row(idx).transpose();
                        double b0 = (tet[1]-v).cross(tet[2]-v).dot(tet[3]-v)*invVol6;
                        if(b0<epsl) continue;
                        double b1 = (v-tet[0]).cross(tet[2]-tet[0]).dot(tet[3]-tet[0])*invVol6;
                        if(b1<epsl) continue;
                        double b2 = (tet[1]-tet[0]).cross(v-tet[0]).dot(tet[3]-tet[0])*invVol6;
                        if(b2<epsl) continue;
                        double b3 = (tet[1]-tet[0]).cross(tet[2]-tet[0]).dot(v-tet[0])*invVol6;
                        if(b3<epsl) continue;
                        map<int, double> bary;
                        bary[T_f(n,0)] = b0; bary[T_f(n,1)] = b1; bary[T_f(n,2)] = b2; bary[T_f(n,3)] = b3;
                        baryCoords[idx] = bary;
                    }
                }
            }
        }
        cout<<"\rGenerating barycentric coord..."<<baryCoords.size()<<"/"<<V.rows()<<"       "<<flush;
        //if((int)baryCoords.size()==V.rows()) break;
    }cout<<endl;
    if((int)baryCoords.size()!=V.rows()){
        cout<<"Check if all the vertices are in frame model!!"<<endl; exit(100);
    }
    return baryCoords;
}
void PrintBaryCoords(string file, map<int, map<int, double>> &baryCoords){
    ofstream ofs(file); ofs<<baryCoords.size()<<endl;
    for(size_t i=0;i<baryCoords.size();i++){
        ofs<<baryCoords[i].size()<<" ";
        for(auto iter:baryCoords[i])
            ofs<<iter.first<<" "<<iter.second<<" ";
        ofs<<endl;
    }ofs.close();
}
map<int, map<int, double>> ReadBaryFile(string file){
    map<int, map<int, double>> baryCoords;
    ifstream ifs(file);
    if(!ifs.is_open()) {
        cout<<file+" is not open!!!"<<endl;
        return baryCoords;
    }
    int num; ifs>>num;
    for(int i=0;i<num;i++){
        int count; ifs>>count;
        int vID; double w;
        map<int, double> bary;
        for(int n=0;n<count;n++){
            ifs>>vID>>w;
            bary[vID]=w;
        }
        baryCoords[i] = bary;
    }ifs.close();
    return baryCoords;
}
SparseMatrix<double> GenerateBarySparse(map<int, map<int, double>> &baryCoords, int v_size){
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplets;
    for(size_t i=0;i<baryCoords.size();i++){
        for(auto w:baryCoords[i]){
            triplets.push_back(T(i,w.first,w.second));
        }
    }
    SparseMatrix<double> mat(baryCoords.size(),v_size);
    mat.setFromTriplets(triplets.begin(),triplets.end());
    return mat;
}
bool CalculateScalingWeights(MatrixXd &C, MatrixXd &V,MatrixXi &T,MatrixXd &W){
    VectorXi b; b.resize(C.rows());
    MatrixXd bc = MatrixXd::Zero(C.rows(),C.rows());
    map<tuple<int,int,int>,vector<int>> grid = GenerateGrid(V);
    for(int n=0;n<C.rows();n++){
        int x = floor(C(n,0)+0.5);
        int y = floor(C(n,1)+0.5);
        int z = floor(C(n,2)+0.5);
        auto key = make_tuple(x,y,z);
        for(int i:grid[key]){
            if(fabs(C(n,0)-V(i,0))>0.01) continue;
            if(fabs(C(n,1)-V(i,1))>0.01) continue;
            if(fabs(C(n,2)-V(i,2))>0.01) continue;
            b(n) = i;
            bc(n,n) = 1;
            break;
        }
    }

    // compute BBW weights matrix
    igl::BBWData bbw_data;
    // only a few iterations for sake of demo
    bbw_data.active_set_params.max_iter = 10;
    bbw_data.verbosity = 2;
    return igl::bbw(V,T,b,bc,bbw_data,W);
}
bool CalculateScalingWeights(MatrixXd &C, MatrixXd &V,MatrixXi &T,MatrixXd &W, vector<int> eyeLV, vector<int> eyeRV){
    VectorXi b; b.resize(C.rows()-3+eyeLV.size()+eyeRV.size());
    MatrixXd bc = MatrixXd::Zero(C.rows()-3+eyeLV.size()+eyeRV.size(),C.rows()-1);
    map<tuple<int,int,int>,vector<int>> grid = GenerateGrid(V);
    int n=0;
    for(;n<C.rows()-3;n++) // exclude head joints
    {
        int x = floor(C(n,0)+0.5);
        int y = floor(C(n,1)+0.5);
        int z = floor(C(n,2)+0.5);
        auto key = make_tuple(x,y,z);
        for(int i:grid[key]){
            if(fabs(C(n,0)-V(i,0))>0.01) continue;
            if(fabs(C(n,1)-V(i,1))>0.01) continue;
            if(fabs(C(n,2)-V(i,2))>0.01) continue;
            b(n) = i;
            bc(n,n) = 1;
            break;
        }
    }
    for(int i:eyeLV){b(n) = i; bc(n++, C.rows()-3)=1; }
    for(int i:eyeRV){b(n) = i; bc(n++, C.rows()-2)=1; }
    // compute BBW weights matrix
    igl::BBWData bbw_data;
    // only a few iterations for sake of demo
    bbw_data.active_set_params.max_iter = 10;
    bbw_data.verbosity = 2;
    return igl::bbw(V,T,b,bc,bbw_data,W);
}
AngleAxisd GetRotMatrix(Vector3d from, Vector3d to) {
    Vector3d axis = from.cross(to); axis.normalize();
    double theta = acos(from.dot(to) / (from.norm()*to.norm()));
    return AngleAxisd(theta, axis);
}

void myDqs(
  const Eigen::MatrixXd & V,
  const std::vector<std::map<int, double>> & W,
  const RotationList & vQ,
  const std::vector<Vector3d> & vT,
  Eigen::MatrixXd & U)
{
  using namespace std;
//  assert(V.rows() <= W.rows());
//  assert(W.cols() == (int)vQ.size());
//  assert(W.cols() == (int)vT.size());
  // resize output
  U.resizeLike(V);

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
  const int nv = V.rows();
#pragma omp parallel for if (nv>10000)
  for(int i = 0;i<nv;i++)
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
    U.row(i) =  v + 2*d0.cross(d0.cross(v) + a0*v) + 2*(a0*de - ae*d0 + d0.cross(de));
  }

}


//void SkelTest(string fileName){
//    ifstream ifs(fileName);
//    if(!ifs.is_open()) {cerr<<"There is no "+fileName<<endl; exit(1);}

//    string dump;
//    vector<G4ThreeVector> vertices;
//    vector<vector<int>> faces;
//    G4ThreeVector aPoint;
//    int a, b, c, vertCount(0);
//    ofstream ofs2("skel.weight");
//    while(getline(ifs, dump)){
//        stringstream ss(dump);
//        ss>>dump;
//        if(dump=="v"){
//            ss>>aPoint;
//            vertices.push_back(aPoint);
//            vertCount++;
//        }
//        else if(dump=="g"){
//            string shellName;
//            ss>>shellName;
//            int id = atoi(shellName.c_str());
//            for(int i=vertices.size()-vertCount;i<vertices.size();i++){
//                ofs2<<i<<" "<<id<<" 1"<<endl;
//            }
//            vertCount = 0;
//        }
//        else if(dump=="f"){
//            ss>>a>>b>>c;
//            faces.push_back({a-1,b-1,c-1});
//        }
//    }ifs.close(); ofs2.close();
//    cout<<"Imported "+fileName<<endl;
//    ofstream ofs("skel.ply");
//    ofs<<"ply"<<endl;
//    ofs<<"format ascii 1.0"<<endl;
//    ofs<<"comment Exported by RapidForm"<<endl;
//    ofs<<"element vertex "<<vertices.size()<<endl;
//    ofs<<"property float x"<<endl;
//    ofs<<"property float y"<<endl;
//    ofs<<"property float z"<<endl;
//    ofs<<"element face "<<faces.size()<<endl;
//    ofs<<"property list uchar int vertex_index"<<endl;
//    ofs<<"end_header"<<endl;
//    for(G4ThreeVector v:vertices)
//        ofs<<v.getX()<<" "<<v.getY()<<" "<<v.getZ()<<endl;
//    for(auto f:faces)
//        ofs<<"3 "<<f[0]<<" "<<f[1]<<" "<<f[2]<<endl;
//    ofs.close();
//    cout<<"skel.ply was generated"<<endl;

//    exit(2);
//}

//map<int, Vec3> ReadJointF(string jointF){
//    ifstream ifs(jointF);
//    if(!ifs.is_open()){
//        cout<<jointF<<" is not open"<<endl;
//        exit(1);
//    }
//    int id; double x, y, z;
//    map<int, Vec3> jointCenters;
//    while(ifs>>id>>x>>y>>z)
//        jointCenters[id]=Vec3(x,y,z);
//    cout<<"Imported "<<jointCenters.size()<<" joint centers"<<endl;
//    return jointCenters;
//}

//map<int, int> ReadKinectJoint(string kinectJointF){
//    ifstream ifs(kinectJointF);
//    if(!ifs.is_open()){
//        cout<<kinectJointF<<" is not open"<<endl;
//        exit(1);
//    }
//    int id,p_id; string dump;
//    map<int, int> parentJ;
//    while(ifs>>id>>dump>>p_id)
//        parentJ[id]=p_id;
//    cout<<"Imported "<<parentJ.size()<<" KINECT joint info."<<endl;
//    return parentJ;
//}

////Specification
//pair<G4TessellatedSolid*, G4TessellatedSolid*> ReadObj(string name, vector<Point3> &verts, map<int,vector<vector<int>>> &faces){
//    //initialization
//    verts.clear(); faces.clear();
//    //read file
//    ifstream ifs(name);
//    if(!ifs.is_open()) {cerr<<"There is no "+name<<endl; exit(1);}

//    cout<<"Reading "+name<<flush;
//    G4ThreeVector point;
//    int id, a, b, c;
//    vector<G4ThreeVector> vertVec;
//    string dump;
//    G4TessellatedSolid* tess0 = new G4TessellatedSolid();
//    G4TessellatedSolid* tess1 = new G4TessellatedSolid();
//    while(getline(ifs, dump)){
//        stringstream ss(dump);
//        ss>>dump;
//        if(dump=="v"){
//            ss>>point;
//            vertVec.push_back(point);
//        }
//        else if(dump=="g"){
//            ss>>id;
//            faces[id] = {};
//        }
//        else if(dump=="f"){
//            ss>>a>>b>>c;
//            faces[id].push_back({a, b, c});
//            if(id==0) tess0->AddFacet(new G4TriangularFacet(vertVec[a-1],vertVec[b-1],vertVec[c-1],ABSOLUTE));
//            else if(id==1) tess1->AddFacet(new G4TriangularFacet(vertVec[a-1],vertVec[b-1],vertVec[c-1],ABSOLUTE));
//        }
//    }ifs.close();
//    tess0->SetSolidClosed(true);
//    tess1->SetSolidClosed(true);
//    for(auto v:vertVec) verts.push_back(Point3(double(v.getX()), double(v.getY()), double(v.getZ())));
//    cout<<" - done"<<endl;
//    return make_pair(tess0, tess1);
//}


//void WriteEle(string name, vector<pair<vector<int>, double>> eleVol, vector<Point3> nodes, vector<vector<double>> weights){
//    ofstream ofs(name);
//    ofs<<eleVol.size()<<"  4  1"<<endl;
//    int count(0);
//    for(auto ele:eleVol){
//        if(nodes[ele.first[0]].z+nodes[ele.first[1]].z+nodes[ele.first[2]].z+nodes[ele.first[3]].z>8) continue;
//        int n(0);
//        double avg(0.);
//        for(int i=0;i<4;i++){
//            if(weights[ele.first[i]][0]==0) continue;
//            if(weights[ele.first[i]][0]==1) continue;
//            avg += weights[ele.first[i]][0];
//            n++;
//        }
//        if(n>0) avg/=n;
//        else avg = weights[ele.first[0]][0];
//        ofs<<count<<" "<<ele.first[0]<<" "<<ele.first[1]<<" "<<ele.first[2]<<" "<<ele.first[3]<<" "<<int(avg*100)<<endl;count++;
//    }
//    ofs.close();
//}

//void SetWeights(const vector<Point3> verts, vector<vector<double>> &weights){
//    cout<<"Setting weights"<<flush;
//    weights.clear();
///*    vector<Point3> verts0, verts1;
//    vector<vector<int>> faces0, faces1;
//    ReadPly("0.ply", verts0, faces0);
//    ReadPly("1.ply", verts1, faces1);
//    G4TessellatedSolid tess0, tess1;
//    vector<G4ThreeVector> verts0_g4, verts1_g4;
//    for(auto vert:verts0) verts0_g4.push_back(G4ThreeVector(vert.x, vert.y, vert.z));
//    for(auto vert:verts1) verts1_g4.push_back(G4ThreeVector(vert.x, vert.y, vert.z));

//    for(auto face:faces0){
//        tess0.AddFacet(new G4TriangularFacet(verts0_g4[face[0]],
//                                             verts0_g4[face[1]],
//                                             verts0_g4[face[2]],
//                                             ABSOLUTE));
//    }tess0.SetSolidClosed(true);
//    for(auto face:faces1){
//        tess1.AddFacet(new G4TriangularFacet(verts1_g4[face[0]],
//                                             verts1_g4[face[1]],
//                                             verts1_g4[face[2]],
//                                             ABSOLUTE));
//    }tess1.SetSolidClosed(true);
//    for(auto vert:verts){
//        if(vert.y>5) {weights.push_back({0,1}); continue;}
//        if(vert.y<-5) {weights.push_back({1,0}); continue;}
//        G4ThreeVector point(vert.x, vert.y, vert.z);
//        double d0_in = tess0.DistanceToIn(point);
//        double d0_out = tess0.DistanceToOut(point);
//        double d0 = (d0_in>d0_out)? d0_in:d0_out;
//        double d1_in = tess1.DistanceToIn(point);
//        double d1_out = tess1.DistanceToOut(point);
//        double d1 = (d1_in>d1_out)? d1_in:d1_out;
////        cout<<vert<<scientific<<": "<<d0<<" "<<d1;getchar();
//        d0 *= d0; d1 *= d1;
//        double r = d0/(d0+d1);
//        cout<<point.getY()<<"\t"<<r<<endl;
// //        cout<<r*r<<endl; getchar();

//        weights.push_back({1-r, r});
//    }*/

//    double margin(5);
//    double constant = -0.5/(margin*margin);
//    for(auto vert:verts){
//        if(vert.y>margin) {weights.push_back({0,1}); continue;}
//        if(vert.y<-margin) {weights.push_back({1,0}); continue;}
//        if(vert.y>0){
//           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
//           weights.push_back({1-w, w});
//        }else{
//            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
//            weights.push_back({w, 1-w});
//        }
//    }
//    cout<<"-done"<<endl;
//}


//double DistanceTo(vector<Point3> verts, Point3 point)
//{
//    double distance(3);
//    for(auto v:verts){
//        Vec3 vec = point-v;
//        if(abs(vec.y)>distance) continue;
//        if(abs(vec.x)>distance) continue;
//        if(abs(vec.z)>distance) continue;
//        double dist2=vec.norm_squared();
//        if(dist2<distance*distance) distance = sqrt(dist2);
//    }
//    return distance;
//}

//void SetWeights2(const vector<Point3> verts, vector<vector<double>> &weights, map<int,vector<vector<int>>> &faces, pair<G4TessellatedSolid*, G4TessellatedSolid*>tessPair)
//{
//    cout<<"Setting weights"<<flush;
//    weights.clear();

//    //quad
//    double margin(5);
//    double constant = -0.5/(margin*margin);
//    for(auto vert:verts){
//        if(vert.y>margin) {weights.push_back({0,1}); continue;}
//        if(vert.y<-margin) {weights.push_back({1,0}); continue;}
//        if(vert.y>0){
//           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
//           weights.push_back({1-w, w});
//        }else{
//            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
//            weights.push_back({w, 1-w});
//        }
//    }

//    //vert ext.
//    vector<int> verts0, verts1;
//    for(auto face:faces[0]){verts0.push_back(face[0]);verts0.push_back(face[1]);verts0.push_back(face[2]);}
//    for(auto face:faces[1]){verts1.push_back(face[0]);verts1.push_back(face[1]);verts1.push_back(face[2]);}
//    sort(verts0.begin(), verts0.end());
//    verts0.erase(unique(verts0.begin(), verts0.end()), verts0.end());
//    sort(verts1.begin(), verts1.end());
//    verts1.erase(unique(verts1.begin(), verts1.end()), verts1.end());
//    for(auto &id:verts0)id--;
//    for(auto &id:verts1)id--;

//    //bone rigid
//    for(auto id:verts0)weights[id] = {1,0};
//    for(auto id:verts1)weights[id] = {0,1};

//    vector<Point3> upperVec, lowerVec;
//    for(auto id:verts0)upperVec.push_back(verts[id]);
//    for(auto id:verts1)lowerVec.push_back(verts[id]);

//    //gradation
//    set<int> rigidNodes(verts0.begin(), verts0.end());
//    rigidNodes.insert(verts1.begin(), verts1.end());
//    for(size_t n=0;n<verts.size();n++){
//        if(rigidNodes.find(n)!=rigidNodes.end()) continue;
//        double upDist = DistanceTo(upperVec, verts[n]);
//        double lowDist = DistanceTo(lowerVec, verts[n]);
//        if(upDist<2 && lowDist<2){
//            double ratio = 1./(upDist+lowDist);
//            weights[n] = {lowDist*ratio, upDist*ratio};
//        }
//        else if(upDist<2){
//            double alpha = (2-upDist)*0.5;
//            weights[n][0] *= (1.-alpha);
//            weights[n][1] *= (1.-alpha);
//            weights[n][0] += alpha;
//        }
//        else if(lowDist<2){
//            double alpha = (2-lowDist)*0.5;
//            weights[n][0] *= (1.-alpha);
//            weights[n][1] *= (1.-alpha);
//            weights[n][1] += alpha;
//        }
//    }


//    cout<<"-done"<<endl;
//}

//void SetWeights3(const vector<Point3> nodes, vector<vector<double>> &weights, map<int, vector<int>> attributes)
//{
//    cout<<"Setting weights"<<flush;
//    weights.clear();

//    //quad
//    double margin(5);
//    double constant = -0.5/(margin*margin);
//    for(auto vert:nodes){
//        if(vert.y>margin) {weights.push_back({0,1}); continue;}
//        if(vert.y<-margin) {weights.push_back({1,0}); continue;}
//        if(vert.y>0){
//           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
//           weights.push_back({1-w, w});
//        }else{
//            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
//            weights.push_back({w, 1-w});
//        }
//    }

//    //bone rigid
//    int upperBone(2), lowerBone(3);
//    for(auto id:attributes[upperBone])weights[id] = {1,0};
//    for(auto id:attributes[lowerBone])weights[id] = {0,1};

//    vector<Point3> upperVec, lowerVec;
//    for(auto id:attributes[upperBone])upperVec.push_back(nodes[id]);
//    for(auto id:attributes[lowerBone])lowerVec.push_back(nodes[id]);

//    //gradation
//    set<int> rigidNodes(attributes[upperBone].begin(), attributes[upperBone].end());
//    rigidNodes.insert(attributes[lowerBone].begin(), attributes[lowerBone].end());
//    for(size_t n=0;n<nodes.size();n++){
//        if(rigidNodes.find(n)!=rigidNodes.end()) continue;
//        double upDist = DistanceTo(upperVec, nodes[n]);
//        double lowDist = DistanceTo(lowerVec, nodes[n]);
//        if(upDist<2 && lowDist<2){
//            double ratio = 1./(upDist+lowDist);
//            weights[n] = {lowDist*ratio, upDist*ratio};
//        }
//        else if(upDist<2){
//            double alpha = (2-upDist)*0.5;
//            weights[n][0] *= (1.-alpha);
//            weights[n][1] *= (1.-alpha);
//            weights[n][0] += alpha;
//        }
//        else if(lowDist<2){
//            double alpha = (2-lowDist)*0.5;
//            weights[n][0] *= (1.-alpha);
//            weights[n][1] *= (1.-alpha);
//            weights[n][1] += alpha;
//        }
//    }

//    cout<<"-done"<<endl;
//}

//std::vector<std::map<int, double>> ReadWeights(string fileName){
//    ifstream ifs(fileName);
//    string dump;
//    stringstream ss;
//    std::map<int, double> wMap;
//    std::vector<std::map<int, double>> weights;
//    double cut(1e-7);
//    while(getline(ifs, dump)){
//        ss.clear(); wMap.clear();
//        ss.str(dump);
//        int id; double w, sum(0);
//        ss>>dump;
//        while(ss>>id){
//            ss>>w;
//            if(w<cut) continue;
//            sum += w;
//            wMap[id] = w;
//        }
//        for(auto &w:wMap) w.second /= sum;
//        weights.push_back(wMap);
//    }ifs.close();

//    return weights;
//}

//std::vector<std::map<int, double>> ReadWeightMat(string fileName){
//    double cut(1e-7);
//    ifstream ifs(fileName);
//    string dump;
//    stringstream ss;
//    std::map<int, double> wMap;
//    std::vector<std::map<int, double>> weights;
//    while(getline(ifs, dump)){
//        ss.clear(); wMap.clear();
//        ss.str(dump);
//        int id(0); double w;
//        double sum(0);
//        while(ss>>w){
//            if(w>cut){
//                wMap[id] = w;
//                sum += w;
//            }
//            id++;
//        }
//        for(auto &w:wMap) w.second /= sum;
//        weights.push_back(wMap);
//    }ifs.close();
//    return weights;
//}

//void ReadPly(string name, vector<Point3> &verts, vector<vector<int>> &faces)
//{
//    //initialization
//    verts.clear(); faces.clear();

//    //read file
//    ifstream ifs(name);
//    if(!ifs.is_open()) {cerr<<"There is no "<<name<<endl; exit(1);}

//    Point3 point;
//    string dump;
//    int nb_verts(0);
//    int nb_faces(0);
//    while(1){
//        getline(ifs, dump);
//        if(dump.find("element vertex")!=string::npos){
//            stringstream ss(dump);
//            ss>>dump>>dump>>nb_verts;
//        }
//        else if(dump.find("element face")!=string::npos){
//            stringstream ss(dump);
//            ss>>dump>>dump>>nb_faces;
//        }
//        else if(dump.find("end_header")!=string::npos) break;
//    }
//    cout<<"Start reading "+name+"...(v:"<<nb_verts<<"/f:"<<nb_faces<<")"<<flush;
//    for(int i=0;i<nb_verts;i++){
//        ifs>>point;
//        verts.push_back(point);
//    }
//    for(int i=0;i<nb_faces;i++){
//        int a, b, c;
//        ifs>>dump>>a>>b>>c;
//        vector<int> face = {a, b, c};
//        faces.push_back(face);
//    }
//    ifs.close();
//    cout<<"-done"<<endl;
//}

//void PrintPly(string name, vector<Point3> verts, vector<vector<int>> faces)
//{
//    ofstream ofs(name);
//    ofs<<"ply"<<endl;
//    ofs<<"format ascii 1.0"<<endl;
//    ofs<<"comment Exported by RapidForm"<<endl;
//    ofs<<"element vertex "<<verts.size()<<endl;
//    ofs<<"property double x"<<endl;
//    ofs<<"property double y"<<endl;
//    ofs<<"property double z"<<endl;
//    ofs<<"element face "<<faces.size()<<endl;
//    ofs<<"property list uchar int vertex_index"<<endl;
//    ofs<<"end_header"<<endl;
//    for(Point3 vert:verts) ofs<<vert<<endl;
//    for(vector<int> face:faces) ofs<<"3 "<<face[0]<<" "<<face[1]<<" "<<face[2]<<endl;
//    ofs.close();
//    cout<<"Printed "+name<<endl;
//}

//void ReadNode(string name, vector<Point3> &nodes)
//{
//    nodes.clear();
//    ifstream ifsNode(name);
//    if(!ifsNode.is_open()) {cerr<<"There is no "<<name<<endl; exit(1);}
//    int nb_node;
//    string dump;
//    getline(ifsNode, dump);
//    stringstream ss(dump);
//    ss>>nb_node;
//    Point3 point;
//    for(int i=0;i<nb_node;i++){
//        ifsNode>>dump>>point;
//        nodes.push_back(point);
//    }ifsNode.close();
//}

//void ReadEle(string name, map<int, vector<int>> &attributes)
//{
//    attributes.clear();
//    ifstream ifsEle(name);
//    if(!ifsEle.is_open()){cerr<<"There is no "<<name<<endl; exit(1);}
//    int nb_ele;
//    string dump;
//    getline(ifsEle, dump);
//    stringstream ss(dump);
//    ss>>nb_ele;
//    int a, b, c, d, id(-100);
//    for(int i=0;i<nb_ele;i++){
//        getline(ifsEle, dump);
//        stringstream ss1(dump);
//        ss1>>dump>>a>>b>>c>>d>>id;
//        if(attributes.find(id)==attributes.end()) attributes[id]={a, b, c, d};
//        else {
//            attributes[id].push_back(a);
//            attributes[id].push_back(b);
//            attributes[id].push_back(c);
//            attributes[id].push_back(d);
//        }
//    }ifsEle.close();
//    for(auto &att:attributes){
//        sort(att.second.begin(), att.second.end());
//        att.second.erase(unique(att.second.begin(), att.second.end()), att.second.end());
//    }
//}

//void ReadEle2(string name, vector<Point3> nodes, vector<pair<vector<int>, double>> &eleVol, map<int, vector<int>> &attributes)
//{
//    attributes.clear();
//    eleVol.clear();
//    ifstream ifsEle(name);
//    if(!ifsEle.is_open()){cerr<<"There is no "<<name<<endl; exit(1);}
//    int nb_ele;
//    string dump;
//    getline(ifsEle, dump);
//    stringstream ss(dump);
//    ss>>nb_ele;
//    int a, b, c, d, id(-100);
//    for(int i=0;i<nb_ele;i++){
//        getline(ifsEle, dump);
//        stringstream ss1(dump);
//        ss1>>dump>>a>>b>>c>>d>>id;
//        Vec3 aa = nodes[a]-nodes[d];
//        Vec3 bb = nodes[b]-nodes[d];
//        Vec3 cc = nodes[c]-nodes[d];
//        double vol = (aa.cross(bb)).dot(cc);
//        vector<int> elements = {a, b, c, d};
//        eleVol.push_back(make_pair(elements, vol));
//        if(attributes.find(id)==attributes.end()) attributes[id]={a, b, c, d};
//        else {
//            attributes[id].push_back(a);
//            attributes[id].push_back(b);
//            attributes[id].push_back(c);
//            attributes[id].push_back(d);
//        }
//    }ifsEle.close();
//    for(auto &att:attributes){
//        sort(att.second.begin(), att.second.end());
//        att.second.erase(unique(att.second.begin(), att.second.end()), att.second.end());
//    }
//}

//void DetectTangled(const vector<Point3> nodes, vector<pair<vector<int>, double>> eleVol)
//{
//    double volDiff(0);
//    double tangledVol(0);
//    int count(0);
//    for(auto ele:eleVol){
//        Vec3 aa = nodes[ele.first[0]] - nodes[ele.first[3]];
//        Vec3 bb = nodes[ele.first[1]] - nodes[ele.first[3]];
//        Vec3 cc = nodes[ele.first[2]] - nodes[ele.first[3]];
//        double vol = (aa.cross(bb)).dot(cc);
//        if(ele.second*vol>0) volDiff+=abs(vol)-abs(ele.second);
//        else {count++; tangledVol+=vol/6.;}
//    }
//    cout<<"   Tangled: "<<count<<", "<<tangledVol<<"   "<<"Volume Diff: "<<volDiff/6.<<endl;;
//}

//void PrintNode(string name, const vector<Point3> nodes)
//{
//    ofstream ofsNode(name+".node");
//    ofsNode<<nodes.size()<<"  3  0  0"<<endl;
//    for(size_t i=0;i<nodes.size();i++){
//        ofsNode<<i<<" "<<nodes[i]<<endl;
//    }ofsNode.close();
//}


//void PrintObj(string name, const vector<Point3> verts, const map<int,vector<vector<int>>> faces)
//{
//    ofstream ofs(name+".obj");
//    for(auto v:verts)ofs<<"v "<<v<<endl;
//    ofs<<endl;
//    for(auto shell:faces){
//        ofs<<"g "<<shell.first<<endl<<endl;
//        for(auto face:shell.second)//void SkelTest(string fileName){
//    ifstream ifs(fileName);
//    if(!ifs.is_open()) {cerr<<"There is no "+fileName<<endl; exit(1);}

//    string dump;
//    vector<G4ThreeVector> vertices;
//    vector<vector<int>> faces;
//    G4ThreeVector aPoint;
//    int a, b, c, vertCount(0);
//    ofstream ofs2("skel.weight");
//    while(getline(ifs, dump)){
//        stringstream ss(dump);
//        ss>>dump;
//        if(dump=="v"){
//            ss>>aPoint;
//            vertices.push_back(aPoint);
//            vertCount++;
//        }
//        else if(dump=="g"){
//            string shellName;
//            ss>>shellName;
//            int id = atoi(shellName.c_str());
//            for(int i=vertices.size()-vertCount;i<vertices.size();i++){
//                ofs2<<i<<" "<<id<<" 1"<<endl;
//            }
//            vertCount = 0;
//        }
//        else if(dump=="f"){
//            ss>>a>>b>>c;
//            faces.push_back({a-1,b-1,c-1});
//        }
//    }ifs.close(); ofs2.close();
//    cout<<"Imported "+fileName<<endl;
//    ofstream ofs("skel.ply");
//    ofs<<"ply"<<endl;
//    ofs<<"format ascii 1.0"<<endl;
//    ofs<<"comment Exported by RapidForm"<<endl;
//    ofs<<"element vertex "<<vertices.size()<<endl;
//    ofs<<"property float x"<<endl;
//    ofs<<"property float y"<<endl;
//    ofs<<"property float z"<<endl;
//    ofs<<"element face "<<faces.size()<<endl;
//    ofs<<"property list uchar int vertex_index"<<endl;
//    ofs<<"end_header"<<endl;
//    for(G4ThreeVector v:vertices)
//        ofs<<v.getX()<<" "<<v.getY()<<" "<<v.getZ()<<endl;
//    for(auto f:faces)
//        ofs<<"3 "<<f[0]<<" "<<f[1]<<" "<<f[2]<<endl;
//    ofs.close();
//    cout<<"skel.ply was generated"<<endl;

//    exit(2);
//}

//map<int, Vec3> ReadJointF(string jointF){
//    ifstream ifs(jointF);
//    if(!ifs.is_open()){
//        cout<<jointF<<" is not open"<<endl;
//        exit(1);
//    }
//    int id; double x, y, z;
//    map<int, Vec3> jointCenters;
//    while(ifs>>id>>x>>y>>z)
//        jointCenters[id]=Vec3(x,y,z);
//    cout<<"Imported "<<jointCenters.size()<<" joint centers"<<endl;
//    return jointCenters;
//}

//map<int, int> ReadKinectJoint(string kinectJointF){
//    ifstream ifs(kinectJointF);
//    if(!ifs.is_open()){
//        cout<<kinectJointF<<" is not open"<<endl;
//        exit(1);
//    }
//    int id,p_id; string dump;
//    map<int, int> parentJ;
//    while(ifs>>id>>dump>>p_id)
//        parentJ[id]=p_id;
//    cout<<"Imported "<<parentJ.size()<<" KINECT joint info."<<endl;
//    return parentJ;
//}

////Specification
//pair<G4TessellatedSolid*, G4TessellatedSolid*> ReadObj(string name, vector<Point3> &verts, map<int,vector<vector<int>>> &faces){
//    //initialization
//    verts.clear(); faces.clear();
//    //read file
//    ifstream ifs(name);
//    if(!ifs.is_open()) {cerr<<"There is no "+name<<endl; exit(1);}

//    cout<<"Reading "+name<<flush;
//    G4ThreeVector point;
//    int id, a, b, c;
//    vector<G4ThreeVector> vertVec;
//    string dump;
//    G4TessellatedSolid* tess0 = new G4TessellatedSolid();
//    G4TessellatedSolid* tess1 = new G4TessellatedSolid();
//    while(getline(ifs, dump)){
//        stringstream ss(dump);
//        ss>>dump;
//        if(dump=="v"){
//            ss>>point;
//            vertVec.push_back(point);
//        }
//        else if(dump=="g"){
//            ss>>id;
//            faces[id] = {};
//        }
//        else if(dump=="f"){
//            ss>>a>>b>>c;
//            faces[id].push_back({a, b, c});
//            if(id==0) tess0->AddFacet(new G4TriangularFacet(vertVec[a-1],vertVec[b-1],vertVec[c-1],ABSOLUTE));
//            else if(id==1) tess1->AddFacet(new G4TriangularFacet(vertVec[a-1],vertVec[b-1],vertVec[c-1],ABSOLUTE));
//        }
//    }ifs.close();
//    tess0->SetSolidClosed(true);
//    tess1->SetSolidClosed(true);
//    for(auto v:vertVec) verts.push_back(Point3(double(v.getX()), double(v.getY()), double(v.getZ())));
//    cout<<" - done"<<endl;
//    return make_pair(tess0, tess1);
//}


//void WriteEle(string name, vector<pair<vector<int>, double>> eleVol, vector<Point3> nodes, vector<vector<double>> weights){
//    ofstream ofs(name);
//    ofs<<eleVol.size()<<"  4  1"<<endl;
//    int count(0);
//    for(auto ele:eleVol){
//        if(nodes[ele.first[0]].z+nodes[ele.first[1]].z+nodes[ele.first[2]].z+nodes[ele.first[3]].z>8) continue;
//        int n(0);
//        double avg(0.);
//        for(int i=0;i<4;i++){
//            if(weights[ele.first[i]][0]==0) continue;
//            if(weights[ele.first[i]][0]==1) continue;
//            avg += weights[ele.first[i]][0];
//            n++;
//        }
//        if(n>0) avg/=n;
//        else avg = weights[ele.first[0]][0];
//        ofs<<count<<" "<<ele.first[0]<<" "<<ele.first[1]<<" "<<ele.first[2]<<" "<<ele.first[3]<<" "<<int(avg*100)<<endl;count++;
//    }
//    ofs.close();
//}

//void SetWei#endifghts(const vector<Point3> verts, vector<vector<double>> &weights){
//    cout<<"Setting weights"<<flush;
//    weights.clear();
///*    vector<Point3> verts0, verts1;
//    vector<vector<int>> faces0, faces1;
//    ReadPly("0.ply", verts0, faces0);
//    ReadPly("1.ply", verts1, faces1);
//    G4TessellatedSolid tess0, tess1;
//    vector<G4ThreeVector> verts0_g4, verts1_g4;
//    for(auto vert:verts0) verts0_g4.push_back(G4ThreeVector(vert.x, vert.y, vert.z));
//    for(auto vert:verts1) verts1_g4.push_back(G4ThreeVector(vert.x, vert.y, vert.z));

//    for(auto face:faces0){
//        tess0.AddFacet(new G4TriangularFacet(verts0_g4[face[0]],
//                                             verts0_g4[face[1]],
//                                             verts0_g4[face[2]],
//                                             ABSOLUTE));
//    }tess0.SetSolidClosed(true);
//    for(auto face:faces1){
//        tess1.AddFacet(new G4TriangularFacet(verts1_g4[face[0]],
//                                             verts1_g4[face[1]],
//           #endif                                  verts1_g4[face[2]],
//                                             ABSOLUTE));
//    }tess1.SetSolidClosed(true);
//    for(auto vert:verts){
//        if(vert.y>5) {weights.push_back({0,1}); continue;}
//        if(vert.y<-5) {weights.push_back({1,0}); continue;}
//        G4ThreeVector point(vert.x, vert.y, vert.z);
//        double d0_in = tess0.DistanceToIn(point);
//        double d0_out = tess0.DistanceToOut(point);
//        double d0 = (d0_in>d0_out)? d0_in:d0_out;
//        double d1_in = tess1.DistanceToIn(point);
//        double d1_out = tess1.DistanceToOut(point);
//        double d1 = (d1_in>d1_out)? d1_in:d1_out;
////        cout<<vert<<scientific<<": "<<d0<<" "<<d1;getchar();
//        d0 *= d0; d1 *= d1;
//        double r = d0/(d0+d1);
//        cout<<point.getY()<<"\t"<<r<<endl;
// //        cout<<r*r<<endl; getchar();

//        weights.push_back({1-r, r});
//    }*/

//    double margin(5);
//    double constant = -0.5/(margin*margin);
//    for(auto vert:verts){
//        if(vert.y>margin) {weights.push_back({0,1}); continue;}
//        if(vert.y<-margin) {weights.push_back({1,0}); continue;}
//        if(vert.y>0){
//           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
//           weights.push_back({1-w, w});
//        }else{
//            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
//            weights.push_back({w, 1-w});
//        }
//    }
//    cout<<"-done"<<endl;
//}


//double DistanceTo(vector<Point3> verts, Point3 point)
//{
//    double distance(3);
//    for(auto v:verts){
//        Vec3 vec = point-v;
//        if(abs(vec.y)>distance) continue;
//        if(abs(vec.x)>distance) continue;
//        if(abs(vec.z)>distance) continue;
//        double dist2=vec.norm_squared();
//        if(dist2<distance*distance) distance = sqrt(dist2);
//    }
//    return distance;
//}

//void SetWeights2(const vector<Point3> verts, vector<vector<double>> &weights, map<int,vector<vector<int>>> &faces, pair<G4TessellatedSolid*, G4TessellatedSolid*>tessPair)
//{
//    cout<<"Setting weights"<<flush;
//    weights.clear();

//    //quad
//    double margin(5);
//    double constant = -0.5/(margin*margin);
//    for(auto vert:verts){
//        if(vert.y>margin) {weights.push_back({0,1}); continue;}
//        if(#endifvert.y<-margin) {weights.push_back({1,0}); continue;}
//        if(vert.y>0){
//           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
//           weights.push_back({1-w, w});
//        }else{
//            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
//            weights.push_back({w, 1-w});
//        }
//    }

//    //vert ext.
//    vector<int> verts0, verts1;
//    for(auto face:faces[0]){verts0.push_back(face[0]);verts0.push_back(face[1]);verts0.push_back(face[2]);}
//    for(auto face:faces[1]){verts1.push_back(face[0]);verts1.push_back(face[1]);verts1.push_back(face[2]);}
//    sort(verts0.begin(), verts0.end());
//    verts0.erase(unique(verts0.begin(), verts0.end()), verts0.end());
//    sort(verts1.begin(), verts1.end());
//    verts1.erase(unique(verts1.begin(), verts1.end()), verts1.end());
//    for(auto &id:verts0)id--;
//    for(auto &id:verts1)id--;

//    //bone rigid
//    for(auto id:verts0)weights[id] = {1,0};
//    for(auto id:verts1)weights[id] = {0,1};

//    vector<Point3> upperVec, lowerVec;
//    for(auto id:verts0)upperVec.push_back(verts[id]);
//    for(auto id:verts1)lowerVec.push_back(verts[id]);

//    //gradation
//    set<int> rigidNodes(verts0.begin(), verts0.end());
//    rigidNodes.insert(verts1.begin(), verts1.end());
//    for(size_t n=0;n<verts.size();n++){
//        if(rigidNodes.find(n)!=rigidNodes.end()) continue;
//        double upDist = DistanceTo(upperVec, verts[n]);
//        double lowDist = DistanceTo(lowerVec, verts[n]);
//        if(upDist<2 && lowDist<2){
//            double ratio = 1./(upDist+lowDist);
//            weights[n] = {lowDist*ratio, upDist*ratio};
//        }#endif
//        else if(upDist<2){
//            double alpha = (2-upDist)*0.5;
//            weights[n][0] *= (1.-alpha);
//            weights[n][1] *= (1.-alpha);
//            weights[n][0] += alpha;
//        }
//        else if(lowDist<2){
//            double alpha = (2-lowDist)*0.5;
//            weights[n][0] *= (1.-alpha);
//            weights[n][1] *= (1.-alpha);
//            weights[n][1] += alpha;
//        }
//    }


//    cout<<"-done"<<endl;
//}

//void SetWeights3(const vector<Point3> nodes, vector<vector<double>> &weights, map<int, vector<int>> attributes)
//{
//    cout<<"Setting weights"<<flush;
//    weights.clear();

//    //quad
//    double margin(5);
//    double constant = -0.5/(margin*margin);
//    for(auto vert:nodes){
//        if(vert.y>margin) {weights.push_back({0,1}); continue;}
//        if(vert.y<-margin) {weights.push_back({1,0}); continue;}
//        if(vert.y>0){
//           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
//           weights.push_back({1-w, w});
//        }else{
//            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
//            weights.push_back({w, 1-w});
//        }
//    }

//    //bone rigid
//    int upperBone(2), lowerBone(3);
//    for(auto id:attributes[upperBone])weights[id] = {1,0};
//    for(auto id:attributes[lowerBone])weights[id] = {0,1};

//    vector<Point3> upperVec, lowerVec;
//    for(auto id:attributes[upperBone])upperVec.push_back(nodes[id]);
//    for(auto id:attributes[lowerBone])lowerVec.push_back(nodes[id]);

//    //gradation
//    set<int> rigidNodes(attributes[upperBone].begin(), attributes[upperBone].end());
//    rigidNodes.insert(attributes[lowerBone].begin(), attributes[lowerBone].end());
//    for(size_t n=0;n<nodes.size();n++){
//        if(rigidNodes.find(n)!=rigidNodes.end()) continue;
//        double upDist = DistanceTo(upperVec, nodes[n]);
//        double lowDist = DistanceTo(lowerVec, nodes[n]);
//        if(upDist<2 && lowDist<2){
//            double ratio = 1./(upDist+lowDist);
//            weights[n] = {lowDist*ratio, upDist*ratio};
//        }
//        else if(upDist<2){
//            double alpha = (2-upDist)*0.5;
//            weights[n][0] *= (1.-alpha);
//            weights[n][1] *= (1.-alpha);
//            weights[n][0] += alpha;
//        }
//        else if(lowDist<2){
//            double alpha = (2-lowDist)*0.5;
//            weights[n][0] *= (1.-alpha);
//            weights[n][1] *= (1.-alpha);
//            weights[n][1] += alpha;
//        }
//    }

//    cout<<"-done"<<endl;
//}

//std::vector<std::map<int, double>> ReadWeights(string fileName){
//    ifstream ifs(fileName);
//    string dump;
//    stringstream ss;
//    std::map<int, double> wMap;
//    std::vector<std::map<int, double>> weights;
//    double cut(1e-7);
//    while(getline(ifs, dump)){
//        ss.clear(); wMap.clear();
//        ss.str(dump);
//        int id; double w, sum(0);
//        ss>>dump;
//        while(ss>>id){
//            ss>>w;
//            if(w<cut) continue;
//            sum += w;
//            wMap[id] = w;
//        }
//        for(auto &w:wMap) w.second /= sum;
//        weights.push_back(wMap);
//    }ifs.close();

//    return weights;
//}

//std::vector<std::map<int, double>> ReadWeightMat(string fileName){
//    double cut(1e-7);
//    ifstream ifs(fileName);
//    string dump;
//    stringstream ss;
//    std::map<int, double> wMap;
//    std::vector<std::map<int, double>> weights;
//    while(ge#endiftline(ifs, dump)){
//        ss.clear(); wMap.clear();
//        ss.str(dump);
//        int id(0); double w;
//        double sum(0);
//        while(ss>>w){
//            if(w>cut){
//                wMap[id] = w;
//                sum += w;
//            }
//            id++;
//        }
//        for(auto &w:wMap) w.second /= sum;
//        weights.push_back(wMap);
//    }ifs.close();
//    return weights;
//}

//void ReadPly(string name, vector<Point3> &verts, vector<vector<int>> &faces)
//{
//    //initialization
//    verts.cl#endifear(); faces.clear();

//    //read file
//    ifstream ifs(name);
//    if(!ifs.is_open()) {cerr<<"There is no "<<name<<endl; exit(1);}

//    Point3 point;
//    string dump;
//    int nb_verts(0);
//    int nb_faces(0);
//    while(1){
//        getline(ifs, dump);
//        if(dump.find("element vertex")!=string::npos){
//            stringstream ss(dump);
//            ss>>dump>>dump>>nb_verts;
//        }
//        else if(dump.find("element face")!=string::npos){
//            stringstream ss(dump);
//            ss>>dump>>dump>>nb_faces;
//        }
//        else if(dump.find("end_header")!=string::npos) break;
//    }
//    cout<<"Start reading "+name+"...(v:"<<nb_verts<<"/f:"<<nb_faces<<")"<<flush;
//    for(int i=0;i<nb_verts;i++){
//        ifs>#endif>point;
//        verts.push_back(point);
//    }
//    for(int i=0;i<nb_faces;i++){
//        int a, b, c;
//        ifs>>dump>>a>>b>>c;
//        vector<int> face = {a, b, c};
//        faces.push_back(face);
//    }
//    ifs.close();
//    cout<<"-done"<<endl;
//}

//void PrintPly(string name, vector<Point3> verts, vector<vector<int>> faces)
//{
//    ofstream ofs(name);
//    ofs<<"ply"<<endl;
//    ofs<<"format ascii 1.0"<<endl;
//    ofs<<"comment Exported by RapidForm"<<endl;
//    ofs<<"element vertex "<<verts.size()<<endl;
//    ofs<<"property double x"<<endl;
//    ofs<<"property double y"<<endl;
//    ofs<<"property double z"<<endl;
//    ofs<<"element face "<<faces.size()<<endl;
//    ofs<<"property list uchar int vertex_index"<<endl;
//    ofs<<"end_header"<<endl;
//    for(Point3 vert:verts) ofs<<vert<<endl;
//    for(vector<int> face:faces) ofs<<"3 "<<face[0]<<" "<<face[1]<<" "<<face[2]<<endl;
//    ofs.close();
//    cout<<"Printed "+name<<endl;
//}#endif

//void ReadNode(string name, vector<Point3> &nodes)
//{
//    nodes.clear();
//    ifstream ifsNode(name);
//    if(!ifsNode.is_open()) {cerr<<"There is no "<<name<<endl; exit(1);}
//    int nb_node;
//    string dump;
//    getline(ifsNode, dump);
//    stringstream ss(dump);
//    ss>>nb_node;
//    Point3 point;
//    for(int i=0;i<nb_node;i++){
//        ifsNode>>dump>>point;
//        nodes.push_back(point);
//    }ifsNode.close();
//}

//void ReadEle(s#endiftring name, map<int, vector<int>> &attributes)
//{
//    attributes.clear();
//    ifstream ifsEle(name);
//    if(!ifsEle.is_open()){cerr<<"There is no "<<name<<endl; exit(1);}
//    int nb_ele;
//    string dump;
//    getline(ifsEle, dump);
//    stringstream ss(dump);
//    ss>>nb_ele;
//    int a, b, c, d, id(-100);
//    for(int i=0;i<nb_ele;i++){
//        getline(ifsEle, dump);
//        stringstream ss1(dump);
//        ss1>>dump>>a>>b>>c>>d>>id;
//        if(attributes.find(id)==attributes.end()) attributes[id]={a, b, c, d};
//        else {
//            attributes[id].push_back(a);
//            attributes[id].push_back(b);
//            attributes[id].push_back(c);
//            attributes[id].push_back(d);
//        }
//    }ifsEle.close();
//    for(auto &att:attributes){
//        sort(att.second.begin(), att.second.end());
//        att.second.erase(unique(att.second.begin(), att.second.end()), att.second.end());
//    }
//}

//void ReadEle2(string name, vector<Point3> nodes, vector<pair<vector<int>, double>> &eleVol, map<int, vector<int>> &attributes)
//{
//    attributes.clear();
//    eleVol.clear();
//    ifstream ifsEle(name);
//    if(!ifsEle.is_open()){cerr<<"There is no "<<name<<endl; exit(1);}
//    int nb_ele;
//    string dump;
//    getline(ifsEle, dump);
//    stringstream ss(dump);
//    ss>>nb_ele;
//    int a, b, c, d, id(-100);
//    for(int i=0;i<nb_ele;i++){
//        getline(ifsEle, dump);
//        stringstream ss1(dump);
//        ss1>>dump>>a>>b>>c>>d>>id;
//        Vec3 aa = nodes[a]-nodes[d];
//        Vec3 bb = nodes[b]-nodes[d];
//        Vec3 cc = nodes[c]-nodes[d];
//        double vol = (aa.cross(bb)).dot(cc);
//        vector<int> elements = {a, b, c, d};
//        eleVol.push_back(make_pair(elements, vol));
//        if(attributes.find(id)==attributes.end()) attributes[id]={a, b, c, d};
//        else {
//            attributes[id].push_back(a);
//            attributes[id].push_back(b);
//            attributes[id].push_back(c);
//            attributes[id].push_back(d);
//        }
//    }ifsEle.close();
//    for(auto &att:attributes){
//        sort(att.second.begin(), att.second.end());
//        att.second.erase(unique(att.second.begin(), att.second.end()), att.second.end());
//    }
//}

//void DetectTangled(const vector<Point3> nodes, vector<pair<vector<int>, double>> eleVol)
//{
//    double volDiff(0);
//    double tangledVol(0);
//    int count(0);
//    for(auto ele:eleVol){
//        Vec3 aa = nodes[ele.first[0]] - nodes[ele.first[3]];
//        Vec3 bb = nodes[ele.first[1]] - nodes[ele.first[3]];
//        Vec3 cc = nodes[ele.first[2]] - nodes[ele.first[3]];
//        double vol = (aa.cross(bb)).dot(cc);
//        if(ele.second*vol>0) volDiff+=abs(vol)-abs(ele.second);
//        else {count++; tangledVol+=vol/6.;}
//    }
//    cout<<"   Tangled: "<<count<<", "<<tangledVol<<"   "<<"Volume Diff: "<<volDiff/6.<<endl;;
//}

//void PrintNode(string name, const vector<Point3> nodes)
//{
//    ofstream ofsNode(name+".node");
//    ofsNode<<nodes.size()<<"  3  0  0"<<endl;
//    for(size_t i=0;i<nodes.size();i++){
//        ofsNode<<i<<" "<<nodes[i]<<endl;
//    }ofsNode.close();
//}


//void PrintObj(string name, const vector<Point3> verts, const map<int,vector<vector<int>>> faces)
//{
//    ofstream ofs(name+".obj");
//    for(auto v:verts)ofs<<"v "<<v<<endl;
//    ofs<<endl;
//    for(auto shell:faces){
//        ofs<<"g "<<shell.first<<endl<<endl;
//        for(auto face:shell.second)
//            ofs<<"f "<<face[0]<<" "<<face[1]<<" "<<face[2]<<endl;
//        ofs<<endl;
//    }ofs.close();
//}#endif

//void dual_quat_deformer(const std::vector<Point3>& in_verts,
//                        std::vector<Point3>& out_verts,
//                        Dual_quat_cu& dual_quat,
//                        const std::vector< std::vector<double> >& weights)
//{
//    out_verts.clear();
//    for(unsigned v = 0; v < in_verts.size(); ++v)
//    {
//        double w0 = weights[v][0];
//        double w1 = weights[v][1];
//        if( dual_quat.rotation().dot( Dual_quat_cu::identity().rotation() ) < 0.f )
//            w1 *= -1.f;
//        Dual_quat_cu dq_blend = Dual_quat_cu::identity()*w0 + dual_quat*w1;

//        // Compute animated position
//        Point3 vi = dq_blend.transform( in_verts[v] );
//        out_vert#endifs.push_back(vi);
//    }
//}

//void dual_quat_deformer(const std::vector<Point3>& in_verts,
//                        std::vector<Point3>& out_verts,
//                        std::map<int, Dual_quat_cu>& dual_quat,
//                        const std::vector<std::map<int, double>>& weights)
//{
//    out_verts.clear();
//    for(unsigned v = 0; v < in_verts.size(); ++v)
//    {
//        Dual_quat_cu dq_blend;
//        bool first(true);
//        Quat_cu q0;

//        for(auto w:weights[v]){
//            if(first){
//                #endifdq_blend = dual_quat[w.first] * w.second;
//                q0 = dual_quat[w.first].rotation();
//                first = false;
//                continue;
//            }
//            if( dual_quat[w.first].rotation().dot( q0 ) < 0.f )
//                dq_blend = dq_blend + dual_quat[w.first] * (-w.second);
//            else dq_blend = dq_blend + dual_quat[w.first] * w.second;
//        }

//        // Compute animated position
//        Point3 vi = dq_blend.transform( in_verts[v] );
//        out_verts.push_back(vi);
//    }
//}

//void dual_quat_deformer(const std::vector<Point3>& in_verts,
//                        std::vector<Point3>& out_verts,
//                        std::map<int, Dual_quat_cu>& dual_quat,
//                        const std::vector<std::map<int, double>>& weights,
//                        std::map<int, Vec3>& newCenters)
//{
//    out_verts.clear();
//    for(unsigned v = 0; v < in_verts.size(); ++v)
//    {
//        Dual_quat_cu dq_blend;
//        bool first(true);
//        Quat_cu q0;

//        for(auto w:weights[v]){
//            if(first){
//                dq_blend = dual_quat[w.first] * w.second;
//                q0 = dual_quat[w.first].rotation();
//                first = false;
//                continue;
//            }
//            if( dual_quat[w.first].rotation().dot( q0 ) < 0.f )
//                dq_blend = dq_blend + dual_quat[w.first] * (-w.second);
//            else dq_blend = dq_blend + dual_quat[w.first] * w.second;
//        }

//        // Compute animated position
//        Point3 vi = dq_blend.transform( in_verts[v] );
//        out_verts.push_back(vi);
//    }
//}

//Mat3 G4Mat2TBXMat(G4RotationMatrix mat){
//    return Mat3(mat.xx(),mat.xy(), mat.xz(),mat.yx(),mat.yy(),mat.yz(),mat.zx(),mat.zy(),mat.zz());
//}

//#include "linmath.h"
//Transfo GetRotMatrix(Vec3 from, Vec3 to) {
//    Vec3 axis = from.cross(to); axis.normalize();
//    double theta = acos(from.dot(to) / (from.norm()*to.norm()));
//    return Transfo::rotate(axis,theta);
//}


//map<int, Transfo> CalculateScalingVec(map<int, Vec3> jCen,
//                                      map<int, Vec3>& newCen,
//                                   map<int, int> parentJ,
//                                   map<int, double> lengths){
//    newCen = jCen;
//    map<int, Transfo> scalingTF;
//    for(auto j:jCen) scalingTF[j.first] = Transfo::identity();
//    vector<int> group = {1,2};
//    for(int id:group){
//        Vec3 orientation = jCen[id]-jCen[parentJ[id]]; //orientation from parent
//        orientation.normalize();
//        newCen[id] = newCen[parentJ[id]] + lengths[id]*orientation;
//    }

//    //torso
//    double xz_scale0 = (lengths[18]+lengths[22]) / (jCen[18]-jCen[22]).norm();
//    double y_scale0 = lengths[1] / (jCen[1]-jCen[0]).norm();
//    scalingTF[0] = Transfo::scale(Vec3(xz_scale0, y_scale0, xz_scale0));

//    double xz_scale2 = (newCen[5]-newCen[12]).norm() / (jCen[5]-jCen[12]).norm();
//    double xz_scale1 = (xz_scale0+xz_scale2)*0.5;
//    double y_scale1 = lengths[2] / (jCen[1]-jCen[2]).norm();
//    scalingTF[1] = Transfo::translate(newCen[1]-jCen[1])*Transfo::scale(jCen[1], Vec3(xz_scale1, y_scale1, xz_scale1));

//    double y_scale2 = (lengths[3]) / (jCen[2]-jCen[3]).norm();
//    Transfo tf2 = Transfo::translate(newCen[2]-jCen[2])*Transfo::scale(jCen[2],Vec3(xz_scale2, y_scale2, xz_scale2));
//    scalingTF[2] = tf2;
//    group = {3, 4, 5, 11, 12};
//    for(int id:group){
//        Vec3 orientation = jCen[id]-jCen[parentJ[id]]; //orientation from parent
//        orientation.normalize();
//        newCen[id] = tf2*jCen[id];
//        scalingTF[id] = tf2;
//    }

//    //limbs
//    group = {6, 7, 13, 14, 19, 20, 23, 24, 26};
//    map<int,double> limbSF;
//    for(auto id:group){
//        Vec3 orientation = jCen[id]-jCen[parentJ[id]]; //orientation from parent
//        double l = orientation.normalize();

//        newCen[id] = newCen[parentJ[id]] + lengths[id]*orientation;

//        double r = (lengths[id]/l)-1;
//        Vec3 s = Vec3(1,1,1)+orientation*r; //sf for parent ID
//        limbSF[parentJ[id]] = r*100;
//        scalingTF[#endifparentJ[id]] = Transfo::translate(newCen[parentJ[id]]-jCen[parentJ[id]])*Transfo::scale(jCen[parentJ[id]],s);
//    }

//    //extrimities
//    group = {7, 14, 20, 24, 26};
//    for(auto id:group){
//        scalingTF[id] = Transfo::translate(newCen[id]-jCen[id]);
//    }

//    cout<<endl<<"<Scaling factors>"<<endl;
//    cout.precision(3);
//    cout<<"lower torso  : xz "<<(xz_scale0-1)*100<<" % / y "<<(y_scale0-1)*100<<" %"<<endl;
//    cout<<"middle torso : xz "<<(xz_scale1-1)*100<<" % / y "<<(y_scale1-1)*100<<" %"<<endl;
//    cout<<"upper Torso  : xz "<<(xz_scale2-1)*100<<" % / y "<<(y_scale2-1)*100<<" %"<<endl;
//    cout<<"neck         : "<<limbSF[3]<<" %"<<endl;
//    cout<<"head         : translation only"<<endl;
//    cout<<"lower arm    : R  "<<limbSF[13]<<" % / L "<<limbSF[6]<<" %"<<endl;
//    cout<<"hand         : translation only"<<endl;
//    cout<<"upper leg    : R  "<<limbSF[22]<<" % / L "<<limbSF[18]<<" %"<<endl;
//    cout<<"lower leg    : R  "<<limbSF[23]<<" % / L "<<limbSF[19]<<" %"<<endl;
//    cout<<"foot         : translation only"<<endl;
//    return scalingTF;
//}

//void ScaleToActor(const vector<Point3>& in_verts,
//                  vector<Point3>& out_verts,
//                  map<int, Transfo> scalingTF,
//                  const std::vector<std::map<int, double>>& weights){
//    out_verts.resize(in_verts.size());
//    for(int i=0;i<in_verts.size();i++){
//        Transfo tf(0);
//        for(auto w:weights[i]) tf = tf+ scalingTF[w.first]*w.second;
//        //if(tf.m[0]!=1) {cout<<tf;getchar();}
//        out_verts[i] = tf*in_verts[i];
//    }
//}


//            ofs<<"f "<<face[0]<<" "<<face[1]<<" "<<face[2]<<endl;
//        ofs<<endl;
//    }ofs.close();
//}

//void dual_quat_deformer(const std::vector<Point3>& in_verts,
//                        std::vector<Point3>& out_verts,
//                        Dual_quat_cu& dual_quat,
//                        const std::vector< std::vector<double> >& weights)
//{
//    out_verts.clear();
//    for(unsigned v = 0; v < in_verts.size(); ++v)
//    {
//        double w0 = weights[v][0];
//        double w1 = weights[v][1];
//        if( dual_quat.rotation().dot( Dual_quat_cu::identity().rotation() ) < 0.f )
//            w1 *= -1.f;
//        Dual_quat_cu dq_blend = Dual_quat_cu::identity()*w0 + dual_quat*w1;

//        // Compute animated position
//        Point3 vi = dq_blend.transform( in_verts[v] );
//        out_verts.push_back(vi);
//    }
//}

//void dual_quat_deformer(const std::vector<Point3>& in_verts,
//                        std::vector<Point3>& out_verts,
//                        std::map<int, Dual_quat_cu>& dual_quat,
//                        const std::vector<std::map<int, double>>& weights)
//{
//    out_verts.clear();
//    for(unsigned v = 0; v < in_verts.size(); ++v)
//    {
//        Dual_quat_cu dq_blend;
//        bool first(true);
//        Quat_cu q0;

//        for(auto w:weights[v]){
//            if(first){
//                dq_blend = dual_quat[w.first] * w.second;
//                q0 = dual_quat[w.first].rotation();
//                first = false;
//                continue;
//            }
//            if( dual_quat[w.first].rotation().dot( q0 ) < 0.f )
//                dq_blend = dq_blend + dual_quat[w.first] * (-w.second);
//            else dq_blend = dq_blend + dual_quat[w.first] * w.second;
//        }

//        // Compute animated position
//        Point3 vi = dq_blend.transform( in_verts[v] );
//        out_verts.push_back(vi);
//    }
//}

//void dual_quat_deformer(const std::vector<Point3>& in_verts,
//                        std::vector<Point3>& out_verts,
//                        std::map<int, Dual_quat_cu>& dual_quat,
//                        const std::vector<std::map<int, double>>& weights,
//                        std::map<int, Vec3>& newCenters)
//{
//    out_verts.clear();
//    for(unsigned v = 0; v < in_verts.size(); ++v)
//    {
//        Dual_quat_cu dq_blend;
//        bool first(true);
//        Quat_cu q0;

//        for(auto w:weights[v]){
//            if(first){
//                dq_blend = dual_quat[w.first] * w.second;
//                q0 = dual_quat[w.first].rotation();
//                first = false;
//                continue;
//            }
//            if( dual_quat[w.first].rotation().dot( q0 ) < 0.f )
//                dq_blend = dq_blend + dual_quat[w.first] * (-w.second);
//            else dq_blend = dq_blend + dual_quat[w.first] * w.second;
//        }

//        // Compute animated position
//        Point3 vi = dq_blend.transform( in_verts[v] );
//        out_verts.push_back(vi);
//    }
//}

//Mat3 G4Mat2TBXMat(G4RotationMatrix mat){
//    return Mat3(mat.xx(),mat.xy(), mat.xz(),mat.yx(),mat.yy(),mat.yz(),mat.zx(),mat.zy(),mat.zz());
//}

//#include "linmath.h"
//Transfo GetRotMatrix(Vec3 from, Vec3 to) {
//    Vec3 axis = from.cross(to); axis.normalize();
//    double theta = acos(from.dot(to) / (from.norm()*to.norm()));
//    return Transfo::rotate(axis,theta);
//}


//map<int, Transfo> CalculateScalingVec(map<int, Vec3> jCen,
//                                      map<int, Vec3>& newCen,
//                                   map<int, int> parentJ,
//                                   map<int, double> lengths){
//    newCen = jCen;
//    map<int, Transfo> scalingTF;
//    for(auto j:jCen) scalingTF[j.first] = Transfo::identity();
//    vector<int> group = {1,2};
//    for(int id:group){
//        Vec3 orientation = jCen[id]-jCen[parentJ[id]]; //orientation from parent
//        orientation.normalize();
//        newCen[id] = newCen[parentJ[id]] + lengths[id]*orientation;
//    }

//    //torso
//    double xz_scale0 = (lengths[18]+lengths[22]) / (jCen[18]-jCen[22]).norm();
//    double y_scale0 = lengths[1] / (jCen[1]-jCen[0]).norm();
//    scalingTF[0] = Transfo::scale(Vec3(xz_scale0, y_scale0, xz_scale0));

//    double xz_scale2 = (newCen[5]-newCen[12]).norm() / (jCen[5]-jCen[12]).norm();
//    double xz_scale1 = (xz_scale0+xz_scale2)*0.5;
//    double y_scale1 = lengths[2] / (jCen[1]-jCen[2]).norm();
//    scalingTF[1] = Transfo::translate(newCen[1]-jCen[1])*Transfo::scale(jCen[1], Vec3(xz_scale1, y_scale1, xz_scale1));

//    double y_scale2 = (lengths[3]) / (jCen[2]-jCen[3]).norm();
//    Transfo tf2 = Transfo::translate(newCen[2]-jCen[2])*Transfo::scale(jCen[2],Vec3(xz_scale2, y_scale2, xz_scale2));
//    scalingTF[2] = tf2;
//    group = {3, 4, 5, 11, 12};
//    for(int id:group){
//        Vec3 orientation = jCen[id]-jCen[parentJ[id]]; //orientation from parent
//        orientation.normalize();
//        newCen[id] = tf2*jCen[id];
//        scalingTF[id] = tf2;
//    }

//    //limbs
//    group = {6, 7, 13, 14, 19, 20, 23, 24, 26};
//    map<int,double> limbSF;
//    for(auto id:group){
//        Vec3 orientation = jCen[id]-jCen[parentJ[id]]; //orientation from parent
//        double l = orientation.normalize();

//        newCen[id] = newCen[parentJ[id]] + lengths[id]*orientation;

//        double r = (lengths[id]/l)-1;
//        Vec3 s = Vec3(1,1,1)+orientation*r; //sf for parent ID
//        limbSF[parentJ[id]] = r*100;
//        scalingTF[parentJ[id]] = Transfo::translate(newCen[parentJ[id]]-jCen[parentJ[id]])*Transfo::scale(jCen[parentJ[id]],s);
//    }

//    //extrimities
//    group = {7, 14, 20, 24, 26};
//    for(auto id:group){
//        scalingTF[id] = Transfo::translate(newCen[id]-jCen[id]);
//    }

//    cout<<endl<<"<Scaling factors>"<<endl;
//    cout.precision(3);
//    cout<<"lower torso  : xz "<<(xz_scale0-1)*100<<" % / y "<<(y_scale0-1)*100<<" %"<<endl;
//    cout<<"middle torso : xz "<<(xz_scale1-1)*100<<" % / y "<<(y_scale1-1)*100<<" %"<<endl;
//    cout<<"upper Torso  : xz "<<(xz_scale2-1)*100<<" % / y "<<(y_scale2-1)*100<<" %"<<endl;
//    cout<<"neck         : "<<limbSF[3]<<" %"<<endl;
//    cout<<"head         : translation only"<<endl;
//    cout<<"lower arm    : R  "<<limbSF[13]<<" % / L "<<limbSF[6]<<" %"<<endl;
//    cout<<"hand         : translation only"<<endl;
//    cout<<"upper leg    : R  "<<limbSF[22]<<" % / L "<<limbSF[18]<<" %"<<endl;
//    cout<<"lower leg    : R  "<<limbSF[23]<<" % / L "<<limbSF[19]<<" %"<<endl;
//    cout<<"foot         : translation only"<<endl;
//    return scalingTF;
//}

//void ScaleToActor(const vector<Point3>& in_verts,
//                  vector<Point3>& out_verts,
//                  map<int, Transfo> scalingTF,
//                  const std::vector<std::map<int, double>>& weights){
//    out_verts.resize(in_verts.size());
//    for(int i=0;i<in_verts.size();i++){
//        Transfo tf(0);
//        for(auto w:weights[i]) tf = tf+ scalingTF[w.first]*w.second;
//        //if(tf.m[0]!=1) {cout<<tf;getchar();}
//        out_verts[i] = tf*in_verts[i];
//    }
//}

