#include <iostream>
#include <ctime>
#include <functional>

#include "PhantomAnimator.hh"
#include "Viewer.hh"
#include "MapContainer.hh"
#define PI 3.14159265358979323846

void PrintUsage()
{
    cout << "<Usage>" << endl;
    cout << "./DCIR_RDC [phantom prefix (ply, tgf)]" << endl;
    exit(1);
}

using namespace Eigen;
using namespace std;
typedef Triplet<double> T;

int main(int argc, char **argv)
{
    if (argc != 2)
        PrintUsage();

    //phantom animator
    PhantomAnimator *phantom = new PhantomAnimator(string(argv[1]));
    phantom->Initialize();

    //charuco mark
    // Quaterniond q;
    // Vector3d tvec;
    // string dump;
    // ifstream ifs("test.txt");
    // ifs >> dump >> q.x() >> q.y() >> q.z() >> q.w();
    // ifs >> dump >> tvec(0) >> tvec(1) >> tvec(2);
    // ifs.close();
    // Affine3d coordAff = Affine3d::Identity();
    // q.normalize();
    // coordAff.rotate(q.toRotationMatrix().transpose());
    // coordAff.translate(-tvec);

    // Affine3d charucoAff = Affine3d::Identity();
    // charucoAff.translate(tvec);
    // charucoAff.rotate(q);
    
    //libigl viewer
    Viewer *viewer = new Viewer(phantom);
    // viewer->SetCharucoAff(charucoAff);
    // viewer->SetCoordAff(coordAff);
    viewer->SetMeshes();
    viewer->SetCores();

    // cout<<coordAff.matrix() * charucoAff.matrix()<<endl;

    viewer->Launch();

    return EXIT_SUCCESS;
}
