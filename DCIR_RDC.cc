#include <iostream>
#include <ctime>
#include <functional>

#include "PhantomAnimator.hh"
#include "Viewer.hh"
#include "MapContainer.hh"
#include "Config.hh"
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
    Config config;
    if(!config.loadConfig("config.txt"))
    {
        cout<<"There is no config.txt -> default options will be used"<<endl;
        config.saveConfig("config.txt");
    }

    //phantom animator
    PhantomAnimator *phantom = new PhantomAnimator(config.getPhantomFile());
    phantom->Initialize();
  
    //libigl viewer
    Viewer *viewer = new Viewer(phantom, config.getIpAdress(), config.getPortNum());
    viewer->SetIsoCenter(config.getIsoCenter());
    viewer->SetMeshes(config.getCArmFile(), config.getPatientFile(), config.getGlassFile());
    viewer->SetCores();
    viewer->Launch();

    return EXIT_SUCCESS;
}
