#include <iostream>
#include <ctime>
#include <functional>

#include "PhantomAnimator.hh"
// #include "Viewer.hh"
// #include "MapContainer.hh"
# define PI 3.14159265358979323846

void PrintUsage(){
    cout<<"<Usage>"<<endl;
    cout<<"./DCIR_RDC [phantom prefix (ply, tgf)]"<<endl;
    exit(1);
}

using namespace Eigen;
using namespace std;
typedef Triplet<double> T;

int main(int argc, char** argv){
    if(argc!=2) PrintUsage();
    
    return EXIT_SUCCESS;
}
