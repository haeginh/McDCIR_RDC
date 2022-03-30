#include "MapContainer.hh"

MapContainer::MapContainer()
: i(80), j(80), k(60) //X-decalcomanie
// : i(30), j(60), k(60) //X-decalcomanie
{
    baseIdx = (i*j + j + 1)*k*0.5;
    // SetIJK(30, 60, 60);
    // isoCenter  = Vector3d(0,150,150); //isocenter when doseMap generated. (grid center)
    
    doseMapS.resize(i*j*k);
    doseMapL.resize(i*j*k);

    ReadMapList();
    //read default map
    ReadMap("../McDCIR_PDC_map/doseMaps/200_200_150/ISO_100.map");
    // ReadMap("../4_map_gen/doseMaps/0.map");
}

void MapContainer::ReadMapList(){
    //read map list
    ifstream ifsList("./doseMaps/list.txt");
    int mapID,mapkVp,mapRot; double mapDAP;
    while(ifsList>>mapID>>mapkVp>>mapRot>>mapDAP){
        mapList[MAPIDX(mapkVp, mapRot)] = mapID;
        mapDAPs[mapID] = mapDAP*1.e-12;
    }
}

void MapContainer::ReadMap(string name)
{
    // Vector3d isoCenter = Vector3d(0,0,60); // data_iso
    // gridStart = isoRelat+isoCenter; // isocenter + isorelat
    invDAP = 1./mapDAPs[0];
    ifstream ifsMap(name, ios::binary);
    ifsMap.read((char*) &doseMapS(0), i*j*k*sizeof(double));
    ifsMap.read((char*) &doseMapL(0), i*j*k*sizeof(double));
    ifsMap.close();
    maxSkin = doseMapS.maxCoeff();

    static char rMaxChar[100]("");
    static char aMaxChar[100]("1");
}