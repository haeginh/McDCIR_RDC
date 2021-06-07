#include "MapContainer.hh"

MapContainer::MapContainer(){

}

void MapContainer::ReadMaps(){
    //read map list
    ifstream ifsList("./doseMaps/list.txt");
    int mapID,mapkVp,mapRot; double mapDAP;
    typedef tuple<int, int> MAPIDX;
    map<MAPIDX, int> mapList;
    map<int, double> mapDAPs;
    while(ifsList>>mapID>>mapkVp>>mapRot>>mapDAP){
        mapList[MAPIDX(mapkVp, mapRot)] = mapID;
        mapDAPs[mapID] = mapDAP*1.e-12;
    }
    //read default map
    vector<double> doseMapS0, doseMapL0;
    int ijk[3] = {60,60,60};
    doseMapS0.resize(ijk[0]*ijk[1]*ijk[2]);
    doseMapL0.resize(ijk[0]*ijk[1]*ijk[2]);
    Vector3d isoCenter = Vector3d(0,0,60); // data_iso
    Vector3d isoRelat  = Vector3d(-150,-150,0)-Vector3d(0,0,60); //isocenter when doseMap generated. (gridstartPos - map_iso)
    Vector3d gridStart = isoRelat+isoCenter; // isocenter + isorelat
    double DAPperNPS = mapDAPs[0];
    ifstream ifsMap("./doseMaps/0_conf.map", ios::binary);
    ifsMap.read((char*) &doseMapS0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
    ifsMap.read((char*) &doseMapL0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
    ifsMap.close();
    double maxSkin0 = *max_element(doseMapS0.begin(),doseMapS0.end());

    static char rMaxChar[100]("");
    static char aMaxChar[100]("1");
}