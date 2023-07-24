#include "MapContainer.hh"
#include <igl/slice.h>
#include <igl/Timer.h>
#include <iomanip>
#include <igl/readDMAT.h>

MapContainer::MapContainer(float _fl)
:fl(_fl), volInv(1.3/125000.)
{
    mapDir = string(getenv("DCIR_MAP_DIR")) + "/maps/";
    specDir = string(getenv("DCIR_MAP_DIR")) + "/spec/";
    //GRID INFO.
    RowVector3i size(400, 400, 300);
    base0 = size.cast<float>()*0.5;
    base = base0;
    int gridSize = 5;
    gridConv = 1./(double) gridSize; //inverse of grid size
    numGrid = size/gridSize;

    totalNum = numGrid(0)*numGrid(1)*numGrid(2);
    skinMapList.resize(totalNum, 12);
    lensMapList.resize(totalNum, 12);

    //c-arm name
    cArmName[make_pair(0, 0)] = "01";
    cArmName[make_pair(0, -30)] = "02";
    cArmName[make_pair(0, 30)] = "03";
    cArmName[make_pair(-30, 0)] = "04";
    cArmName[make_pair(-30, -30)] = "05";
    cArmName[make_pair(-30, 30)] = "06";
    cArmName[make_pair(30, 0)] = "07";
    cArmName[make_pair(30, -30)] = "08";
    cArmName[make_pair(30, 30)] = "09";

    //spot coord
    spotCoord.resize(3, 2);
    spotCoord.row(0) = RowVector2f(-3, -30); //lung
    spotCoord.row(1) = RowVector2f(-3, -10); //sto
    spotCoord.row(2) = RowVector2f(-3, 6); //pel
    spotName = {"lung", "sto", "pel"}; //should be changed!!

    //FD
    fdName = {"/01", "/02", "/03", "/04", "/05"};

    //default map
    ReadDAPdata(specDir + "DAP.dat");
    // skinMap.resize(totalNum);
    // lensMap.resize(totalNum);
    // ReadMap("RAO10CRA40_box.map", skinMap, lensMap);
    // skinMap*=(1./dapMap[69]);
    // lensMap*=(1./dapMap[69]);
    // ReadSpectrum(specDir + "68.spec");
    igl::readDMAT(mapDir + "sto/0401.sMap", skinMapList);
    igl::readDMAT(mapDir + "sto/0401.lMap", lensMapList);
    ReadSpectrum(specDir + "60.spec");
    skinMap*=(1./dapMap[60]);
    lensMap*=(1./dapMap[60]);
}

void MapContainer::ReadSpectrum(string name)
{
    ifstream ifs(name);
    if(!ifs.is_open())
    {
        cout<<"failed to read "+ name<<endl;
        return;
    }
    string dump;
    // skinMap = ArrayXf::Zero(totalNum);
    // lensMap = ArrayXf::Zero(totalNum);
    VectorXf spec = VectorXf::Zero(12); 
    while (getline(ifs, dump))
	{
		stringstream ss(dump);
		ss >> dump;
		if (dump == "Brem[uGy/mAs@1m]")
		{
			double brem, chara;
			ifs>>brem>>chara;
			// factorInv = 1./(brem + chara)*1e6; //mAs/Gy@1m
		}
		else if (dump == "Energy[keV]")
		{
			while (getline(ifs, dump)) //interval 10
			{
				float energy(0.), intensity(0.);
				stringstream ss2(dump);
				ss2 >> energy >> intensity;
                int idx = max(int(floor(energy*0.1+0.5)-1), 0);
                spec(idx) += intensity; // #/(cm2 mAs) @1m
			}
		}
	}
	ifs.close(); 
    // skinMap *= volInv*spec.sum();//Gy (/ DAP)
    // #/(Gy cm2) @1m : 1/DAP
    spec = spec / spec.sum();
    skinMap = skinMapList * spec; //Gy / DAP
    lensMap = lensMapList * spec;
    return;
}

bool MapContainer::ReadMap(string name, ArrayXf& _skinMap, ArrayXf& _lensMap)
{
    // invDAP = 1./mapDAPs[0];
    ifstream ifsMap(name, ios::binary);
    if(!ifsMap.is_open())
    {
        cout<<"failed to read "+ name<<endl;
        return false;
    }
    ifsMap.read((char*) &_skinMap(0), totalNum*sizeof(float));
    ifsMap.read((char*) &_lensMap(0), totalNum*sizeof(float));
    ifsMap.close();
    _skinMap *= volInv; 
    _lensMap *= volInv; 
    return true;
}

//table, fd
bool MapContainer::SetCArm(int kVp, RowVector3f cArm, RowVector3f tableTrans, int fd)
{
    int rot1 = floor(cArm(0)/30. + 0.5) * 30;
    int ang1 = floor(cArm(1)/30. + 0.5) * 30;
    int kVp1 = floor(kVp*0.2+0.5) * 5;
    
    Index idx;
    (spotCoord.rowwise() - tableTrans.leftCols(2)).rowwise().squaredNorm().minCoeff(&idx);
    int spot1 = idx; //chk

    int fd1 = SelectFD(cArm(0), cArm(1), tableTrans(2) - 23, cArm(2), fd);

    if((rot0!=rot1) || (ang0!=ang1) || (spot0!=spot1) || (fd0!=fd1)) //if rotation changes
    {
        rot0=rot1; ang0=ang1; kVp0=kVp1; spot0 = spot1; fd0 = fd1;

        string name = spotName[spot1]+fdName[fd1]+cArmName[make_pair(rot1, ang1)];
        igl::readDMAT(mapDir + name + ".sMap", skinMapList);
        igl::readDMAT(mapDir + name + ".lMap", lensMapList);
  
        ReadSpectrum(specDir + to_string(kVp0)+".spec");
        skinMap *= (1./dapMap[kVp]);
        lensMap *= (1./dapMap[kVp]);
        cout<<"ROT "<<setw(5)<<rot1<<" ANG "<<setw(5)<<ang1<<setw(5)<<kVp1<<" kVp"<<setw(5)<<fd1<<" FD - spot"<<spot1<<endl;
    }
    else if(kVp0!=kVp1) //if peak volatage changes only
    {
        cout<<"ROT "<<setw(5)<<rot1<<" ANG "<<setw(5)<<ang1<<setw(5)<<kVp1<<" kVp"<<setw(5)<<fd1<<" FD"<<endl;
        kVp0=kVp1;
        ReadSpectrum(specDir + to_string(kVp0)+".spec");
        skinMap *= (1./dapMap[kVp]);
        lensMap *= (1./dapMap[kVp]);
    }

    cArm_IT = (CarmRotation(rot0, ang0) * CarmRotation(cArm(0), cArm(1)).inverse()).transpose();
    base = base0 + RowVector3f(0, 0, -tableTrans(2)+5);
    return true;
}

void MapContainer::SetDoseRate(const MatrixXf &U, VectorXd &D, double dap)
{
    MatrixXi gridM = ((U.rowwise() + base)*cArm_IT*gridConv).array().floor().cast<int>();
    // MatrixXi gridM = ((U.rowwise()+base0)*gridConv).array().floor().cast<int>();
    
    ArrayXi idx = gridM.col(0) * numGrid(2) * numGrid(1) + gridM.col(1) * numGrid(2) + gridM.col(2);
    ArrayXi in = ((idx.array()>0) * (idx.array()<totalNum)).cast<int>();
    D = D.array() * igl::slice(skinMap, idx*in, 1).cast<double>() * in.cast<double>() * dap; //Gy/s
}

int MapContainer::SelectFD(float rot, float ang, float tableZ, float sid, int fd)
{
    float cosTheta = (CarmRotation(rot, ang)*Vector3f(0, 0, -1)).dot(Vector3f(0, 0, -1));
    float l = fl + (tableZ*cosTheta); //focal spot to table
    float diag = float(fd) * (l/sid); //diag. at bed height
    if(fd!=48)
    {
        Index idx;
        (Array3f(14.142, 21.2132, 28.28427) - diag).abs2().minCoeff(&idx);
        return int(idx);
    }
    else
    {
        if(diag<28.67) return 3;
        else return 4;
    }

}

void MapContainer::ReadDAPdata(string name)
{
    ifstream ifs(name);
    if(!ifs.is_open())
    {
        cout<<name <<" is not open!!!"<<endl;
        exit(100);
    }
    string dump;
    getline(ifs, dump);
    int kVp;
    double dap;
    for(int i=0;i<120;i++)
    {
        ifs>>kVp>>dap>>dump>>dump>>dump;
        dapMap[kVp] = dap;
    }
    ifs.close();
    return;
}