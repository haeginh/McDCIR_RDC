#include "MapContainer.hh"
#include <igl/slice.h>
#include <igl/Timer.h>
#include <iomanip>
#include <igl/readDMAT.h>

MapContainer::MapContainer(float _fl)
:fl(_fl), volInv(1./125000.)
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
    spotCoord.row(0) = RowVector2f(3, 5); //lung
    spotCoord.row(1) = RowVector2f(3, 27); //sto
    spotCoord.row(2) = RowVector2f(3, 43); //pel
    spotName = {"sto", "sto", "sto"}; //should be changed!!

    //FD
    fdName = {"/01", "/02", "/03", "/04", "/05"};

    //default map
    igl::readDMAT(mapDir + "sto/0101.sMap", skinMapList);
    igl::readDMAT(mapDir + "sto/0101.lMap", lensMapList);
    ReadSpectrum(specDir + "60.spec");
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
    double factorInv;
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
			factorInv = 1./(brem + chara)*1e6; //mAs/Gy@1m
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
    spec *= factorInv*volInv; // #/(Gy cm2) @1m : 1/DAP
    // cout<<skinMap.maxCoeff()<<endl;
    skinMap = skinMapList * spec; 
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
    return true;
}

//table, fd
bool MapContainer::SetCArm(int kVp, RowVector3f cArm, RowVector3f tableTrans, int fd)
{
    int rot1 = floor(cArm(0)/30. + 0.5) * 30;
    int ang1 = floor(cArm(1)/30. + 0.5) * 30;
    int kVp1 = floor(kVp*0.2+0.5) * 5;
    
    Index idx;
    (spotCoord.rowwise() - tableTrans.leftCols(2)).rowwise().squaredNorm().maxCoeff(&idx);
    int spot1 = idx; //chk

    int fd1 = SelectFD(cArm(0), cArm(1), tableTrans(2) - 18.4285, cArm(2), fd);

    if(rot0!=rot1 || ang0!=ang1 || spot0!=spot1 || fd0!=fd1) //if rotation changes
    {
        rot0=rot1; ang0=ang1; kVp0=kVp1;

        string name = spotName[spot1]+fdName[fd1]+cArmName[make_pair(rot1, ang1)];
        igl::readDMAT(mapDir + name + ".sMap", skinMapList);
        igl::readDMAT(mapDir + name + ".lMap", lensMapList);

        // for(int i=0;i<10;i++)
        // {
        //     //erase abs for non-decalcomanie
        //     ReadMap(mapDir+"R"+to_string(abs(rot0))+"A"+to_string(ang0)+"_"+to_string(i+1)+"0keV.map", skinMap, lensMap);
        //     skinMapList.push_back(skinMap);
        //     lensMapList.push_back(lensMap);
        // }        
        ReadSpectrum(specDir + to_string(kVp0)+".spec");
        cout<<"ROT "<<setw(5)<<rot1<<" ANG "<<setw(5)<<ang1<<setw(5)<<kVp1<<" kVp"<<setw(5)<<skinMap.maxCoeff()<<" /Gycm2(max)"<<endl;
    }
    else if(kVp0!=kVp1) //if peak volatage changes only
    {
        cout<<"ROT "<<setw(5)<<rot1<<" ANG "<<setw(5)<<ang1<<setw(7)<<kVp1<<" kVp"<<endl;
        kVp0=kVp1;
        ReadSpectrum(specDir + to_string(kVp0)+".spec");
    }

    cArm_IT = (CarmRotation(rot0, ang0) * CarmRotation(cArm(0), cArm(1)).inverse()).transpose();
    base = base0 + RowVector3f(0, 0, tableTrans(2));
    return true;
}

void MapContainer::SetDoseRate(const MatrixXf &U, VectorXd &D, double dap)
{
    MatrixXi gridM = ((U.rowwise() + base)*cArm_IT*gridConv).array().floor().cast<int>();
    // MatrixXi gridM;
    // if(rot0>0) gridM = ((U.rowwise() - base)*cArm_IT*gridConv).array().floor().cast<int>();
    // else{//erase for non-decalcomanie
    //     MatrixXf U1 = U;
    //     U1.col(0) *= -1.;
    //     gridM = ((U1.rowwise() - base)*cArm_IT*gridConv).array().floor().cast<int>();
    // }
    // cout<<gridM.colwise().maxCoeff()<<endl<<gridM.colwise().minCoeff()<<endl;
    ArrayXi idx = gridM.col(0) * numGrid(2) * numGrid(1) + gridM.col(1) * numGrid(2) + gridM.col(2);
    ArrayXi in = ((idx.array()>0) * (idx.array()<totalNum)).cast<int>();
    D = D.array() * igl::slice(skinMap, idx*in, 1).cast<double>() * in.cast<double>() * dap; //Gy/s
}

int MapContainer::SelectFD(float rot, float ang, float tableZ, float sid, int fd)
{
    float h = fabs(tableZ-15.);
    float cosTheta = (CarmRotation(rot, ang)*Vector3f(0, 0, -1)).dot(Vector3f(0, 0, -1));
    float l = fl - (h/cosTheta);
    float diag = float(fd) * (l/sid);
    if(fd!=48)
    {
        Index idx;
        (Array3f(17.281, 25.922, 34.563) - diag).abs2().minCoeff(&idx);
        return int(idx);
    }
    else
    {
        if(diag<35.030) return 3;
        else return 4;
    }

}