#include "MapContainer.hh"
#include <igl/slice.h>
#include <igl/Timer.h>

MapContainer::MapContainer()
:dap(0.02), mapDir("./doseMaps/"), specDir("../McDCIR_PDC_map/spectra/")
// : i(30), j(60), k(60) //X-decalcomanie
{
    //GRID INFO.
    RowVector3i size(400, 400, 300);
    base = size.cast<float>()*0.5;
    int gridSize = 5;
    gridConv = 1./(double) gridSize; //inverse of grid size
    numGrid = size/gridSize;

    totalNum = numGrid(0)*numGrid(1)*numGrid(2);

    for(int i=1;i<11;i++)
    {
        ArrayXf skinMap(totalNum), lensMap(totalNum);
        ReadMap(mapDir+"R0A0_"+to_string(i)+"0keV.map", skinMap, lensMap);
        skinMapList.push_back(skinMap);
        lensMapList.push_back(lensMap);
    }
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
    skinMap = ArrayXf::Zero(totalNum);
    lensMap = ArrayXf::Zero(totalNum);
    while (getline(ifs, dump))
	{
		stringstream ss(dump);
		ss >> dump;
		if (dump == "Brem[uGy/mAs@1m]")
		{
			double brem, chara;
			ifs>>brem>>chara;
			factorInv = 1/(brem + chara)*1e6; //mAs/Gy@1m
		}
		else if (dump == "Energy[keV]")
		{
			while (getline(ifs, dump))
			{
				double energy, intensity;
				stringstream ss2(dump);
				ss2 >> energy >> intensity;
                skinMap += skinMapList[floor(energy*0.1+0.5)]*intensity;
                lensMap += lensMapList[floor(energy*0.1+0.5)]*intensity;
			}
		}
	}
	ifs.close();
    skinMap *= factorInv;
    lensMap *= factorInv;
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

bool MapContainer::SetCArm(int kVp, int rot, int ang)
{
    int rot1 = floor(rot/30. + 0.5) * 30;
    int ang1 = floor(ang/30. + 0.5) * 30;
    int kVp1 = floor(kVp*0.2+0.5) * 5;

    if(rot0!=rot1 || ang0!=ang1) //if rotation changes
    {
        rot0=rot1; ang0=ang1; kVp0=kVp1;
        for(int i=0;i<10;i++)
        {
            //erase abs for non-decalcomanie
            ReadMap(mapDir+"R"+to_string(abs(rot0))+"A"+to_string(ang0)+"_"+to_string(i+1)+"0keV.map", skinMap, lensMap);
            skinMapList.push_back(skinMap);
            lensMapList.push_back(lensMap);
        }        
        ReadSpectrum(specDir + to_string(kVp0)+".spec");
    }
    else if(kVp0!=kVp1) //if peak volatage changes only
    {
        kVp0=kVp1;
        ReadSpectrum(specDir + to_string(kVp0)+".spec");
    }

    cArm_IT = (CarmRotation(rot0, ang0) * CarmRotation(rot, ang).inverse()).transpose();
    return true;
}

void MapContainer::SetDose(const MatrixXf &U, VectorXd &D)
{
    MatrixXi gridM;
    if(rot0>0) gridM = ((U.rowwise() + base)*cArm_IT*gridConv).array().floor().cast<int>();
    else{//erase for non-decalcomanie
        MatrixXf U1 = U;
        U1.col(0) *= -1.;
        gridM = ((U1.rowwise() + base)*cArm_IT*gridConv).array().floor().cast<int>();
    }
    ArrayXi idx = gridM.col(0) * numGrid(2) * numGrid(1) + gridM.col(1) * numGrid(2) + gridM.col(2);
    ArrayXi in = ((idx.array()>0) * (idx.array()<totalNum)).cast<int>();
    D = D.array() * igl::slice(skinMap, idx*in, 1).cast<double>() * in.cast<double>() * dap;
}