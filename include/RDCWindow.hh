#ifndef RDCWindow_class
#define RDCWindow_class

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
// #include <igl/embree/ambient_occlusion.h>
#include <igl/embree/EmbreeIntersector.h>
#include <igl/project.h>
// #include <igl/opengl/glfw/imgui/ImGuizmoPlugin.h>
#include <imgui/imgui.h>
#include "PhantomAnimator.hh"
#include "MapContainer.hh"
#include "Communicator.hh"

using namespace std;
using namespace Eigen;

typedef std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>
    VectorList;
typedef std::vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf>>
    MatrixList;
class RDCWindow
{
public:
    // singleton
    static RDCWindow &Instance();
    // setting window frame
    void Initialize();
    void Launch()
    {
        viewer.launch(true, false, "DCIR System (RDC module)");
        viewer.selected_data_index = phantom_data[bodyID];
    }
    bool PreDrawFunc(igl::opengl::glfw::Viewer &_viewer);
    void SetBodyID(int id) { bodyID = id; }

    Matrix3f GetCarmRotation(double rot, double ang)
    {
        return (AngleAxisf(rot / 180. * igl::PI, Vector3f(0, 1, 0)) *
                AngleAxisf(ang / 180. * igl::PI, Vector3f(1, 0, 0)))
            .matrix();
    }

    PhantomAnimator *GetPhantom(int i)
    {
        if (i < indivPhantoms.size())
            return indivPhantoms[i];
        else
            return nullptr;
    }
    // bool beamOn;
    bool useManualBeam;
    bool recording;
    int loadNum;
    vector<vector<float>> recordData;
    bool postureUpdated;
    igl::opengl::glfw::Viewer viewer;

    // data id
    unsigned int apron_data, patient_data, table_data,
        cArm_data, beam_data, glass_data, phantomAcc_data, grid_data, axis_data, sphere_data;
    vector<unsigned int> phantom_data;
    int mainID() { return phantom_data[bodyID]; }

    MatrixXd V_cArm, V_beam, V_glass, V_patient, V_table, V_sphere;

    bool show_C, show_BE;
    bool show_leadGlass, show_beam;

    bool SetMap(int kVp, int rot, int ang)
    {
        return mapContainer->SetCArm(kVp, rot, ang);
    }
    void SetDose(const MatrixXf &U, VectorXd &D)
    {
        mapContainer->SetDose(U, D);
    }
    void SetDAP(double dap) { mapContainer->SetDAP(dap); }
    // VectorList accD;

    unsigned int v_left, v_middle, v_right;

    // shadow-related
    MatrixXf B_patient, B_patient1, N_patient, N_patient1;
    VectorXd A_patient, A_patient1;
    void CalculateSourceFacets(RowVector3f trans, Matrix3f rot, float dist2 = 200)
    {
        B_patient1 = B_patient.rowwise() + trans;
        VectorXi idx = (B_patient1.rowwise().cross(rot * Vector3f(0, 0, 1)).rowwise().squaredNorm().array() < dist2).cast<int>();
        VectorXi sourceIdx(idx.sum());
        for (int i = 0, n = 0; i < idx.rows(); i++)
            if (idx(i) > 0) sourceIdx(n++) = i;
        B_patient1 = igl::slice(B_patient1, sourceIdx, 1);
        N_patient1 = igl::slice(N_patient, sourceIdx, 1);
        A_patient1 = igl::slice(A_patient, sourceIdx, 1);
     
        // A_patient1 = A_patient1/A_patient1.sum(); 
    }

    MatrixXf InitTree(igl::embree::EmbreeIntersector &ei,  map<int, Body> bodyMap)
    {
        MatrixXi totalF;// = viewer.data(patient_data).F;
        MatrixXf totalV;// = viewer.data(patient_data).V.cast<float>();
        
        for(auto id:bodyMap)
        {
            int numF = indivPhantoms[id.first]->F.rows();
            int numV = indivPhantoms[id.first]->V.rows();
            totalF.conservativeResize(totalF.rows()+numF, 3);
            totalF.bottomRows(numF) = indivPhantoms[id.first]->F.array() + totalV.rows();
            totalV.conservativeResize(totalV.rows()+numV, 3);
            totalV.bottomRows(numV) = indivPhantoms[id.first]->U.cast<float>();
        }
        ei.init(totalV, totalF);
        return totalV;
    }

    //naive + dot production exception
    VectorXd GenerateShadow(const igl::embree::EmbreeIntersector &ei, const MatrixXd &V, const MatrixXd &N)
    {
        VectorXd S = VectorXd::Zero(V.rows());
        VectorXd dot0 = ((-V).array() * N.array()).rowwise().sum();
#pragma omp for
        for (int i = 0; i < V.rows(); i++)
        {
            // if (dot0(i) < 0)  continue;
            MatrixXf dir = (-B_patient1).rowwise() + V.row(i).cast<float>();
            VectorXf dot = (N_patient1.array() * dir.array()).rowwise().sum();
            igl::Hit hit;
            for (int j = 0; j < B_patient1.rows(); j++)
            {
                // if (dot(j) < 0) continue;
                if (!ei.intersectSegment(B_patient1.row(j), dir.row(j)*0.999, hit))
                {
                    S(i) += A_patient1(j);
                }
            }
        }
        return S;
    }

    //sphere shadow map
    MatrixXd GenerateShadow(const igl::embree::EmbreeIntersector &ei, const MatrixXf &totalV, double grid = 30., double r = 250., double noShadow2=625)
    {
        double bound = grid * sqrt(3.) / r*0.5;
        VectorXd shadow = VectorXd::Zero(totalV.rows()); // 1:bright, 0:dark
        MatrixXf sph(totalV.rows()*B_patient1.rows(), 3);
        MatrixXf D(totalV.rows(), B_patient1.rows());
        VectorXf distToOrigin = totalV.rowwise().squaredNorm();
        for (int i = 0; i<B_patient1.rows(); i++)
        {
            sph.block(totalV.rows()*i, 0, totalV.rows(), 3) = ProjectToSphericalRoom(B_patient1.row(i), totalV, grid, r);
            D.col(i) = (totalV.rowwise()-B_patient1.row(i)).rowwise().norm();
        }
        map<tuple<int, int, int, int>, float> shadowMap;
        igl::Hit hit;
        MatrixXf test = MatrixXf::Zero(B_patient1.rows(), 3);
        test.row(2) = VectorXf::Ones(B_patient1.rows());
#pragma omp for
        for (int i = 0; i < totalV.rows(); i++)
        {
            if(distToOrigin(i)<noShadow2) {shadow(i) = 1; continue;}
            ArrayXf dot = ((B_patient1.rowwise() - totalV.row(i)).array() * N_patient1.array()).rowwise().sum();
            double tot = (A_patient1.array() * (dot<0).cast<double>()).sum();
            if(tot==0) continue;
            for (int b = 0; b < B_patient1.rows(); b++)
            {
                if(dot(b)>0) continue;
                int row = totalV.rows()*b + i;
                auto p = make_tuple(b, sph(row, 0), sph(row, 1), sph(row, 2));
                // Generate shadowmap for initially called map idx
                if (shadowMap.find(p) == shadowMap.end())
                {
                    if (!ei.intersectRay(B_patient1.row(b), sph.row(row), hit))
                        shadowMap[p] = r;
                    else
                        shadowMap[p] = hit.t* sph.row(row).norm();
                }
                if (D(i, b) - shadowMap[p] < bound * D(i, b))
                    shadow(i) += A_patient1(b);
            }

            if(tot>0) shadow(i)/=tot;
        }
        return shadow;
    }

    MatrixXf ProjectToSphericalRoom(RowVector3f O, const MatrixXf &V, double grid, double r)
    {
        MatrixXf V1 = V.rowwise() - O;
        MatrixXf sph = V1.array().colwise() * (V1.rowwise().norm().cwiseInverse()).array() * r;
        return (((1. / grid) * sph).array() + 0.5).floor();
    }

    int maxNumPeople;
    int dataSize;
    // bool connectPDC;
    clock_t lastFrameT;
    // thread PDCsender;
    // void SendPDCData();
    // int PDCprocess;
    // clock_t lastFrameT_PDC;
private:
    RDCWindow() : maxNumPeople(5), bodyID(0), recording(false), loadNum(-1), show_beam(true), show_leadGlass(true)
    {
        if(getenv("DCIR_MAX_NUM_OF_PEOPLE")) 
            maxNumPeople = atoi(getenv("DCIR_MAX_NUM_OF_PEOPLE"));
        dataSize = 18+maxNumPeople*145;
        for (int i = 0; i < maxNumPeople; i++)
        {
            auto phantom = new PhantomAnimator;
            indivPhantoms.push_back(phantom);
        }
        mapContainer = new MapContainer;
        SetMap(80, 0, 0); //default map
    }
    void SetMeshes(string dir);
    igl::opengl::glfw::imgui::ImGuiMenu mainMenu;
    // string phantomName;

    // view id

    int bodyID;
    vector<PhantomAnimator *> indivPhantoms;
    MapContainer *mapContainer;

    //something else
    vector<float> SetAframeData(DataSet data, float frameTimeInMSEC);
    DataSet ReadAframeData(vector<float> frameData);
};

#endif