#ifndef RDCWindow_class
#define RDCWindow_class

#include <igl/opengl/glfw/Viewer.h>
#include <igl/embree/EmbreeIntersector.h>
#include <igl/project.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "PhantomAnimator.hh"
#include "MapContainer.hh"
#include "Communicator.hh"
#include "Config.hh"

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
    static RDCWindow &Instance()
    {
        static RDCWindow RDCWindow;
        return RDCWindow;
    }
private:
    RDCWindow();

public:
    // create phantom classes
    void SetConfig(Config config);
    // setting window frame
    void InitializeGUI();
    void Launch()
    {
        viewer.launch(true, false, "DCIR System (RDC module)");
    }
    bool PreDrawFunc(igl::opengl::glfw::Viewer &_viewer);
    void SetBodyID(int id) { bodyID = id; }

    Matrix3f GetCarmRotation(double rot, double ang)
    {
        return (AngleAxisf(rot / 180. * igl::PI, Vector3f(0, 1, 0)) *
                AngleAxisf(ang / 180. * igl::PI, Vector3f(1, 0, 0)))
            .matrix();
    }

    void DeletePhantom(int i)
    {
        delete(indivPhantoms[i]);
        indivPhantoms.erase(i);
        phantomDataID.erase(i);
    }
    // bool beamOn;
    DataSet currentFrame, prevFrame;
    DataSet manualFrame;
    bool recording;
    int loadNum; bool playOnce;bool keepCurr;
    vector<vector<float>> recordData;
    MatrixXd recordVal;
    igl::opengl::glfw::Viewer viewer;

    // data id
    unsigned int patient_data, table_data, externalOBJ_data,
        cArm_data, beam_data, glass_data, phantomAcc_data, grid_data, axis_data, sphere_data, extra_data;
 
    // vector<unsigned int> phantom_data;
    // int mainID() { return phantom_data[bodyID]; }

    MatrixXd V_cArm, V_beam, V_glass, V_patient, V_table, V_sphere;
    MatrixXi F_table;
    int det_v, det_f;

    bool show_C, show_BE;
    bool show_leadGlass, show_beam;

    // bool SetMap(int kVp, int rot, int ang, RowVector3f tableTrans, int sid, int fd)
    // {
    //     return mapContainer->SetCArm(kVp, rot, ang, tableTrans, sid, fd);
    // }
    // void SetDoseRate(const MatrixXf &U, VectorXd &D, double dap)
    // {
    //     mapContainer->SetDoseRate(U, D, dap);
    // }
    // void SetDAP(double dap) { mapContainer->SetDAP(dap); }
    // VectorList accD;

    unsigned int v_left, v_middle;//, v_right;

    // shadow-related
    MatrixXf B_patient, N_patient, B_patient1;//, N_patient, N_patient1;
    VectorXd A_patient;//, A_patient1;
    VectorXi sourceIdx;
    void CalculateSourceFacets(RowVector3f trans, Matrix3f rot, float dist2 = 400)
    {
        B_patient1 = B_patient.rowwise() + trans;
        VectorXi idx = (B_patient1.rowwise().cross(rot * Vector3f(0, 0, 1)).rowwise().squaredNorm().array() < dist2).cast<int>();
        sourceIdx.resize(idx.sum());
        for (int i = 0, n = 0; i < idx.rows(); i++)
            if (idx(i) > 0) sourceIdx(n++) = i;

        B_patient1 = igl::slice(B_patient1, sourceIdx,1);
    }

    MatrixXf InitTree(igl::embree::EmbreeIntersector &ei, const MatrixXd &extraV, const MatrixXi &extraF)
    {
        MatrixXi totalF=viewer.data(cArm_data).F.bottomRows(det_f).array() + (-viewer.data(cArm_data).V.rows()+det_v);
        MatrixXf totalV=viewer.data(cArm_data).V.bottomRows(det_v).cast<float>();
        if(use_curtain)
        {
            totalF.conservativeResize(totalF.rows()+2, 3);
            totalF.bottomRows(2) = F_table.bottomRows(2).array()+totalV.rows()-V_table.rows()+4;
            totalV.conservativeResize(totalV.rows()+4, 3);
            totalV.bottomRows(4) = viewer.data(table_data).V.bottomRows(4).cast<float>();
        }
        if(extraF.rows())
        {
            totalF.conservativeResize(totalF.rows()+extraF.rows(), 3);
            totalF.bottomRows(extraF.rows()) = extraF.array()+totalV.rows();
            totalV.conservativeResize(totalV.rows()+extraV.rows(), 3);
            totalV.bottomRows(extraV.rows()) = extraV.cast<float>();
        }
        if(sourceIdx.rows())
        {
            totalF.conservativeResize(totalF.rows()+sourceIdx.rows(), 3);
            totalF.bottomRows(sourceIdx.rows()) = igl::slice(viewer.data(patient_data).F, sourceIdx, 1).array()+totalV.rows();
            totalV.conservativeResize(totalV.rows()+V_patient.rows(), 3);
            totalV.bottomRows(V_patient.rows()) = V_patient.cast<float>().rowwise() + currentFrame.bed;
        }
        if(currentFrame.glassChk)
        {
            totalF.conservativeResize(totalF.rows() + viewer.data(glass_data).F.rows(), 3);
            totalF.bottomRows(viewer.data(glass_data).F.rows()) = viewer.data(glass_data).F.array() + totalV.rows();
            totalV.conservativeResize(totalV.rows()+ V_glass.rows(), 3);
            totalV.bottomRows(V_glass.rows()) = viewer.data(glass_data).V.cast<float>();
        }

        VectorXi doseCalV;
        for(auto id:currentFrame.bodyMap)
        {
            PhantomAnimator* phantom;
            if(indivPhantoms.find(id.first)!=indivPhantoms.end())
                phantom = indivPhantoms[id.first];
            else continue; //extras
            MatrixXd C;
            int numF = phantom->F.rows();
            int numV = phantom->V.rows();
            if(phantom->useApron)
            {
                doseCalV.conservativeResize(doseCalV.rows() + phantom->outApron.rows());
                doseCalV.bottomRows(phantom->outApron.rows()) = phantom->outApron.array() + totalV.rows();
            }
            else
            {
                doseCalV.conservativeResize(doseCalV.rows() + numV);
                doseCalV.bottomRows(numV) = VectorXi::LinSpaced(numV, 0, numV-1).array() + totalV.rows();
            }
            totalF.conservativeResize(totalF.rows()+numF, 3);
            totalF.bottomRows(numF) = phantom->F.array() + totalV.rows();
            totalV.conservativeResize(totalV.rows()+numV, 3);
            totalV.bottomRows(numV) = phantom->U.cast<float>();
        }

        if(externalOBJ_data>=0 && viewer.data(externalOBJ_data).V.rows()>=0)
        {
            int numF = viewer.data(externalOBJ_data).F.rows();
            int numV = viewer.data(externalOBJ_data).V.rows();

            doseCalV.conservativeResize(doseCalV.rows() + numV);
            doseCalV.bottomRows(numV) = VectorXi::LinSpaced(numV, 0, numV-1).array() + totalV.rows();

            totalF.conservativeResize(totalF.rows() + numF, 3);
            totalF.bottomRows(viewer.data(externalOBJ_data).F.rows()) = viewer.data(externalOBJ_data).F.array() + totalV.rows();
            totalV.conservativeResize(totalV.rows()+viewer.data(externalOBJ_data).V.rows(), 3);
            totalV.bottomRows(numV) = viewer.data(externalOBJ_data).V.cast<float>();
        }

        ei.init(totalV, totalF);
        return igl::slice(totalV,doseCalV,1);
    }

    //naive + dot production exception
//     VectorXd GenerateShadow(const igl::embree::EmbreeIntersector &ei, const MatrixXd &V, const MatrixXd &N)
//     {
//         VectorXd S = VectorXd::Zero(V.rows());
//         VectorXd dot0 = ((-V).array() * N.array()).rowwise().sum();
// #pragma omp for
//         for (int i = 0; i < V.rows(); i++)
//         {
//             // if (dot0(i) < 0)  continue;
//             MatrixXf dir = (-B_patient1).rowwise() + V.row(i).cast<float>();
//             VectorXf dot = (N_patient1.array() * dir.array()).rowwise().sum();
//             igl::Hit hit;
//             for (int j = 0; j < B_patient1.rows(); j++)
//             {
//                 // if (dot(j) < 0) continue;
//                 if (!ei.intersectSegment(B_patient1.row(j), dir.row(j)*0.999, hit))
//                 {
//                     S(i) += A_patient1(j);
//                 }
//             }
//         }
//         return S;
//     }

    //sphere shadow map
    MatrixXd GenerateShadow(const igl::embree::EmbreeIntersector &ei, const MatrixXf &totalV, double grid = 30., double r = 250., double noShadow2=625)
    {
        double bound = grid * sqrt(3.) / r*0.5;
        VectorXd shadow = VectorXd::Zero(totalV.rows()); // 1:bright, 0:dark
        MatrixXf sph(totalV.rows()*sourceIdx.rows(), 3);
        MatrixXf D(totalV.rows(), sourceIdx.rows());
        VectorXf distToOrigin = totalV.rowwise().squaredNorm();
        for (int i = 0; i<sourceIdx.rows(); i++)
        {
            sph.block(totalV.rows()*i, 0, totalV.rows(), 3) = ProjectToSphericalRoom(B_patient.row(sourceIdx(i)), totalV, grid, r);
            D.col(i) = (totalV.rowwise()-B_patient.row(sourceIdx(i))).rowwise().norm();
        }
        map<tuple<int, int, int, int>, float> shadowMap;
        igl::Hit hit;
        MatrixXf N_patient1 = igl::slice(N_patient,sourceIdx,1);
        MatrixXf A_patient1 = igl::slice(A_patient,sourceIdx,1).cast<float>();
#pragma omp for
        for (int i = 0; i < totalV.rows(); i++)
        {
            if(distToOrigin(i)<noShadow2) {shadow(i) = 1; continue;}
            ArrayXf dot = -((B_patient1.rowwise() - totalV.row(i)).array() * N_patient1.array()).rowwise().sum();
            dot = dot * A_patient1.array() * (dot>0).cast<float>();
            // dot = A_patient1.array() * dot;
            double tot = dot.sum();
            if(tot==0) continue;
            for (int b = 0; b < B_patient1.rows(); b++)
            {
                if(dot(b)==0) continue;
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
                    shadow(i) += dot(b);
            }
            shadow(i)/=tot;
        }
        return shadow;
    }

    MatrixXf ProjectToSphericalRoom(RowVector3f O, const MatrixXf &V, double grid, double r)
    {
        MatrixXf V1 = V.rowwise() - O;
        MatrixXf sph = V1.array().colwise() * (V1.rowwise().norm().cwiseInverse()).array() * r;
        return (((1. / grid) * sph).array() + 0.5).floor();
    }

    int numStaff;
    map<int, pair<int, bool>> workerIdData; //accepts only 2set of color
    // bool connectPDC;
    clock_t lastFrameT;
    // thread PDCsender;
    // void SendPDCData();
    // int PDCprocess;
    // clock_t lastFrameT_PDC;
    int bodyID;
    bool stop, show_externalDose, force_withDAP,force_manualBeam, use_curtain;
    VectorXd externalD;
    float bodyDelayTol;
    PhantomAnimator * GetMainPhantomHandle(){
        return GetPhantom(bodyID);
    }
    PhantomAnimator *GetPhantom(int i)
    {
        if(indivPhantoms.find(i)==indivPhantoms.end()) return nullptr;
        return indivPhantoms[i];
    }
    PhantomAnimator *AddNewPhantom(int i)
    {
        if(indivPhantoms.find(i)!=indivPhantoms.end())
        {
            cout<<"PHANTOM #"<<i<<" already exists!"<<endl;
        }
        else indivPhantoms[i] = new PhantomAnimator;
        return indivPhantoms[i];
    }
    map<int, pair<int, int>> phantomDataID;
    vector<string> profileNames;
    vector<map<int, double>> jointLengths;
    vector<RowVector3d> eyeL_vec, eyeR_vec;
    bool extraChk, animBeamOnOnly;
private:
    bool SetPhantoms(string fileName);
    void SetMeshes(string dir);
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    igl::opengl::glfw::imgui::ImGuiMenu mainMenu;
    // string phantomName;

    // view id

    // vector<PhantomAnimator *> indivPhantoms;
    map<int, PhantomAnimator*> indivPhantoms;
    PhantomAnimator* extraPhantom;
    MapContainer *mapContainer;

    //something else
    vector<float> SetAframeData(DataSet data, float frameTimeInMSEC);

    map<int, MatrixXd> fdSize;
public:
    DataSet ReadAframeData(vector<float> frameData);
};

#endif