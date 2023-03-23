#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cassert>
#include <float.h>

#include <cmath>
// #include <filesystem>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <ros/console.h>
using namespace std;


#if defined(WIN32) || defined(_WIN32)
  #include <windows.h>
#endif

#ifdef __APPLE__
  #include "TargetConditionals.h"
#endif

#include "getIntegratorSolver.h"
#include "generateMeshGraph.h"
#include "generateMassMatrix.h"

#include "massSpringSystem.h"
#include "massSpringSystemFromObjMeshConfigFile.h"
#include "massSpringStencilForceModel.h"

#include "forceModelAssembler.h"

#include "graph.h"
#include "configFile.h"

#include "listIO.h"
#include "matrixIO.h"

#include "integratorBase.h"
#include "integratorBaseSparse.h"
#include "implicitNewmarkSparse.h"

// config file
string configFilename;
char renderingMeshFilename[4096]="beam.obj";
char fixedVerticesFilename[4096]="constraints.bou";
char massSpringSystemObjConfigFilename[4096]="beam3_vox.massspring";
char initialPositionFilename[4096]="__none";
char initialVelocityFilename[4096]="__none";
char forceLoadsFilename[4096]="loads.csv";
char vegaConfigurationsFilename[4096]="vegaConfigurations.csv";
char momentRigidSectionLoadsFilename[4096]="moments.csv";
char momentRigidSectionLoadsTimedFilename[4096]="__none";
char solverMethod[4096]="implicitNewmark";
float dampingMassCoef; // Rayleigh mass damping
float dampingStiffnessCoef; // Rayleigh stiffness damping
float dampingLaplacianCoef = 0.0; // Laplacian damping (rarely used)
float deformableObjectCompliance = 0.0; // scales all user forces by the provided factor

// adjusts the stiffness of the object to cause all frequencies scale by the provided factor:
// keep it to 1.0 (except for experts)
float frequencyScaling = 1.0;
int maxIterations=1; // for implicit integration
double epsilon=0.0001; // for implicit integration
char backgroundColorString[4096] = "255 255 255";
int numInternalForceThreads=4;
int numSolverThreads=1;

// simulation
bool stopSimulation=false;
int syncTimestepWithGraphics=1;
float timeStep = 1.0 / 30;
float newmarkBeta = 0.25;
float newmarkGamma = 0.5;
int use1DNewmarkParameterFamily = 1;
int substepsPerTimeStep = 1;
double inversionThreshold;
double fps = 0.0;
double cpuLoad = 0;
double forceAssemblyTime = 0.0;
double systemSolveTime = 0.0;
int enableTextures = 0;
int staticSolver = 0;
int graphicFrame = 0;
int lockAt30Hz = 0;
int pulledVertex = -1;
int forceNeighborhoodSize = 5;
int dragStartX, dragStartY;
int explosionFlag = 0;
int timestepCounter = 0;
int subTimestepCounter = 0;
int numFixedVertices;
int * fixedVertices;
int numexternalLoads;
int * externalLoads;
int numForceLoads = 0;
double * forceLoads = nullptr;
IntegratorBase * integratorBase = nullptr;
ImplicitNewmarkSparse * implicitNewmarkSparse = nullptr;
IntegratorBaseSparse * integratorBaseSparse = nullptr;
ForceModel * forceModel = nullptr;
int enableCompressionResistance = 1;
double compressionResistance = 500;
int centralDifferencesTangentialDampingUpdateMode = 1;
int addGravity=0;
double g=9.81;

VolumetricMesh * volumetricMesh = nullptr;
Graph * meshGraph = nullptr;

enum massSpringSystemSourceType { OBJ, TETMESH, CUBICMESH, CHAIN, NONE } massSpringSystemSource = NONE;
enum deformableObjectType { STVK, COROTLINFEM, LINFEM, MASSSPRING, INVERTIBLEFEM, UNSPECIFIED } deformableObject = UNSPECIFIED;
enum invertibleMaterialType { INV_STVK, INV_NEOHOOKEAN, INV_MOONEYRIVLIN, INV_NONE } invertibleMaterial = INV_NONE;
enum solverType { IMPLICITNEWMARK, IMPLICITBACKWARDEULER, EULER, SYMPLECTICEULER, CENTRALDIFFERENCES, UNKNOWN } solver = UNKNOWN;

StencilForceModel * stencilForceModel = nullptr;
ForceModelAssembler *forceModelAssembler = nullptr;
MassSpringStencilForceModel * massSpringStencilForceModel = nullptr;

MassSpringSystem * massSpringSystem = nullptr;
SparseMatrix * massMatrix = nullptr;
SparseMatrix * LaplacianDampingMatrix = nullptr;
SparseMatrix * tangentStiffnessMatrix = nullptr;
SparseMatrix * massRemovedDofs = nullptr;
SparseMatrix * tangentStiffnessRemovedDofs = nullptr;
int numFixedDOFs;
int * fixedDOFs;

int n;
double error_q = 0.0;
double convergence_eps =4.00e-13;
double * u = nullptr;
double * uvel = nullptr;
double * uaccel = nullptr;
double * f_ext = nullptr;
double * f_extBase = nullptr;
double * uSecondary = nullptr;
double * uInitial = nullptr;
double * velInitial = nullptr;
double * q_ = nullptr;
double * q_prev = nullptr;
double * q_vel = nullptr;
double * time_force_data = nullptr;

double *moment_ext = nullptr;
int *moment_location = nullptr;
int nSections=0;
int nTimeStepMoment=0;
int timed_force=0;
double *fExt_file =nullptr;
int *fExt_locations =nullptr;
int n_fExt=0;
int increaseF_extGradually=0;
int impulse_force=0;
int n_tip_mesh=0;
double beamWidth=0.0;
double totalTimeCount=0.0;
float time_for_full_load=0.0;
ros::Subscriber sub;
ros::Subscriber reset_sub;
ros::Publisher pose_pub;
ros::Publisher state_pub;
ros::Publisher vel_pub;
ros::Publisher slope_pub;
ros::Publisher episode_reset_pub;
ros::Publisher complete_pose_pub;
std_msgs::Float32MultiArray pose_pub_msg;
std_msgs::Float32MultiArray vel_pub_msg;
std_msgs::Bool reset_msg;
float x,y,vx,vy;

// interpolation to secondary mesh
int secondaryDeformableObjectRenderingMesh_interpolation_numElementVertices;
int * secondaryDeformableObjectRenderingMesh_interpolation_vertices = nullptr;
double * secondaryDeformableObjectRenderingMesh_interpolation_weights = nullptr;

void stopDeformations_buttonCallBack(int code);

//font is, for example, GLUT_BITMAP_9_BY_15
void resetCallBack(std_msgs::Bool msg)
{
  if(msg.data == true)
  { double q_system[3*n]= { 0.0 };
    integratorBaseSparse->SetExternalForces(q_system);
    integratorBaseSparse->SetqState(q_system,q_system,q_system);
  
    reset_msg.data = false;
    episode_reset_pub.publish(reset_msg);
    // ROS_INFO("Episode reset Completed!");
    q_=integratorBase->Getq();
    std_msgs::Float32MultiArray state_msg;
    x=0.0;
    y=0.0;
    state_msg.data.push_back(x);
    state_msg.data.push_back(y);
    state_pub.publish(state_msg);
  }
}

void forceCallBack(const std_msgs::Float32MultiArray msg)
{  
  // ROS_INFO("C1. Rceived force from Steady state!");

  for(int section=0;section<nSections;section++)
  {
    // calculating the slope of the rigid section and applying moment on that
    double delta_y=q_[3*moment_location[2*section+1]+1]-q_[3*moment_location[2*section]+1];
    double delta_x=beamWidth+q_[3*moment_location[2*section+1]]-q_[3*moment_location[2*section]];
    double section_width=sqrt(delta_x*delta_x+delta_y*delta_y);
    double F=msg.data[section]/section_width;
    f_ext[3*moment_location[2*section]+0]=F*delta_y/section_width;
    f_ext[3*moment_location[2*section]+1]=-F*delta_x/section_width;
    f_ext[3*moment_location[2*section+1]+0]=-F*delta_y/section_width;
    f_ext[3*moment_location[2*section+1]+1]=F*delta_x/section_width;
  }
  integratorBaseSparse->SetExternalForces(f_ext);
  // cout<<"the torque given to vega is"<<msg<<endl;

  ros::Rate rate(1/timeStep);
  for(int i=0; i<substepsPerTimeStep; i++)
  {
    int code = integratorBase->DoTimestep();
    fflush(nullptr);
    subTimestepCounter++;
  }

  timestepCounter++;
  q_=integratorBase->Getq();
  q_vel=integratorBase->Getqvel();
  // calculating the new x y vx and vy
  {
    x=0.0;
    vx=0.0;
    for(int gg=0;gg<n_tip_mesh;gg++)
    {
      if(gg==0 || gg==(n_tip_mesh-1))
        {x=x+q_[3*(n-n_tip_mesh+gg)]*2;vx=vx+q_vel[3*(n-n_tip_mesh+gg)]*2;}
      else
        {x=x+q_[3*(n-n_tip_mesh+gg)];vx=vx+q_vel[3*(n-n_tip_mesh+gg)];}
    }
    x=x/(2+n_tip_mesh); 
    vx=vx/(2+n_tip_mesh); 

    y=0.0;
    vy=0.0;
    for(int gg=0;gg<n_tip_mesh;gg++)
    {
      if(gg==0 || gg==(n_tip_mesh-1))
        {y=y+q_[3*(n-n_tip_mesh+gg)+1]*2;vy=vy+q_vel[3*(n-n_tip_mesh+gg)+1]*2;}
      else
        {y=y+q_[3*(n-n_tip_mesh+gg)+1];vy=vy+q_vel[3*(n-n_tip_mesh+gg)+1];}
    }
    y=y/(2+n_tip_mesh);
    vy=vy/(2+n_tip_mesh);
  }

  std_msgs::Float32MultiArray slope_msg;
  // caluculating the slope at the tip
  for(int section=0;section<nSections;section++)
  {
    double delta_y=q_[3*moment_location[2*section+1]+1]-q_[3*moment_location[2*section]+1];
    double delta_x=beamWidth+q_[3*moment_location[2*section+1]]-q_[3*moment_location[2*section]];
    double slope=atan2(delta_y,delta_x);
    slope_msg.data.push_back(slope);
  }  

  std_msgs::Float32MultiArray state_msg;
  for(int i=0;i<n;i++)
  {
    state_msg.data.push_back(q_[3*i]);  
    state_msg.data.push_back(q_[3*i+1]);
  }  

                                                              //////////// NEW CODE ON MARCH 5, HERE 2 parameters should be changed. i+ 2*(n_tip_mesh) decided the number of states, 0.75 is the length of the soft robot
  std_msgs::Float32MultiArray complete_pose_msg;
  for(int i=0; i<n/n_tip_mesh; i = i+ 2*(n_tip_mesh))
  {
    complete_pose_msg.data.push_back(state_msg.data[1+n_tip_mesh/2+(2*i*n_tip_mesh)]);
    complete_pose_msg.data.push_back(state_msg.data[2+n_tip_mesh/2+(2*i*n_tip_mesh)] + 0.75*(n - (i+1)*n_tip_mesh)/((float)n-n_tip_mesh));
  }
                                                              /////////////END of new code
  ros::spinOnce();
  // rate.sleep();
  // ROS_INFO("C2. New state published by vega to steady state controller!");
  // cout<<"n/n_tip_mesh"<<n/n_tip_mesh<<",,,,,,n_tip_mesh/2"<<n_tip_mesh/2<<endl;
  // cout<<complete_pose_msg<<endl;
  // cout<<sizeof(state_msg)<<endl;
  state_pub.publish(state_msg);
  pose_pub_msg.data={x,y};
  vel_pub_msg.data={vx,vy};
  pose_pub.publish(pose_pub_msg);
  vel_pub.publish(vel_pub_msg);
  slope_pub.publish(slope_msg);
  complete_pose_pub.publish(complete_pose_msg);
}

// called periodically by GLUT:
void idleFunction(void)
{
  ros::NodeHandle node_handle;
  cout.precision(10);

  // subscribers
  sub = node_handle.subscribe("force", 1, forceCallBack);
  reset_sub = node_handle.subscribe("reset",1,resetCallBack);
  cout<<"node initialized. Subscribed to force"<<endl;

  // publishers
  episode_reset_pub = node_handle.advertise<std_msgs::Bool>("reset", 1);
  slope_pub= node_handle.advertise<std_msgs::Float32MultiArray>("slopes", 1);
  pose_pub= node_handle.advertise<std_msgs::Float32MultiArray>("state", 1); 
  state_pub= node_handle.advertise<std_msgs::Float32MultiArray>("complete_state", 1);
  vel_pub= node_handle.advertise<std_msgs::Float32MultiArray>("velocity", 1);
  complete_pose_pub = node_handle.advertise<std_msgs::Float32MultiArray>("complete_pose", 1);
  // reset external forces (usually to zero)
  memcpy(f_ext, f_extBase, sizeof(double) * 3 * n);
  f_ext=f_extBase;
  cout<<"timeStep is:"<<timeStep<<endl;
  integratorBase->SetTimestep(timeStep/substepsPerTimeStep);
  ros::spin();
}


/////////////////////////////////////
// program initialization
void initSimulation()
{
  if (strcmp(massSpringSystemObjConfigFilename, "__none") != 0)
    massSpringSystemSource = OBJ;

  if ((massSpringSystemSource == OBJ) || (massSpringSystemSource == TETMESH) || (massSpringSystemSource == CUBICMESH) || (massSpringSystemSource == CHAIN))
    deformableObject = MASSSPRING;

  // load mass spring system (if any)
  if (deformableObject == MASSSPRING)
  {
    switch (massSpringSystemSource)
    {
    case OBJ:
    {
      printf("Loading mass spring system from an obj file...\n");
      MassSpringSystemFromObjMeshConfigFile massSpringSystemFromObjMeshConfigFile;
      MassSpringSystemObjMeshConfiguration massSpringSystemObjMeshConfiguration;
      if (massSpringSystemFromObjMeshConfigFile.GenerateMassSpringSystem(massSpringSystemObjConfigFilename, &massSpringSystem, &massSpringSystemObjMeshConfiguration) != 0)
      {
        printf("Error initializing the mass spring system.\n");
        exit(1);
      }
      strcpy(renderingMeshFilename, massSpringSystemObjMeshConfiguration.massSpringMeshFilename);
    }
    break;

    default:
      printf("Error: mesh spring system configuration file was not specified.\n");
      exit(1);
      break;
    }

    n = massSpringSystem->GetNumParticles();

    // create the mass matrix
    massSpringSystem->GenerateMassMatrix(&massMatrix);
    // cout<<"massMatrix:";
    // massMatrix->Print();
    // create the mesh graph (used only for the distribution of user forces over neighboring vertices)
    meshGraph = new Graph(massSpringSystem->GetNumParticles(), massSpringSystem->GetNumEdges(), massSpringSystem->GetEdges());
  }

  int scaleRows = 1;
  meshGraph->GetLaplacian(&LaplacianDampingMatrix, scaleRows);
  LaplacianDampingMatrix->ScalarMultiply(dampingLaplacianCoef);

  if (!((deformableObject == MASSSPRING) && (massSpringSystemSource == CHAIN)))
  {
    // read the fixed vertices
    // 1-indexed notation
    if (strcmp(fixedVerticesFilename, "__none") == 0)
    {
      numFixedVertices = 0;
      fixedVertices = nullptr;
    }
    else
    {
      if (ListIO::load(fixedVerticesFilename, &numFixedVertices,&fixedVertices) != 0)
      {
        printf("Error reading fixed vertices.\n");
        exit(1);
      }
      ListIO::sort(numFixedVertices, fixedVertices);
    }
  }
  else
  {
    numFixedVertices = 1;
    fixedVertices = (int*) malloc (sizeof(int) * numFixedVertices);
    fixedVertices[0] = massSpringSystem->GetNumParticles();
  }
  // ListIO::print(numFixedVertices,fixedVertices);

  //fixing just the desired DOF out of 3 for each node/vertex
  numFixedDOFs=0;
  cout<<"numFixedVertices:"<<numFixedVertices<<endl;
  for (int i=0;i<numFixedVertices;i++)
  {
    if(fixedVertices[i]%10==1)
      numFixedDOFs++;
    if((fixedVertices[i]/10)%10==1)
      numFixedDOFs++;
    if((fixedVertices[i]/100)%10==1)
      numFixedDOFs++;
  } // this loop was to find the number of DOF to allocate memory accordingly

  numFixedDOFs=numFixedDOFs;
  free(fixedDOFs);
  fixedDOFs = (int*) malloc (sizeof(int) * numFixedDOFs);
  int indexCounter=0; // this variable is to store index as we find DOFs to constaint
  for(int i=0; i<numFixedVertices; i++)
  {
    if((fixedVertices[i]/100)%10==1)
    {
      fixedDOFs[indexCounter]= 3*(fixedVertices[i]/1000-1) + 0;
      indexCounter++;
    }
    if((fixedVertices[i]/10)%10==1)
    {
      fixedDOFs[indexCounter]= 3*(fixedVertices[i]/1000-1) + 1;
      indexCounter++;
    }
    if(fixedVertices[i]%10==1)
    {
      fixedDOFs[indexCounter]= 3*(fixedVertices[i]/1000-1) + 2;
      indexCounter++;
    }
  }
  ListIO::sort(numFixedDOFs, fixedDOFs);


  //finding reduced mass matrix
  // massRemovedDofs = new SparseMatrix(*massMatrix);
  // massRemovedDofs->RemoveRowsColumns(numFixedDOFs, fixedDOFs);
  // massRemovedDofs->BuildSuperMatrixIndices(numFixedDOFs, fixedDOFs, massMatrix);
  // massRemovedDofs->AssignSuperMatrix(*massMatrix);
  // massRemovedDofs->SaveToMatlabFormat("M.csv");
  // massMatrix->SaveToMatlabFormat("M2.csv");

  for(int i=0; i<numFixedVertices; i++)
    fixedVertices[i]=fixedVertices[i]/1000-1;
  printf("Boundary vertices processed.\n");

  // make room for deformation and force vectors
  u = (double*) calloc (3*n, sizeof(double));
  uvel = (double*) calloc (3*n, sizeof(double));
  uaccel = (double*) calloc (3*n, sizeof(double));
  f_ext = (double*) calloc (3*n, sizeof(double));
  f_extBase = (double*) calloc (3*n, sizeof(double));
  q_ = (double*) calloc (3*n, sizeof(double));
  q_prev = (double*) calloc (3*n, sizeof(double));
  q_vel = (double*) calloc (3*n, sizeof(double));

  // load initial condition
  if (strcmp(initialPositionFilename, "__none") != 0)
  {
    cout<<"here"<<endl;
    int nf;
    string line;
    string pose="";
    ifstream PoseFile(initialPositionFilename);

    //finding number of nodes in the file
    getline(PoseFile,line);
    nf=stoi(line);
    if(nf!=n)
    {
      cout<<"the file parsed is not correct.Number of particles dont match!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
      exit(1);
    }

    int i=0,j=0,cnt=0;
    uInitial = (double*) calloc (3*n, sizeof(double));
    while (getline (PoseFile, line)) {
      // Output the text file are interpreted as x,y,z and stored in uInitial
      while(line[cnt]!='\0')
      {
        if(line[cnt]==',')
        {
          uInitial[3*i+j]=stod(pose);
          pose="";
          j++;
        }
        else
          pose=pose+line[cnt];
        cnt++;
      }
      uInitial[3*i+j]=stod(pose);
      pose="";
      j=0;
      cnt=0;
      i++;
    }

    //printing the Uinitial matrix
    for(int i=0;i<n;i++)
    {
      cout<<uInitial[3*i]<<"X"<<uInitial[3*i+1]<<"Y"<<uInitial[3*i+2]<<"Z"<<endl;
    }

  }
  else
  {
    uInitial = (double*) calloc (3*n, sizeof(double));
  }

  // load initial velocity
  if (strcmp(initialVelocityFilename, "__none") != 0)
  {
    int m1, n1;
    ReadMatrixFromDisk_(initialVelocityFilename, &m1, &n1, &velInitial);
    if ((m1 != 3*n) || (n1 != 1))
    {
      printf("Error: initial position matrix size mismatch.\n");
      exit(1);
    }
  }

  // create force model, to be used by the integrator
  printf("Creating force model...\n");

  if (deformableObject == MASSSPRING)
  {
    printf("Force model: MASSSPRING\n");

    massSpringStencilForceModel = new MassSpringStencilForceModel(massSpringSystem);
    stencilForceModel = massSpringStencilForceModel;
  }

  assert(stencilForceModel != nullptr);
  forceModelAssembler = new ForceModelAssembler(stencilForceModel);
  forceModel = forceModelAssembler;

  // initialize the integrator
  printf("Initializing the integrator, n = %d...\n", n);
  printf("Solver type: %s\n", solverMethod);

  integratorBaseSparse = nullptr;
  if (solver == IMPLICITNEWMARK)
  {
    // cout<<"number of fixed vertices are:"<<numFixedDOFs<<endl;
    implicitNewmarkSparse = new ImplicitNewmarkSparse(3*n, timeStep, massMatrix, forceModel, numFixedDOFs, fixedDOFs,
       dampingMassCoef, dampingStiffnessCoef, maxIterations, epsilon, newmarkBeta, newmarkGamma, numSolverThreads);
    integratorBaseSparse = implicitNewmarkSparse;
  }

  integratorBase = integratorBaseSparse;
  if (integratorBase == nullptr)
  {
    printf("Error: failed to initialize numerical integrator.\n");
    exit(1);
  }

  // set integration parameters
  integratorBaseSparse->SetDampingMatrix(LaplacianDampingMatrix);
  integratorBase->ResetToRest();
  integratorBase->SetState(uInitial, velInitial);
  integratorBase->SetTimestep(timeStep / substepsPerTimeStep);

  if (implicitNewmarkSparse != nullptr)
  {
    implicitNewmarkSparse->UseStaticSolver(staticSolver);
    if (velInitial != nullptr)
      implicitNewmarkSparse->SetState(implicitNewmarkSparse->Getq(), velInitial);
  }

  // reading beam dimensions and some configurations file
  if (strcmp(vegaConfigurationsFilename, "__none") != 0)
  {
    //read the configuration file
    std::ifstream Dfile(vegaConfigurationsFilename);
    if (Dfile.is_open())
    {
      cout<<"reading the beam dimensions file."<<endl;
      string word;
      if(Dfile>>word)
        n_tip_mesh=stoi(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        beamWidth=stod(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        increaseF_extGradually=stoi(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        time_for_full_load=stof(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        impulse_force=stoi(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      if(Dfile>>word)
        timed_force=stoi(word);
      else
      {
        cout<<"there is a possible error in beam dimensions file!!!"<<endl;
        exit(1);
      }
      cout<<"The width of the beam is:"<<beamWidth<<"  and the number of nodes in a row is:"<<n_tip_mesh<<"and impulse force is:"<<impulse_force<<endl;
    }
    else
    {
      cout<<"error opening the beam dimensions file"<<endl;
      exit(1);
    }
  }

  // reading external moment file
  if (strcmp(momentRigidSectionLoadsFilename, "__none") != 0)
  {
    //read the moment file file
    std::ifstream Mfile(momentRigidSectionLoadsFilename);
    if (Mfile.is_open())
    {
      cout<<"reading the external moment file."<<endl;
      string word;
      Mfile>>word;
      nSections=stoi(word);
      moment_ext = (double*) calloc (nSections, sizeof(double));
      moment_location = (int*) calloc (2*nSections, sizeof(int));
      for(int i=0;i<nSections;i++)
      {
        if(Mfile>>word)
          moment_location[2*i]=stoi(word)-1;
        else
        {
          cout<<"there is a possible error in moment file!!!"<<endl;
          exit(1);
        }
        if(Mfile>>word)
          moment_location[2*i+1]=stoi(word)-1;
        else
        {
          cout<<"there is a possible error in moment file!!!"<<endl;
          exit(1);
        }
        if(Mfile>>word)
          moment_ext[i]=stod(word);
        else
        {
          cout<<"there is a possible error in moment file!!!"<<endl;
          exit(1);
        }
        cout<<"applying moment of :"<<moment_ext[i]<<" between nodes :"<<moment_location[2*i]+1<<" and "<<moment_location[2*i+1]+1<<endl;
      }
    }
    else
    {
      cout<<"error opening the external moments file"<<endl;
      exit(1);
    }
  }

  // reading external moment timed file
  if (strcmp(momentRigidSectionLoadsTimedFilename, "__none") != 0 and timed_force!=0)
  {
    //read the moment file file
    std::ifstream Mfile(momentRigidSectionLoadsTimedFilename);
    if (Mfile.is_open())
    {
      cout<<"reading the external moment timed file."<<endl;
      string word;
      Mfile>>word;
      nTimeStepMoment=stoi(word);
      Mfile>>word;
      nSections=stoi(word);
      moment_ext = (double*) calloc (nTimeStepMoment*nSections, sizeof(double));
      moment_location = (int*) calloc (2*nSections, sizeof(int));
      for(int i=0;i<nSections;i++)
      {
        if(Mfile>>word)
          moment_location[2*i]=stoi(word)-1;
        else
        {
          cout<<"there is a possible error in moment timed file!!!"<<endl;
          exit(1);
        }
        if(Mfile>>word)
          moment_location[2*i+1]=stoi(word)-1;
        else
        {
          cout<<"there is a possible error in moment timed file!!!"<<endl;
          exit(1);
        }
      }
      for(int t=0;t<nTimeStepMoment;t++)
      {
        for(int i=0;i<nSections;i++)
        {
          if(Mfile>>word)
            moment_ext[t*nSections+i]=stod(word);
          else
          {
            cout<<"there is a possible error in moment timed file!!!"<<endl;
            exit(1);
          }
          // cout<<"applying moment of :"<<moment_ext[i]<<" between nodes :"<<moment_location[2*i]+1<<" and "<<moment_location[2*i+1]+1<<endl;
        }
      }
    }
    else
    {
      cout<<"error opening the external moments timed file"<<endl;
      exit(1);
    }
  }

  // reading external load file and applying the corresponding force
  if (strcmp(forceLoadsFilename, "__none") != 0)
  {
    //read the force file
    std::ifstream Ffile(forceLoadsFilename);
    if (Ffile.is_open())
    {
      cout<<"reading the external force file."<<endl;
      string word;
      if(Ffile>>word)
        n_fExt=stoi(word);
      cout<<"the number of nodes on which f_ext is applied is:"<<n_fExt<<endl;
      fExt_file = (double*) calloc (n_fExt, sizeof(double));
      fExt_locations = (int*) calloc (n_fExt, sizeof(int));
      for(int fExt_count=0;fExt_count<n_fExt;fExt_count++)
      {
        if(Ffile>>word)
          fExt_locations[fExt_count]=stoi(word);
        else
        {
          cout<<"there is a possible error in force file!!!"<<endl;
          exit(1);
        }
        if(Ffile>>word)
          fExt_file[fExt_count]=stod(word);
        else
        {
          cout<<"there is a possible error in force file!!!"<<endl;
          exit(1);
        }
      }
    }
    else
    {
      cout<<"error opening the external force file"<<endl;
      exit(1);
    }
  }
}

// set up the configuration file
void initConfigurations()
{
  printf("Parsing configuration file %s...\n", configFilename.c_str());
  ConfigFile configFile;

  // specify the entries of the config file

  // at least one of the following must be present:
  configFile.addOptionOptional("massSpringSystemObjConfigFilename", massSpringSystemObjConfigFilename, "__none");
  configFile.addOptionOptional("solver", solverMethod, "implicitNewmark");
  configFile.addOptionOptional("initialPositionFilename", initialPositionFilename, "__none");
  configFile.addOptionOptional("initialVelocityFilename", initialVelocityFilename, "__none");
  configFile.addOptionOptional("g", &g, g);

  configFile.addOptionOptional("renderingMeshFilename", renderingMeshFilename, "__none");
  configFile.addOptionOptional("fixedVerticesFilename", fixedVerticesFilename, "__none");
  configFile.addOptionOptional("substepsPerTimeStep", &substepsPerTimeStep, substepsPerTimeStep);
  configFile.addOption("dampingMassCoef", &dampingMassCoef);
  configFile.addOption("dampingStiffnessCoef", &dampingStiffnessCoef);
  configFile.addOptionOptional("forceLoadsFilename", forceLoadsFilename, "__none");
  configFile.addOptionOptional("vegaConfigurationsFilename", vegaConfigurationsFilename, "__none");
  configFile.addOptionOptional("momentRigidSectionLoadsFilename", momentRigidSectionLoadsFilename, "__none");
  configFile.addOptionOptional("momentRigidSectionLoadsTimedFilename", momentRigidSectionLoadsTimedFilename, "__none");

  // parse the configuration file
  if (configFile.parseOptions((char*)configFilename.c_str()) != 0)
  {
    printf("Error parsing options.\n");
    exit(1);
  }

  // the config variables have now been loaded with their specified values

  // informatively print the variables (with assigned values) that were just parsed
  configFile.printOptions();

  if (strcmp(solverMethod, "implicitNewmark") == 0)
    solver = IMPLICITNEWMARK;
  if (strcmp(solverMethod, "implicitBackwardEuler") == 0)
    solver = IMPLICITBACKWARDEULER;
  if (strcmp(solverMethod, "Euler") == 0)
    solver = EULER;
  if (strcmp(solverMethod, "symplecticEuler") == 0)
    solver = SYMPLECTICEULER;
  if (strcmp(solverMethod, "centralDifferences") == 0)
    solver = CENTRALDIFFERENCES;
  if (solver == UNKNOWN)
  {
    printf("Error: unknown implicit solver specified.\n");
    exit(1);
  }
}

void deformableObjectCompliance_spinnerCallBack(int code)
{
  if (deformableObjectCompliance < 0)
    deformableObjectCompliance = 0;
}

void timeStep_spinnerCallBack(int code)
{
  if (timeStep < 0)
    timeStep = 0;

  integratorBase->SetTimestep(timeStep / substepsPerTimeStep);
}

void frequencyScaling_spinnerCallBack(int code)
{
  if (frequencyScaling < 0)
    frequencyScaling = 0;
  integratorBase->SetInternalForceScalingFactor(frequencyScaling * frequencyScaling);
}

void newmarkBeta_spinnerCallBack(int code)
{
  if (newmarkBeta < 0)
    newmarkBeta = 0;

  if (newmarkBeta > 0.5)
    newmarkBeta = 0.5;

  if (use1DNewmarkParameterFamily)
  {
    if (newmarkBeta > 0.25)
      newmarkGamma = sqrt(4.0 * newmarkBeta) - 0.5;
    else
      newmarkGamma = 0.5;
  }

  if (implicitNewmarkSparse != nullptr)
  {
    implicitNewmarkSparse->SetNewmarkBeta(newmarkBeta);
    implicitNewmarkSparse->SetNewmarkGamma(newmarkGamma);
  }
}

void newmarkGamma_spinnerCallBack(int code)
{
  if (newmarkGamma < 0.5)
    newmarkGamma = 0.5;

  if (newmarkGamma > 1.0)
    newmarkGamma = 1.0;

  if (use1DNewmarkParameterFamily)
    newmarkBeta = (newmarkGamma + 0.5) * (newmarkGamma + 0.5) / 4.0;

  if (implicitNewmarkSparse != nullptr)
  {
    implicitNewmarkSparse->SetNewmarkBeta(newmarkBeta);
    implicitNewmarkSparse->SetNewmarkGamma(newmarkGamma);
  }
}

void newmark_checkboxuse1DNewmarkParameterFamilyCallBack(int code)
{
  if (use1DNewmarkParameterFamily)
  {
    newmarkBeta = (newmarkGamma + 0.5) * (newmarkGamma + 0.5) / 4.0;

    if (implicitNewmarkSparse != nullptr)
    {
      implicitNewmarkSparse->SetNewmarkBeta(newmarkBeta);
      implicitNewmarkSparse->SetNewmarkGamma(newmarkGamma);
    }
  }
}

void rayleighMass_spinnerCallBack(int code)
{
  if (dampingMassCoef < 0)
    dampingMassCoef = 0;

  integratorBase->SetDampingMassCoef(dampingMassCoef);
}

void rayleighStiffness_spinnerCallBack(int code)
{
  if (dampingStiffnessCoef < 0)
    dampingStiffnessCoef = 0;

  integratorBase->SetDampingStiffnessCoef(dampingStiffnessCoef);
}

void timeStepSubdivisions_spinnerCallBack(int code)
{
  if (substepsPerTimeStep < 1)
    substepsPerTimeStep = 1;

  integratorBase->SetTimestep(timeStep / substepsPerTimeStep);
}

void staticSolver_checkboxCallBack(int code)
{
  implicitNewmarkSparse->UseStaticSolver(staticSolver);
}

// main function
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vega_simulator");
  // parse command line options
  timeStep=stof(argv[1]);
  substepsPerTimeStep=stoi(argv[2]);

  char configFilenameC[4096]="beam3_vox_massspring.config" ;

  printf("Starting application.\n");
  configFilename = string(configFilenameC);
  printf("Loading scene configuration from %s.\n", configFilename.c_str());

  initConfigurations(); // parse the config file
  initSimulation(); // init the simulation
  idleFunction();
  return 0;
}
