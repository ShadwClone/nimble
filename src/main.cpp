/*******************************************************************************
* The applicaion will help users with disabilities acquiring the writing
* and drawing skills by using a haptic device. It uses a robotic mapping from
* a haptic user interface to a virtual environment. For the tasks, the
* application consists of a force reflecting haptic interface drive, PHANToM
* Omni with OpenHaptics Toolkit (version 3.0) and OpenGL.
*******************************************************************************/
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <ctime>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <list>
#include <limits>
#include <string>

//playback
#include <cstdio>

#if defined(WIN32)
#include <conio.h>
#include <windows.h>
#include <direct.h>
#else
#include "conio.h"
#endif

#if defined(WIN32) || defined(linux)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#endif

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
#include <HLU/hlu.h>

#include "imageloader.h"
#include "constants.h"

using namespace std;

GLuint _textureList[4];
GLuint screenTextureList[4];
HLuint friction;

/* Shape ids for haptically rendered shapes. */
HLuint gSurfaceShapeId = 0;
HLuint gLineShapeId = 0;
HLuint gHapticsStringShapeId = 0;
HLuint gPlaneShapeId = 0;;
static GLfloat linePos[] = {0, 0, 0};
static double gLineShapeSnapDistance = 0.0;
static HHD hHD = HD_INVALID_HANDLE;
static HHLRC hHLRC = 0;

/* Shape id for shapes we will render haptically. */
HLuint gBoxesShapeId;

GLfloat mass_weight = 0.0; // default weight
GLfloat k_damping;
GLfloat spring_stiffness;
hduVector3Dd position;

#define CURSOR_SIZE_PIXELS 30
static double gCursorScale;
static GLuint gCursorDisplayList = 0;
int menuSelection;


/*******************************************************************************
Point mass structure, represents a draggable mass.
*******************************************************************************/
struct PointMass
{
  hduVector3Dd m_position;
  hduVector3Dd m_velocity;
  HDdouble m_mass;
  HDdouble m_kStiffness;
  HDdouble m_kDamping;
};
 
PointMass pointMass;
HLuint effect = NULL;

/*******************************************************************************
 ANN: Spatio-temporal device state for artificial neural network analysis
*******************************************************************************/
struct DeviceState
{
  HDdouble position[3];
  LARGE_INTEGER counter;
};

list<DeviceState> deviceStateList;
HDSchedulerHandle deviceStateHandle = NULL;


// Function prototypes
void glutDisplay(void);
void glutReshape(int width, int height);
void glutIdle(void);   
void glutContextMenu(int); 

void exitHandler(void);

void getPatternSelection();
void loadPattern();
void attachContextMenu();

void initGL();
void initHD();
void initScene();
void drawSceneHaptics();
void drawSceneGraphics();
void drawCursor_Air();
void updateWorkspace();
void initRendering();

GLuint loadTexture(Image* image);

/*******************************************************************************
 Initializes GLUT for displaying a simple haptic scene.
*******************************************************************************/
int main(int argc, char *argv[])
{
  glutInit(&argc, argv);
    
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  int screenWidth  = glutGet(GLUT_SCREEN_WIDTH),
      screenHeight = glutGet(GLUT_SCREEN_HEIGHT);

  // initialize window and center it
  glutInitWindowSize(800,600);
  glutInitWindowPosition((screenWidth-800)/2,(screenHeight-600)/2);
  glutCreateWindow("Haptic Learning Center");

  // load pattern
  getPatternSelection();
  loadPattern();
  
  attachContextMenu();

  // Set glut callback functions.
  glutDisplayFunc(glutDisplay);
  glutReshapeFunc(glutReshape);
  glutIdleFunc(glutIdle);

  
  atexit(exitHandler); // Provide a cleanup routine for application exit.
  initScene(); // Initializes OpenGL and Haptic scenes
  glutMainLoop(); // Start main graphics loop

  return 0;
}


/*******************************************************************************
 Initializes the scene.  Handles initializing both OpenGL and HD/HL.
*******************************************************************************/
void initScene()
{
  initGL();
  initHD();
}


// write the devices states to file with the given file name.
void writeDeviceStatesToFile()
{
  //TODO: incorperate patient id into name
  ostringstream fileName;
  time_t rawtime = time(NULL);
  tm *timeInfo = localtime(&rawtime);
  string fileDir("output/");

  fileName << timeInfo->tm_year+1900
           << setfill('0') << setw(2) << timeInfo->tm_mon+1 
           << timeInfo->tm_mday << timeInfo->tm_hour 
           << timeInfo->tm_min << timeInfo->tm_sec << ".txt";

  fileDir.append(fileName.str());

  ofstream dsFile(fileDir.c_str());

  if(dsFile.is_open())
  {
    dsFile.clear();

    dsFile << "%YAML 1.2" << endl
           << "---" << endl
           << "date: " << endl //TODO: date
           << "location: " << endl
           << "patient-id: " << endl; //TODO: patient ID

    dsFile << "pattern: " << endl
           << "    type: ";

    if(menuSelection < 4)
      dsFile << "complexity" << endl;
    else if(menuSelection < 7)
      dsFile << "straight to Curvy" << endl;
    else if(menuSelection < 10)
      dsFile << "width" << endl;

    dsFile << "    level: ";

    if(menuSelection%3 == 1)
      dsFile << "1" << endl;
    else if(menuSelection%3 == 2)
      dsFile << "2" << endl;
    else if(menuSelection%3 == 0)
      dsFile << "3" << endl;
    
    //workspace in the Air/Desk 
    dsFile << "workspace: in air\n"
           << "coordinate-space: world\n";

    double countTime;
    LARGE_INTEGER freq;
    LONGLONG counterEpoch = deviceStateList.front().counter.QuadPart;

    QueryPerformanceFrequency(&freq);

    countTime = (double(deviceStateList.back().counter.QuadPart - counterEpoch)
                 *1.0e3) / (double(freq.QuadPart));


    dsFile << "total-time: " << countTime << endl
           << "data: " << endl
           << "- [x, y, z, time]" << endl;
    
    list<DeviceState>::iterator it;

    for(it = deviceStateList.begin(); it != deviceStateList.end(); it++)
    {
      countTime = (double(it->counter.QuadPart - counterEpoch)*1.0e3)
                  /(double(freq.QuadPart));
        
      dsFile << fixed << setprecision(4) << "- ["
             << it->position[0] << ", " 
             << it->position[1] << ", "
             << it->position[2] << ", "
             << setprecision(1) << countTime << "]" << endl;
    }
    
    deviceStateList.clear();

    dsFile.close();
  }
  else
    cout << "CAN'T OPEN OUTPUT FILE: " << fileDir << endl;
}


/*******************************************************************************
 GLUT callback for redrawing the view.
*******************************************************************************/
void glutDisplay()
{   
  drawSceneHaptics();
  drawSceneGraphics();
  glutSwapBuffers();
}


/*******************************************************************************
 GLUT callback for reshaping the window. This is the main place where the 
 viewing and workspace transforms get initialized.
*******************************************************************************/
void glutReshape(int w, int h)
{
  static const double kPI = 3.1415926535897932384626433832795;
  static const double kFovY = 45;

  double nearDist, farDist, aspect;

  glViewport(0, 0, w, h);

  // Compute the viewing parameters based on a fixed fov and viewing
  // a canonical box centered at the origin.

  nearDist = 1.0 / tan((kFovY / 2.0) * kPI / 180.0);
  farDist = nearDist + 6.1;
  aspect = (double) w / h;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(kFovY, aspect, nearDist, farDist);

  // Place the camera down the Z axis looking at the origin.
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();            
  gluLookAt(0, 0, nearDist + 1.0, 0, 0, 0,0, 1, 0);
    
  updateWorkspace();
}


/*******************************************************************************
 GLUT callback for idle state.  Use this as an opportunity to request a redraw.
 Checks for HLAPI errors that have occurred since the last idle check.
*******************************************************************************/
void glutIdle()
{
  HLerror error;

  while (HL_ERROR(error = hlGetError()))
  {
    //fprintf(stderr, "HL Error: %s\n", error.errorCode);

    if(error.errorCode == HL_DEVICE_ERROR)
    {
      hduPrintError(stderr, &error.errorInfo,"Error during haptic rendering\n");
    }
  }

  glutPostRedisplay();
}


/*******************************************************************************
 Sets up general OpenGL rendering properties: lights, depth buffering, etc.
*******************************************************************************/
void initGL()
{
    static const GLfloat light_model_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    static const GLfloat light0_diffuse[] = {0.9f, 0.9f, 0.9f, 0.9f};   
    static const GLfloat light0_direction[] = {0.0f, -0.4f, 1.0f, 0.0f};    
    
    // Enable depth buffering for hidden surface removal.
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    
    // Cull back faces.
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    
    // Setup other misc features.
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    
    // Setup lighting model.
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);    
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_direction);
    glEnable(GL_LIGHT0);   
}


/*******************************************************************************
 Servo loop thread callback. Computes a force effect.
*******************************************************************************/
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache *cache, void *userdata)
{
  PointMass *pPointMass = static_cast<PointMass *>(userdata);

  // Get the time delta since the last update.
  HDdouble instRate;
  hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
  HDdouble deltaT = 1.0 / instRate;
    
  // Get the current proxy position from the state cache.
  // Note that the effect state cache is maintained in workspace coordinates,
  // so we don't need to do any transformations in using the proxy
  // position for computing forces.
  hduVector3Dd proxyPos;

  hlCacheGetDoublev(cache, HL_PROXY_POSITION, proxyPos);

  // Compute the inertia force based on pulling the point mass around
  // by a spring.
  hduVector3Dd springForce = pPointMass->m_kStiffness * (proxyPos - pPointMass->m_position);
  hduVector3Dd damperForce = -pPointMass->m_kDamping * pPointMass->m_velocity;
  hduVector3Dd inertiaForce = springForce + damperForce;
      
  // Perform Euler integration of the point mass state.
  hduVector3Dd acceleration = inertiaForce / pPointMass->m_mass;
  pPointMass->m_velocity += acceleration * deltaT;    
  pPointMass->m_position += pPointMass->m_velocity * deltaT;
                                   
  // Gravity well starts here
  //hduVector3Dd position;
  hdGetDoublev(HD_CURRENT_POSITION,position);
  hduVector3Dd gravityWellCenter(0.0,0.0,0.0); //coord doesn't work yet
  const float k = k_damping;
  hduVector3Dd forceVector = (gravityWellCenter-position)*k;
    
  force[0] += forceVector[0];
  force[1] += forceVector[1];
  force[2] += forceVector[2];
  // gravity well ends here

  // Send the opposing force to the device.
  force[0] += -inertiaForce[0];
  force[1] += -inertiaForce[1];
  force[2] += -inertiaForce[2];
}


/*******************************************************************************
 Servo loop thread callback called when the effect is started.
*******************************************************************************/
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata)
{
  PointMass *pPointMass = (PointMass *) userdata;
    
  fprintf(stdout, "Custom effect started\n");

  // Initialize the position of the mass to be at the proxy position.
  hlCacheGetDoublev(cache, HL_PROXY_POSITION, pPointMass->m_position);

  pPointMass->m_velocity.set(0, 0, 0);
}


/*******************************************************************************
 Servo loop thread callback called when the effect is stopped.
*******************************************************************************/
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata)
{
  fprintf(stdout, "Custom effect stopped\n");
}


/*******************************************************************************
 Initializes the control parameters used for simulating the point mass.
*******************************************************************************/
void initPointMass(PointMass *pPointMass)
{
  pPointMass->m_mass = mass_weight; // Kg        

  // Query HDAPI for the max spring stiffness and then tune it down to allow
  // for stable force rendering throughout the workspace.
  hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &pPointMass->m_kStiffness);
  pPointMass->m_kStiffness *= spring_stiffness;

  // Compute damping constant so that the point mass motion is critically damped.
  pPointMass->m_kDamping = 2 * sqrt(pPointMass->m_mass * pPointMass->m_kStiffness);
}


/*******************************************************************************
 ANN: Servo loop thread callback called when the effect is stopped.
*******************************************************************************/
HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData)
{
  DeviceState state;
  list<DeviceState> *pDSL = static_cast<list<DeviceState> *>(pUserData);

  QueryPerformanceCounter(&state.counter);
  
  hdGetDoublev(HD_CURRENT_POSITION, state.position);

  pDSL->push_back(state);

  return HD_CALLBACK_CONTINUE;
}


/*******************************************************************************
 Initialize the HDAPI. This involves initing a device configuration, enabling
 forces, and scheduling a haptic thread callback for servicing the device.
*******************************************************************************/
void initHD()
{
  HDErrorInfo error;

  hHD = hdInitDevice(HD_DEFAULT_DEVICE);

  if(HD_DEVICE_ERROR(error = hdGetError()))
  {
    hduPrintError(stderr, &error, "Failed to initialize haptic device");
    fprintf(stderr, "Press any key to exit");
    getchar();
    exit(-1);
  }

  hHLRC = hlCreateContext(hHD);
  hlMakeCurrent(hHLRC);

  // Enable optimization of the viewing parameters when rendering
  // geometry for OpenHaptics.
  hlEnable(HL_HAPTIC_CAMERA_VIEW);

  // Generate id's for the shapes.
  gBoxesShapeId = hlGenShapes(1);
  gLineShapeId  = hlGenShapes(1);

  // Initialize the point mass.
  initPointMass(&pointMass);
  effect = hlGenEffects(1);
  hlBeginFrame();

  hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc) computeForceCB, &pointMass);
  hlCallback(HL_EFFECT_START, (HLcallbackProc) startEffectCB, &pointMass);
  hlCallback(HL_EFFECT_STOP, (HLcallbackProc) stopEffectCB, &pointMass);

  hlStartEffect(HL_EFFECT_CALLBACK, effect);
  hlEndFrame();
}


/*******************************************************************************
 This handler is called when the application is exiting.  Deallocates any state 
 and cleans up.
*******************************************************************************/
void exitHandler()
{
  // Deallocate the sphere shape id we reserved in initHD().
  hlDeleteShapes(gBoxesShapeId, 1);
  hlDeleteShapes(gLineShapeId, 1);

  // Free up the haptic rendering context.
  hlMakeCurrent(NULL);

  if(hHLRC != NULL)
    hlDeleteContext(hHLRC);

  // Free up the haptic device.
  if(hHD != HD_INVALID_HANDLE)
    hdDisableDevice(hHD);

  hlBeginFrame();
  hlStopEffect(effect);
  hlEndFrame();

  hlDeleteEffects(effect, 1);
}


/******************************************************************************
 Context menu handler.
******************************************************************************/
void glutContextMenu(int key)
{
  switch(key)
  {
    case 0: // Get Ratio Point 1
      break;

    case 1: // Get Ratio Point 2
      break;

    case 2: // No Effect
      mass_weight = 0.0;
      k_damping = 0.0;
      spring_stiffness = 0.0;
      initPointMass(&pointMass);
      break;

    case 3: // Low Inertia Effect
      mass_weight = 0.010;
      k_damping = 0.001;
      spring_stiffness = 0.1;
      initPointMass(&pointMass);
      break;

    case 4: // Medium Inertia Effect
      mass_weight = 0.030;
      k_damping = 0.003;
      spring_stiffness = 0.2;
      initPointMass(&pointMass);
      break;

    case 5: // High Inertia Effect
      mass_weight = 0.050;
      k_damping = 0.005;
      spring_stiffness = 0.4;
      initPointMass(&pointMass);
      break;

    case 6: // Start Recording
      //Schedule device state sampling callback in servo loop -AK
      deviceStateHandle = hdScheduleAsynchronous(DeviceStateCallback,
                                                 (void *) &deviceStateList,
                                                 HD_MAX_SCHEDULER_PRIORITY);
      hdStartScheduler();
      break;

    case 7: // Quit
      // Unschedule device state sampling callback in servo loop -AK
      if(deviceStateHandle)
      {
        hdStopScheduler();
        hdUnschedule(deviceStateHandle);
        writeDeviceStatesToFile();
        deviceStateHandle = NULL;
      }

      exit(0);
  }
}


/*******************************************************************************
 Use the current OpenGL viewing transforms to initialize a transform for the
 haptic device workspace so that it's properly mapped to world coordinates.
*******************************************************************************/
void updateWorkspace()
{
  GLdouble modelview[16];
  GLdouble projection[16];
  GLint viewport[4];

  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetIntegerv(GL_VIEWPORT, viewport);

  hlMatrixMode(HL_TOUCHWORKSPACE);
  hlLoadIdentity();
    
  // Fit haptic workspace to view volume.
  hluFitWorkspace(projection);

  // Compute cursor scale.
  gCursorScale = hluScreenToModelScale(modelview, projection, (HLint*)viewport);
  gCursorScale *= CURSOR_SIZE_PIXELS;
}


/*******************************************************************************
 Print pattern selection menu, and get selection from user.
*******************************************************************************/
void getPatternSelection()
{
  cout << endl
       << "-----------------------------------------------" << endl
       << "Haptic Testing Menu" << endl
       << "===============================================" << endl
       << endl
       << "Complexity:" << endl 
       << " [1] Level 1" << endl
       << " [2] Level 2" << endl
       << " [3] Level 3" << endl
       << endl 
       << "Straight to Curvy:" << endl 
       << " [4] Level 1" << endl
       << " [5] Level 2" << endl
       << " [6] Level 3" << endl
       << endl 
       << "Width:" << endl 
       << " [7] Level 1" << endl
       << " [8] Level 2" << endl
       << " [9] Level 3" << endl
       << endl 
       << "[0] Exit Application" << endl
       << endl
       << endl
       << "Please enter your choice: ";

  cin >> menuSelection;
}


void loadPattern()
{
  string filepath = "patterns/";

  switch(menuSelection)
  {
    // Complexity
    case 1:filepath += "comp1.bmp"; break;
    case 2:filepath += "comp2.bmp"; break;
    case 3:filepath += "comp3.bmp"; break;

    // Straight to Curvy
    case 4:filepath += "stc1.bmp"; break;
    case 5:filepath += "stc2.bmp"; break;
    case 6:filepath += "stc3.bmp"; break;
    
    // Width
    case 7:filepath += "wid1.bmp"; break;
    case 8:filepath += "wid2.bmp"; break;
    case 9:filepath += "wid3.bmp"; break;

    case 0: // Exit Application
      cout << "Thank you." << endl;
      exit(0);
      
    default:
      cout << "Selection not valid." << endl;
      exit(0);
  }
  
  // load selected pattern
  _textureList[3] = loadTexture(loadBMP(filepath.c_str()));
}


void attachContextMenu()
{
  glutCreateMenu(glutContextMenu);  
  glutAddMenuEntry("No Effect", 2);
  glutAddMenuEntry("Low Inertia Effect", 3);
  glutAddMenuEntry("Medium Inertia Effect", 4);
  glutAddMenuEntry("High Inertia Effect", 5);
  glutAddMenuEntry("Start Recording",6);  
  glutAddMenuEntry("Quit", 7);
  glutAttachMenu(GLUT_RIGHT_BUTTON);
}


//Makes the image into a texture, and returns the id of the texture
GLuint loadTexture(Image* image)
{
  GLuint textureId;
  glGenTextures(1, &textureId); //Make room for our texture
  glBindTexture(GL_TEXTURE_2D, textureId); //Tell OpenGL which texture to edit

  //Map the image to the texture
  glTexImage2D(GL_TEXTURE_2D,    //Always GL_TEXTURE_2D
               0,                //0 for now
               GL_RGB,           //Format OpenGL uses for image
               image->width,     //image width
               image->height,    //image height
               0,                //image border
               GL_RGB,           //GL_RGB pixel format
               GL_UNSIGNED_BYTE, //GL_UNSIGNED_BYTE pixel format
               image->pixels);   //actual pixel data

  return textureId; //Returns the id of the texture
}


/*******************************************************************************
 The main routine for displaying the scene. Gets the latest snapshot of state
 from the haptic thread and uses it to display a 3D cursor.
*******************************************************************************/
void drawSceneGraphics()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);           
  
  // Draw 3D cursor at haptic device position.
  drawCursor_Air();

  glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);

  glMatrixMode(GL_MODELVIEW); //Switch to the drawing perspective
  glLoadIdentity(); //Reset the drawing perspective

  glTranslatef(0.0f, 0.0f, -4.0f); //Move forward 5 units
  
  glPushMatrix(); //Save the transformations performed thus far
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, _textureList[3]);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  glBegin(GL_QUADS);
    glTexCoord2f(0,0);
    glVertex2f(-1,-1);
    glTexCoord2f(1,0);
    glVertex2f( 1,-1);
    glTexCoord2f(1,1);
    glVertex2f( 1, 1);
    glTexCoord2f(0,1);
    glVertex2f(-1, 1);
  glEnd();

  glPopMatrix();
  glPopAttrib();
}


/*******************************************************************************
 The main routine for rendering scene haptics.
*******************************************************************************/
void drawSceneHaptics()
{
  // Check for events
  hlCheckEvents();

  // Start haptic frame (Must do this before rendering any haptic shapes)
  hlBeginFrame();
  
  // Draw 3D cursor at haptic device position
  drawCursor_Air();

  // Set material properties for the shapes to be drawn
  hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.7f);
  hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.1f);
  hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.2f);
  hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION, 0.3f);
  hlHintb(HL_SHAPE_DYNAMIC_SURFACE_CHANGE, HL_TRUE);
  
  hlTouchModel(HL_CONTACT);
  
  // Start the haptic shape
  hlBeginShape(HL_SHAPE_DEPTH_BUFFER, gBoxesShapeId);

  glPushMatrix();

  glBegin(GL_QUADS);
    glTexCoord2f(0.0f, -1.0f);  //bottom left
    glVertex3f(-8.35f, -6.5f, 0.0f);
    glTexCoord2f(0.95f, -1.0f); //bottom right
    glVertex3f(5.4f, -6.5f, 0.0f);
    glTexCoord2f(0.95f, 0.0f);  //top right
    glVertex3f(5.4f, 3.64f, 0.0f);
    glTexCoord2f(0.0f, 0.0f);   //top left
    glVertex3f(-8.35f, 3.64f, 0.0f);
  glEnd();

  glPopMatrix();
  hlEndShape();

  // End the haptic frame
  hlEndFrame();
}


/*******************************************************************************
 Draws a 3D cursor for the haptic device using the current local transform,
 the workspace to world transform and the screen coordinate scale.
 ******************************************************************************/
void drawCursor_Air()
{
  static const double kCursorRadius = 0.3;
  static const double kCursorHeight = 1.0;
  static const int kCursorTess = 8;
  HLdouble proxyTransform[16];
  HLdouble proxyPosition[3];

  GLUquadricObj *qobj = 0;

  //TODO: move to better location. Can the line state be saved?
  /*/ overlay path history
  if(!deviceStateList.empty())
  {
    list<DeviceState>::iterator it;

    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    glBegin(GL_LINE_STRIP);

    for(it = deviceStateList.begin(); it != deviceStateList.end(); it++)
      glVertex2f(it->x, it->y);

    glEnd();

    glPopAttrib();
  }
  // */


  glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
  
  glPushMatrix();

  if(!gCursorDisplayList)
  {
    gCursorDisplayList = glGenLists(1);
    glNewList(gCursorDisplayList, GL_COMPILE);
    qobj = gluNewQuadric();
               
    glPushMatrix();
    glColor3f(0.3, 0.5, 0.9); //BLUE
    gluSphere(qobj, kCursorRadius, kCursorTess, kCursorTess);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0.9, 0.7, 0.1);//brown-yellow
    glTranslated(0.0, 0.0, kCursorHeight/3);
    gluSphere(qobj, kCursorRadius/3, kCursorTess, kCursorTess);
    gluCylinder(qobj, 0.0, kCursorRadius/2, kCursorHeight/5.0, kCursorTess, kCursorTess);
    glTranslated(0.0, 0.0, kCursorHeight/5);
    gluCylinder(qobj, kCursorRadius/2, 0.0, kCursorHeight/5.0, kCursorTess, kCursorTess);
    glPopMatrix();

    gluDeleteQuadric(qobj);
    glEndList();
  }
  
  // Get the proxy transform in world coordinates.
  hlGetDoublev(HL_PROXY_TRANSFORM, proxyTransform);

  proxyTransform[14]= 0.0;
  glMultMatrixd(proxyTransform);

  // Apply the local cursor scale factor.
  glScaled(gCursorScale, gCursorScale, gCursorScale);

  hdGetDoublev(HD_CURRENT_POSITION, position);

  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glColor3f(0.0, 0.5, 1.0);

  glCallList(gCursorDisplayList);

  glPopMatrix(); 
  glPopAttrib();
}

