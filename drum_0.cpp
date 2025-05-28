//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2024, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Federico Barbagli
    \author    Chris Sewell
    \author    Francois Conti
    \version   3.3.0
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
#include "math/CMaths.h"
#include <iostream>
#include <time.h>
#include <random>
using namespace Eigen::internal;
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in a window display
cCamera* camera;

// a viewport to display the scene viewed by the camera
cViewport* viewport = nullptr;

// a light source to illuminate the objects in the virtual scene
cSpotLight* light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a scope to monitor the sound signal
cScope* scope;

// a virtual body
cMultiMesh* body;

// a virtual vinyl
cMesh* membrane;

// a virtual Rim
cMesh* rim;

// audio device to play sound
cAudioDevice* audioDevice;

// audio buffers to store a forward and backward sound files
cAudioBuffer* audioBufferFwd;
cAudioBuffer* audioBufferRim;

// audio sources which plays the forward and backward audio buffers
cAudioSource* audioSourceFwd;
cAudioSource* audioSourceRim;

// Global variables for the audio stream
int membrane_direction = 1;
unsigned int pos = 0;

// angular velocity of the membrane
double angVel = 0;

// angular position of the membrane
double angPos = 0;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = nullptr;

// current size of GLFW window
int windowW = 0;
int windowH = 0;

// current size of GLFW framebuffer
int framebufferW = 0;
int framebufferH = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window is resized
void onWindowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when the window framebuffer is resized
void onFrameBufferSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void onErrorCallback(int a_error, const char* a_description);

// callback when a key is pressed
void onKeyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback when window content scaling is modified
void onWindowContentScaleCallback(GLFWwindow* a_window, float a_xscale, float a_yscale);

// this function renders the scene
void renderGraphics(void);

// this function contains the main haptics simulation loop
void renderHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
    DEMO:    turntable.cpp

    This example demonstrates the use of friction, animation, and
    sound effects.  With the haptic device you can spin the membrane
    back and forth and at different speeds.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 24-turntable" << endl;
    cout << "Copyright 2003-2024" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // get current path
    bool fileload;
    string currentpath = cGetCurrentPath();


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set GLFW error callback
    glfwSetErrorCallback(onErrorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    windowW = 0.8 * mode->height;
    windowH = 0.5 * mode->height;
    int x = 0.5 * (mode->width - windowW);
    int y = 0.5 * (mode->height - windowH);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // enable double buffering
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    // set the desired number of samples to use for multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);

    // specify that window should be resized based on monitor content scale
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(windowW, windowH, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // set GLFW key callback
    glfwSetKeyCallback(window, onKeyCallback);

    // set GLFW window size callback
    glfwSetWindowSizeCallback(window, onWindowSizeCallback);

    // set GLFW framebuffer size callback
    glfwSetFramebufferSizeCallback(window, onFrameBufferSizeCallback);

    // set GLFW window content scaling callback
    glfwSetWindowContentScaleCallback(window, onWindowContentScaleCallback);

    // get width and height of window
    glfwGetFramebufferSize(window, &framebufferW, &framebufferH);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set window size
    glfwSetWindowSize(window, windowW, windowH);

    // set GLFW current display context
    glfwMakeContextCurrent(window);

    // set GLFW swap interval for the current display context
    glfwSwapInterval(swapInterval);


#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(1.70 * 0.5, 0.14, 1.10 * 0.8),    // camera position (eye)
        cVector3d(0, 0, 0),    // lookat position (target)
        cVector3d(0.00, 0.00, 1.00));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos(1.5, 0.40, 1.5);

    // define the direction of the light beam
    light->setDir(-2.0, -0.5, -2.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    //light->m_shadowMap->setQualityLow();
    light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(30);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // define the radius of the tool (sphere)
    double toolRadius = 0.01;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"f
    tool->enableDynamicObjects(true);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(0.8);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATE OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


    /////////////////////////////////////////////////////////////////////////
    // BODY OBJECT
    /////////////////////////////////////////////////////////////////////////

    // create a virtual mesh
    body = new cMultiMesh();

    // add object to world
    world->addChild(body);

    // set the position of body object at the center of the world
    body->setLocalPos(0.0, 0.0, 0.0);
    body->rotateAboutGlobalAxisDeg(cVector3d(1, 0, 0), 90);

    // load an object file
    fileload = body->loadFromFile(currentpath + "../resources/models/turntable/drum_new.obj");
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // compute a boundary box
    body->computeBoundaryBox(true);

    // get dimensions of object
    double size = cSub(body->getBoundaryMax(), body->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        body->scale(0.5);
    }

    // setup collision detection algorithm
    body->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    body->setStiffness(0.8 * maxStiffness, true);


    /////////////////////////////////////////////////////////////////////////
    // RIM OBJECT
    /////////////////////////////////////////////////////////////////////////

    rim = new cMesh();
    cCreatePipe(rim, 0.15, 0.05, 0.06, 32, 1);

    /////////////////////////////////////////////////////////////////////////
    // VINYL OBJECT
    /////////////////////////////////////////////////////////////////////////

    // create a mesh for the vynil
    membrane = new cMesh();

    // build mesh using primitives
    cCreateCylinder(membrane, 0.01, 0.215, 36, 1, false, true);
    cCreateDisk(membrane, 0.215, 0.215, 36, cVector3d(0, 0, 0));
    //membrane->setTransparencyLevel(0.0);
    cVector3d min, max;
    body->computeBoundaryBox(true);
    min = body->getBoundaryMin();
    max = body->getBoundaryMax();
    double size1 = cSub(body->getBoundaryMax(), body->getBoundaryMin()).length();

    // domain in X, Y, Z
    double xDomain = max.x() - min.x();
    double yDomain = max.y() - min.y();
    double zDomain = max.z() - min.z();
    std::cout << (zDomain);
    //membrane->setLocalPos(1.0, 0.0, 0.0);
    cVector3d drumPos = body->getLocalPos();
    double drumX = drumPos.x();
    double drumY = drumPos.y();
    double drumZ = drumPos.z();

    membrane->setLocalPos(drumX, drumY, zDomain / 2);
	rim->setLocalPos(drumX, drumY, zDomain / 2 + 1);

    // create texture image
    cTexture2dPtr drumImage = cTexture2d::create();

    // load texture image from file
    fileload = drumImage->loadFromFile(currentpath + "../resources/models/turntable/drum_tex2.jpg");
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // set material properties
    body->m_material->setWhite();
    body->setTexture(drumImage);
    body->setUseTexture(true, true);
    body->setUseMaterial(true, true);


    // create collision detector
    membrane->createAABBCollisionDetector(toolRadius);
	rim->createAABBCollisionDetector(toolRadius);

    // add vynil to world
    world->addChild(membrane);
	world->addChild(rim);

    // position vinyl
    membrane->translate(-0.0, 0, -0.065);
	rim->translate(-0.0, 0, -2);

    // set stiffness properties
    membrane->setStiffness(0.5 * maxStiffness, true);
	rim->setStiffness(0.5 * maxStiffness, true);

    // set static and dynamic friction
    double staticFriction = (double)100 / 100.0;
    double dynamicFriction = (double)100 / 100.0;
    membrane->setFriction(staticFriction, dynamicFriction, true);
	rim->setFriction(staticFriction, dynamicFriction, true);


    /////////////////////////////////////////////////////////////////////////
    // SETUP AUDIO
    /////////////////////////////////////////////////////////////////////////

    // create an audio device to play sounds
    audioDevice = new cAudioDevice();


    //-----------------------------------------------------------------------
    // LOAD FORWARD SOUNDTRACK
    //-----------------------------------------------------------------------

    // create an audio buffer and load audio wave file
    audioBufferFwd = new cAudioBuffer();

    fileload = audioBufferFwd->loadFromFile(currentpath + "../resources/sounds/tom.mp3");
    if (!fileload)
    {
        cout << "Error - Sound file failed to load or initialize correctly." << endl;
        close();
        return (-1);
    }

    // create audio source
    audioSourceFwd = new cAudioSource();

    // assign auio buffer to audio source
    audioSourceFwd->setAudioBuffer(audioBufferFwd);

    // set volume
    audioSourceFwd->setGain(10.0);

    // set speed at which the audio file is played. we will modulate this with the membrane speed.
    //audioSourceFwd->setPitch(0.0);

    // loop audio play
    //audioSourceFwd->setLoop(true);

    // start playing
    //audioSourceFwd->play();


    //-----------------------------------------------------------------------
    // LOAD BACKWARD SOUNDTRACK
    //-----------------------------------------------------------------------

    // create an audio buffer and load audio wave file
    audioBufferRim = new cAudioBuffer();

    fileload = audioBufferRim->loadFromFile(currentpath + "../resources/sounds/rim.mp3");
    if (!fileload)
    {
        cout << "Error - Sound file failed to load or initialize correctly." << endl;
        close();
        return (-1);
    }

    // create audio source
    audioSourceRim = new cAudioSource();

    // assign auio buffer to audio source
    audioSourceRim->setAudioBuffer(audioBufferRim);

    // set volume
    audioSourceRim->setGain(10.0);

    // set speed at which the audio file is played. we will modulate this with the membrane speed.
    //audioSourceBwd->setPitch(0.0);

    // loop audio play
    //audioSourceRim->setLoop(true);

    // start playing
    audioSourceRim->play();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONT_CALIBRI_20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
        cColorf(1.0f, 1.0f, 1.0f),
        cColorf(0.8f, 0.8f, 0.8f),
        cColorf(0.8f, 0.8f, 0.8f));


    //--------------------------------------------------------------------------
    // VIEWPORT DISPLAY
    //--------------------------------------------------------------------------

    // get content scale factor
    float contentScaleW, contentScaleH;
    glfwGetWindowContentScale(window, &contentScaleW, &contentScaleH);

    // create a viewport to display the scene.
    viewport = new cViewport(camera, contentScaleW, contentScaleH);


    //--------------------------------------------------------------------------
    // START HAPTIC SIMULATION THREAD
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(renderHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // render graphics
        renderGraphics();

        // process events
        glfwPollEvents();
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void onWindowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    windowW = a_width;
    windowH = a_height;

    // render scene
    renderGraphics();
}

//------------------------------------------------------------------------------

void onFrameBufferSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update frame buffer size
    framebufferW = a_width;
    framebufferH = a_height;
}

//------------------------------------------------------------------------------

void onWindowContentScaleCallback(GLFWwindow* a_window, float a_xscale, float a_yscale)
{
    // update window content scale factor
    viewport->setContentScale(a_xscale, a_yscale);
}

//------------------------------------------------------------------------------

void onErrorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}
//------------------------------------------------------------------------------

void onKeyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
        }

        // set the desired swap interval and number of samples to use for multisampling
        glfwSwapInterval(swapInterval);
        glfwWindowHint(GLFW_SAMPLES, 4);
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
    delete audioDevice;
    delete audioBufferFwd;
    delete audioBufferRim;
    delete audioSourceFwd;
    delete audioSourceRim;
}

//------------------------------------------------------------------------------

void renderGraphics(void)
{
    // sanity check
    if (viewport == nullptr) { return; }

    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // get width and height of CHAI3D internal rendering buffer
    int displayW = viewport->getDisplayWidth();
    int displayH = viewport->getDisplayHeight();

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (displayW - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    viewport->renderView(framebufferW, framebufferH);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) cout << "Error: " << gluErrorString(error) << endl;

    // swap buffers
    glfwSwapBuffers(window);

    // signal frequency counter
    freqCounterGraphics.signal(1);
}

//------------------------------------------------------------------------------


void renderHaptics(void)
{

    // reset clock
    cPrecisionClock clock;
    clock.reset();
    static cVector3d prevToolPos = tool->getDeviceGlobalPos();
    cVector3d currToolPos = tool->getDeviceGlobalPos();
	static double prevTime = clock.getCurrentTimeSeconds();

    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // main haptic simulation loop
    while (simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

        // restart the simulation clock
        clock.reset();
        clock.start();

        // signal frequency counter
        freqCounterHaptics.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();



        /////////////////////////////////////////////////////////////////////
        // ANIMATION
        /////////////////////////////////////////////////////////////////////

        // init
        cVector3d torque(0, 0, 0);

        cVector3d toolForce = -tool->getDeviceGlobalForce();

        bool wasInContactLastFrameMem = false;

        bool isInContactNowMem = tool->isInContact(membrane);

        bool wasInContactLastFrameRim = false;

        bool isInContactNowRim = tool->isInContact(body);


        // figure out if we're touching the membrane
        if (isInContactNowMem && !wasInContactLastFrameMem)
        {
            wasInContactLastFrameMem = true;
            cVector3d currToolPos = tool->getDeviceGlobalPos();
            double currTime = clock.getCurrentTimeSeconds();

            // Compute velocity
            double deltaTime = currTime - prevTime;
            cVector3d toolVelocity(0, 0, 0);
            if (deltaTime > 1e-6) {
                toolVelocity = (currToolPos - prevToolPos) / deltaTime;
            }
			cout << "Tool Velocity: " << toolVelocity << endl;

            // Save for next iteration
            prevToolPos = currToolPos;
            prevTime = currTime;
            cVector3d toolForce = -tool->getDeviceGlobalForce();
            //cout << "Force: " << toolForce.length() << endl;
            //double impactThreshold = 0.5; // adjust sensitivity if needed

            if (toolVelocity.length() > 0)
            {
                // Restart tom.mp3 playback immediately on first contact
                //audioSourceFwd->stop();  // Reset playback
                audioSourceFwd->setGain(toolVelocity.length() * 0.1);
                //audioSourceFwd->setPitch(1.0);
                audioSourceFwd->play();
                
            }
            // Get global positions
            cVector3d toolPos = tool->getDeviceGlobalPos();
            cVector3d membranePos = membrane->getGlobalPos();

            // Only consider vertical direction (Z)
            double penetration = membranePos.z() + 0.01 - toolPos.z(); // 0.01 = vinyl thickness offset

            // If tool is inside the membrane surface, apply bounce force

            double stiffness = 50.0;  // Higher = stronger bounce
            double bounceForceZ = stiffness * penetration;
            cout << "Bounce Force: " << bounceForceZ << endl;
            const long max_rand = 5;
            double random_double_x = 0.1
                + (0.3)
                * (rand() % max_rand)
                / max_rand;
            double random_double_y = 0.1
                + (0.3)
                * (rand() % max_rand)
                / max_rand;

            tool->setDeviceGlobalForce(random_double_x * bounceForceZ, random_double_y * bounceForceZ, bounceForceZ);
            // Apply upward force only
            //cVector3d bounceForce(0.0, 0.0, bounceForceZ);
            //cVector3d basePos = membrane->getLocalPos();
            //tool->setLocalPos(basePos.x(), basePos.y(), basePos.z() + 0.5);
        }
        else if (isInContactNowRim && !wasInContactLastFrameRim)
        {
            wasInContactLastFrameRim = true;
            cVector3d toolForce = -tool->getDeviceGlobalForce();
            //cout << "Force: " << toolForce.length() << endl;
            //double impactThreshold = 0.5; // adjust sensitivity if needed
            cVector3d currToolPos = tool->getDeviceGlobalPos();
            double currTime = clock.getCurrentTimeSeconds();

            // Compute velocity
            double deltaTime = currTime - prevTime;
            cVector3d toolVelocity(0, 0, 0);
            if (deltaTime > 1e-6) {
                toolVelocity = (currToolPos - prevToolPos) / deltaTime;
            }
            cout << "Tool Velocity: " << toolVelocity << endl;

            // Save for next iteration
            prevToolPos = currToolPos;
            prevTime = currTime;
            //cout << "Force: " << toolForce.length() << endl;
            //double impactThreshold = 0.5; // adjust sensitivity if needed

            if (toolVelocity.length() > 0)
            {
                // Restart tom.mp3 playback immediately on first contact
                //audioSourceFwd->stop();  // Reset playback
                audioSourceFwd->setGain(toolVelocity.length() * 0.1);
                //audioSourceFwd->setPitch(1.0);
                audioSourceFwd->play();
			}

            /*
            if (toolForce.length() > 0)
            {
                // Restart tom.mp3 playback immediately on first contact
                audioSourceRim->stop();  // Reset playback
                audioSourceRim->setGain(toolForce.length() * 0.1);
                //audioSourceFwd->setPitch(1.0);
                audioSourceRim->play();
            }
            */
        }
      

        /*
/////////////////////////////////////////////////////////////////////
// BOUNCE SIMULATION
/////////////////////////////////////////////////////////////////////

        // Bounce physics variables
        double bounceZ = 1.0;
        double bounceVelZ = 0.0;
        double membraneMass = 0.1;           // in kg
        double springK = 50.0;            // spring stiffness
        double damping = 1.0;              // damping coefficient
        double gravity = -9.81;            // optional gravity
        tool->applyToDevice();
// Get contact force
        cVector3d contactForce(0, 0, 0);
        if (tool->isInContact(membrane))
        {
            contactForce = -tool->getDeviceGlobalForce();
        }

        // Only vertical (Z) force affects bounce
        double forceZ = contactForce.z();

        // Apply spring-damper physics (F = m*a)
        double accZ = (forceZ - springK * bounceZ - damping * bounceVelZ) / membraneMass;

        // Integrate velocity and position
        bounceVelZ += accZ * timeInterval;
        bounceZ += bounceVelZ * timeInterval;

        // Clamp bounce to avoid sinking
        if (bounceZ < 0.0)
        {
            bounceZ = 0.0;
            bounceVelZ = 0.0;
        }

        */
        //wasInContactLastFrameMem = isInContactNowMem;
        //wasInContactLastFrameRim = isInContactNowRim;
    }


    // update rotational acceleration

    // update rotational velocity

    // set a threshold on the rotational velocity term

    // compute the next rotation of the torus

    // update position of membrane


    // set audio pitch and volume of forward and backward soundtrack based on rotational velocity


// exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
