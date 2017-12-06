// Simple CLI app to tele operate the jaco arm from the omega.7

#include <Windows.h>
#include <conio.h>
#include <iostream>

// CHAI3D header
#include "chai3d.h"

// JACO2 Headers
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include "KinovaTypes.h"


#define KEY_ESCAPE	27


using namespace chai3d;
using namespace std;


// maximum number of devices supported by this application
const int MAX_DEVICES = 16;

// DECLARED VARIABLES

// a world that contains all objects of the virtual environment
cWorld* world;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];

// number of haptic devices detected
int numHapticDevices = 0;

// global variable to store the position [m] of each haptic device
cVector3d hapticDevicePosition[MAX_DEVICES];

// some small spheres (cursor) representing position of each haptic device
cShapeSphere* cursor[MAX_DEVICES];

// some lines representing the velocity vector of each haptic device
cShapeLine* velocity[MAX_DEVICES];

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;


// DECLARED FUNCTIONS

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);




//A handle to the API.
HINSTANCE commandLayer_handle;

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetCartesianCommand)(CartesianPosition &);





// Get Input
bool getInput(char *c)
{
	if (_kbhit())
	{
		*c = _getch();
		return true;
	}
	return false;
}


int main(int argc, char* argv[])
{
    // INITIALIZATION
	cout << "Teleop CLI program" << endl << endl;

	cout << "Press [Esc] or [q] to exit" << endl;
	cout << "Press [f] to force field" << endl;
	cout << "Press [d] to toggle damping" << endl;

	cout << endl;


	// We load the API.
	commandLayer_handle = LoadLibrary("CommandLayerWindows.dll");

	// We load the functions from the library (Under Windows, use GetProcAddress)
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");

	// Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) ||
		(MyMoveHome == NULL) || (MyInitFingers == NULL))

	{
		cout << "Error loading JACO2 API" << endl;
		return 0;
	}

	// Do Jaco stuff

	CartesianPosition currentCommand;

	int result = (*MyInitAPI)();

	cout << "Initialization's result:" << result << endl;

	KinovaDevice list[MAX_KINOVA_DEVICE];

	int devicesCount = MyGetDevices(list, result);

	for (int i=0; i<devicesCount; i++)
	{
		cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

		// Setting the current device as the active device.
		MySetActiveDevice(list[i]);

		cout << "Send the robot to HOME position" << endl;
		MyMoveHome();

		cout << "Initializing the fingers" << endl;
		MyInitFingers();

		TrajectoryPoint pointToSend;
		pointToSend.InitStruct();

		//We specify that this point will be used an angular(joint by joint) velocity vector.
		pointToSend.Position.Type = CARTESIAN_VELOCITY;

		pointToSend.Position.CartesianPosition.X = 0;
		pointToSend.Position.CartesianPosition.Y = -0.15; //Move along Y axis at 20 cm per second
		pointToSend.Position.CartesianPosition.Z = 0;
		pointToSend.Position.CartesianPosition.ThetaX = 0;
		pointToSend.Position.CartesianPosition.ThetaY = 0;
		pointToSend.Position.CartesianPosition.ThetaZ = 0;

		pointToSend.Position.Fingers.Finger1 = 0;
		pointToSend.Position.Fingers.Finger2 = 0;
		pointToSend.Position.Fingers.Finger3 = 0;

		for (int i=0; i<200; i++)
		{
			// We send the velocity vector every 5 ms as long as we want
			// the robot to move along that vector
			MySendBasicTrajectory(pointToSend);
			Sleep(5);
		}

		pointToSend.Position.CartesianPosition.Y = 0;
		pointToSend.Position.CartesianPosition.Z = 0.1;

		for (int i = 0; i < 200; i++)
		{
			// We send the velocity vector every 5 ms as long as we want the
			// robot to move along that vector
			MySendBasicTrajectory(pointToSend);
			Sleep(5);
		}

		cout << "Send the robot to HOME position" << endl;
		MyMoveHome();

		// We specify that this point will be an angular(joint by joint) position
		pointToSend.Position.Type = CARTESIAN_POSITION;

		// We get the actual angular command of the robot
		MyGetCartesianCommand(currentCommand);

		pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
		pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y - 0.1f;
		pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
		pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
		pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
		pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ;

		cout << "*********************************" << endl;
		cout << "Sending the first point to the robot." << endl;
		MySendBasicTrajectory(pointToSend);

		pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z + 0.1f;
		cout << "Sending the second point to the robot." << endl;
		MySendBasicTrajectory(pointToSend);

		cout << "*********************************" << endl << endl << endl;
	}

	cout << endl << "Closing JACO2 API" << endl;
	result = (*MyCloseAPI)();

	FreeLibrary(commandLayer_handle);

    // WORLD

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // HAPTIC DEVICES

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get number of haptic devices
    numHapticDevices = handler->getNumDevices();

    // setup each haptic device
    for (int i=0; i<numHapticDevices; i++)
    {
        // get a handle to the first haptic device
        handler->getDevice(hapticDevice[i], i);

        // open a connection to haptic device
        hapticDevice[i]->open();

        // calibrate device (if necessary)
        hapticDevice[i]->calibrate();

        // retrieve information about the current haptic device
        cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

        // create a sphere (cursor) to represent the haptic device
        cursor[i] = new cShapeSphere(0.01);

        // insert cursor inside world
        world->addChild(cursor[i]);

        // create small line to illustrate the velocity of the haptic device
        velocity[i] = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));

        // insert line inside world
        world->addChild(velocity[i]);

        // display a reference frame if haptic device supports orientations
        if (info.m_sensedRotation == true)
        {
            // display reference frame
            cursor[i]->setShowFrame(true);

            // set the size of the reference frame
            cursor[i]->setFrameSize(0.05);
        }

        // if the device has a gripper, enable the gripper to simulate a user switch
        hapticDevice[i]->setEnableGripperUserSwitch(true);
    }

    // START SIMULATION

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    // MAIN LOOP
	char key = ' ';
    while(true)
    {
		if (getInput(&key))
		{
			// Handle key input
			//cout << endl << "You Pressed: " << key << endl;


			if ((key == KEY_ESCAPE) || (key == 'q'))
			{
				// Quit
				cout << '\r';
				cout << "Exiting...";
				cout << "                                                            " << endl;
				break;
			}
			else if (key == 'd')
			{
				useDamping = !useDamping;

				cout << '\r';
				cout << "Damping ";
				if (useDamping) cout << "enabled";
				else cout << "disabled";
				cout << "                                                            " << endl;
			}
			else if (key == 'f')
			{
				useForceField = !useForceField;

				cout << '\r';
				cout << "Force field ";
				if (useForceField) cout << "enabled";
				else cout << "disabled";
				cout << "                                                            " << endl;
			}
		}

		// Output position data
		for (int i = 0; i<numHapticDevices; i++)
		{
			// Rollback a line to not flood the terminal
			cout << '\r';
			cout << "Device position: " << hapticDevicePosition[i].str(3);
		}

    }

    // exit
    return (0);
}



void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    for (int i=0; i<numHapticDevices; i++)
    {
        hapticDevice[i]->close();
    }

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}


void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        for (int i=0; i<numHapticDevices; i++)
        {
            /////////////////////////////////////////////////////////////////////
            // READ HAPTIC DEVICE
            /////////////////////////////////////////////////////////////////////

            // read position 
            cVector3d position;
            hapticDevice[i]->getPosition(position);

            // read orientation 
            cMatrix3d rotation;
            hapticDevice[i]->getRotation(rotation);

            // read gripper position
            double gripperAngle;
            hapticDevice[i]->getGripperAngleRad(gripperAngle);

            // read linear velocity 
            cVector3d linearVelocity;
            hapticDevice[i]->getLinearVelocity(linearVelocity);

            // read angular velocity
            cVector3d angularVelocity;
            hapticDevice[i]->getAngularVelocity(angularVelocity);

            // read gripper angular velocity
            double gripperAngularVelocity;
            hapticDevice[i]->getGripperAngularVelocity(gripperAngularVelocity);

            // read user-switch status (button 0)
            bool button0, button1, button2, button3;
            button0 = false;
            button1 = false;
            button2 = false;
            button3 = false;

            hapticDevice[i]->getUserSwitch(0, button0);
            hapticDevice[i]->getUserSwitch(1, button1);
            hapticDevice[i]->getUserSwitch(2, button2);
            hapticDevice[i]->getUserSwitch(3, button3);


            /////////////////////////////////////////////////////////////////////
            // UPDATE 3D CURSOR MODEL
            /////////////////////////////////////////////////////////////////////
       
            // update arrow
            velocity[i]->m_pointA = position;
            velocity[i]->m_pointB = cAdd(position, linearVelocity);

            // update position and orientation of cursor
            cursor[i]->setLocalPos(position);
            cursor[i]->setLocalRot(rotation);

            // adjust the  color of the cursor according to the status of
            // the user-switch (ON = TRUE / OFF = FALSE)
            if (button0)
            {
                cursor[i]->m_material->setGreenMediumAquamarine(); 
            }
            else if (button1)
            {
                cursor[i]->m_material->setYellowGold();
            }
            else if (button2)
            {
                cursor[i]->m_material->setOrangeCoral();
            }
            else if (button3)
            {
                cursor[i]->m_material->setPurpleLavender();
            }
            else
            {
                cursor[i]->m_material->setBlueRoyal();
            }

            // update global variable for graphic display update
            hapticDevicePosition[i] = position;


            /////////////////////////////////////////////////////////////////////
            // COMPUTE AND APPLY FORCES
            /////////////////////////////////////////////////////////////////////
            
            // desired position
            cVector3d desiredPosition;
            desiredPosition.set(0.0, 0.0, 0.0);

            // desired orientation
            cMatrix3d desiredRotation;
            desiredRotation.identity();

            // variables for forces    
            cVector3d force (0,0,0);
            cVector3d torque (0,0,0);
            double gripperForce = 0.0;

            // apply force field
            if (useForceField)
            {
                // compute linear force
                double Kp = 25; // [N/m]
                cVector3d forceField = Kp * (desiredPosition - position);
                force.add(forceField);

                // compute angular torque
                double Kr = 0.05; // [N/m.rad]
                cVector3d axis;
                double angle;
                cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
                deltaRotation.toAxisAngle(axis, angle);
                torque = rotation * ((Kr * angle) * axis);
            }
    
            // apply damping term
            if (useDamping)
            {
                cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

                // compute linear damping force
                double Kv = 1.0 * info.m_maxLinearDamping;
                cVector3d forceDamping = -Kv * linearVelocity;
                force.add(forceDamping);

                // compute angular damping force
                double Kvr = 1.0 * info.m_maxAngularDamping;
                cVector3d torqueDamping = -Kvr * angularVelocity;
                torque.add(torqueDamping);

                // compute gripper angular damping force
                double Kvg = 1.0 * info.m_maxGripperAngularDamping;
                gripperForce = gripperForce - Kvg * gripperAngularVelocity;
            }

            // send computed force, torque, and gripper force to haptic device
            hapticDevice[i]->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
        }

        // update frequency counter
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}
