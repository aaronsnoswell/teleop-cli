// Simple CLI app to tele operate the jaco arm from the omega.7

#include <conio.h>

// CHAI3D header
#include "chai3d.h"

// Custom Jaco wrapper
#include "jacowrapper.h"


// Escape key code
#define KEY_ESCAPE			27

// Maximum haptic devices supported by this application
#define MAX_DEVICES			16

// JACO2 SDK communication mode
#define JACOSDK_USB			false
#define JACOSDK_ETHERNET	true


using namespace chai3d;
using namespace std;
using namespace jacowrapper;


#pragma region Declarations

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

#pragma endregion



// Called when things are shutting down
void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for haptics loop to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	for (int i = 0; i<numHapticDevices; i++)
	{
		hapticDevice[i]->close();
	}

	// Close the Jaco device
	(*MyCloseAPI)();
	UnloadSymbols();

	// delete resources
	delete hapticsThread;
	delete world;
	delete handler;
}


// Haptic thread function
void updateHaptics(void)
{
	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	// main haptic simulation loop
	while (simulationRunning)
	{
		for (int i = 0; i<numHapticDevices; i++)
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
			cVector3d force(0, 0, 0);
			cVector3d torque(0, 0, 0);
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


// Gets input if there is any (does not block)
bool getInput(char *c)
{
	if (_kbhit())
	{
		*c = _getch();
		return true;
	}
	return false;
}


// Main function
int main(int argc, char* argv[])
{
    // INITIALIZATION
	cout << "Teleop CLI program" << endl << endl;

	cout << "Press [Esc] or [q] to exit" << endl;
	cout << "Press [f] to force field" << endl;
	cout << "Press [d] to toggle damping" << endl;

	cout << endl;

	// Load Jaco DLLs
	if (!LoadSymbols(JACOSDK_USB))
	{
		// Error loading JACO2 SDK
		return 0;
	}

	int result = (*MyInitAPI)();

	KinovaDevice list[MAX_KINOVA_DEVICE];
	int devicesCount = MyGetDevices(list, result);

	if (devicesCount == 0)
	{
		cerr << "Couldn't find any JACO devices" << endl;
		return 0;
	}
	
	cout << "Using Kinova device " << list[0].SerialNumber << " on USB bus" << endl;

	// Setting the current device as the active device.
	MySetActiveDevice(list[0]);
	MyMoveHome();
	MyInitFingers();

	// Get initial robot pose
	CartesianPosition initialCommand;
	MyGetCartesianCommand(initialCommand);

	// Prepare a point for us to command it with
	TrajectoryPoint pointToSend;
	pointToSend.InitStruct();
	pointToSend.Position.Type = CARTESIAN_POSITION;

    // ===== Set up haptic world

    // create a new world.
    world = new cWorld();

    // ===== Set up haptic Devices

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

    // ===== START SIMULATION

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

		cVector3d pos = hapticDevicePosition[0];

		// Output current pos
		cout << '\r';
		cout << "Master position: " << hapticDevicePosition[0].str(3);

		CartesianPosition currentCommand;
		MyGetCartesianCommand(currentCommand);

		printf(
			" Robot position: %f %f %f",
			currentCommand.Coordinates.X,
			currentCommand.Coordinates.Y,
			currentCommand.Coordinates.Z
		);

		// Set the commanded position
		pointToSend.Position.CartesianPosition.X = initialCommand.Coordinates.X + pos.x();
		pointToSend.Position.CartesianPosition.Y = initialCommand.Coordinates.Y + pos.y();
		pointToSend.Position.CartesianPosition.Z = initialCommand.Coordinates.Z + pos.z();
		pointToSend.Position.CartesianPosition.ThetaX = initialCommand.Coordinates.ThetaX;
		pointToSend.Position.CartesianPosition.ThetaY = initialCommand.Coordinates.ThetaY;
		pointToSend.Position.CartesianPosition.ThetaZ = initialCommand.Coordinates.ThetaZ;
		
		// Send it
		MySendBasicTrajectory(pointToSend);

		// Run Jaco inner loop at 200Hz
		Sleep(5);
    }

    // exit
    return (0);
}
