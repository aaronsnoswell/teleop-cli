// Simple CLI app to tele operate the jaco arm from the omega.7

#include <conio.h>
#include <thread>

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

// Global variables
cWorld* world;
cHapticDeviceHandler* handler;
cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];
int kinovaDeviceCount = 0;
int hapticDeviceCount = 0;
cVector3d hapticDevicePosition[MAX_DEVICES];
cShapeSphere* cursor[MAX_DEVICES];
cShapeLine* velocity[MAX_DEVICES];

// Demo configuration flags
bool useDamping = false;
bool useForceField = true;

// Haptic thread variables
bool hapticSimulationRunning = false;
bool hapticSimulationFinished = true;
cFrequencyCounter freqCounterHaptics;
cThread* hapticsThread;

// JACO thread variables
bool JACOSimulationRunning = false;
bool JACOSimulationFinished = true;
cFrequencyCounter freqCounterJACO;
thread JACOThread;

#pragma endregion


// Called when things are shutting down
void close(void)
{
	// Stop the device threads
	hapticSimulationRunning = false;
	JACOSimulationRunning = false;

	// Wait for threads to terminate
	while (!hapticSimulationFinished) { cSleepMs(100); }
	while (!JACOSimulationFinished) { cSleepMs(100); }
	JACOThread.join();

	// Close the JACO device
	(*MyCloseAPI)();
	UnloadSymbols();

	// Close haptic devices
	for (int i = 0; i<hapticDeviceCount; i++)
	{
		hapticDevice[i]->close();

		// Delete per-device heap resources
		delete cursor[i];
		delete velocity[i];
	}

	// Delete heap resources
	delete hapticsThread;
	delete world;
	delete handler;
	delete [] &cursor;
	delete [] &velocity;
}


// Haptic thread function
void updateHaptics(void)
{
	// Haptic simulation in now running
	hapticSimulationRunning = true;
	hapticSimulationFinished = false;

	// main haptic simulation loop
	while (hapticSimulationRunning)
	{
		for (int i = 0; i<hapticDeviceCount; i++)
		{
			// Read status
			cVector3d position;
			cMatrix3d rotation;
			double gripperAngle;
			cVector3d linearVelocity;
			cVector3d angularVelocity;
			double gripperAngularVelocity;
			bool button0, button1, button2, button3;
			button0 = false;
			button1 = false;
			button2 = false;
			button3 = false;

			hapticDevice[i]->getPosition(position);
			hapticDevice[i]->getRotation(rotation);
			hapticDevice[i]->getGripperAngleRad(gripperAngle);
			hapticDevice[i]->getLinearVelocity(linearVelocity);
			hapticDevice[i]->getAngularVelocity(angularVelocity);
			hapticDevice[i]->getGripperAngularVelocity(gripperAngularVelocity);
			hapticDevice[i]->getUserSwitch(0, button0);
			hapticDevice[i]->getUserSwitch(1, button1);
			hapticDevice[i]->getUserSwitch(2, button2);
			hapticDevice[i]->getUserSwitch(3, button3);

			// Update visualisations
			velocity[i]->m_pointA = position;
			velocity[i]->m_pointB = cAdd(position, linearVelocity);
			cursor[i]->setLocalPos(position);
			cursor[i]->setLocalRot(rotation);

			// Adjust the  color of the cursor according to user switch
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

			// Update global variable for graphic display update
			hapticDevicePosition[i] = position;

			// COMPUTE AND APPLY FORCES

			// Desired state
			cVector3d desiredPosition;
			desiredPosition.set(0.0, 0.0, 0.0);

			cMatrix3d desiredRotation;
			desiredRotation.identity();

			// Variables for forces    
			cVector3d force(0, 0, 0);
			cVector3d torque(0, 0, 0);
			double gripperForce = 0.0;

			// Apply force field
			if (useForceField)
			{
				// Compute linear force
				double Kp = 25; // [N/m]
				cVector3d forceField = Kp * (desiredPosition - position);
				force.add(forceField);

				// Compute angular torque
				double Kr = 0.05; // [N/m.rad]
				cVector3d axis;
				double angle;
				cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
				deltaRotation.toAxisAngle(axis, angle);
				torque = rotation * ((Kr * angle) * axis);
			}

			// Apply damping term
			if (useDamping)
			{
				cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

				// Compute linear damping force
				double Kv = 1.0 * info.m_maxLinearDamping;
				cVector3d forceDamping = -Kv * linearVelocity;
				force.add(forceDamping);

				// Compute angular damping force
				double Kvr = 1.0 * info.m_maxAngularDamping;
				cVector3d torqueDamping = -Kvr * angularVelocity;
				torque.add(torqueDamping);

				// Compute gripper angular damping force
				double Kvg = 1.0 * info.m_maxGripperAngularDamping;
				gripperForce = gripperForce - Kvg * gripperAngularVelocity;
			}

			// Send computed force, torque, and gripper force to haptic device
			hapticDevice[i]->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
		}

		// Update frequency counter
		freqCounterHaptics.signal(1);
	}

	// Exit haptics thread
	hapticSimulationFinished = true;
}


// JACO thread function
void updateJACO(void)
{
	// JACO simulation in now running
	JACOSimulationRunning = true;
	JACOSimulationFinished = false;

	// JACO initialisation
	MyMoveHome();
	MyInitFingers();

	// Get initial robot pose
	CartesianPosition initialCommand;
	MyGetCartesianCommand(initialCommand);

	// Prepare a point for us to command it with
	TrajectoryPoint pointToSend;
	pointToSend.InitStruct();
	pointToSend.Position.Type = CARTESIAN_POSITION;

	// main haptic simulation loop
	while (JACOSimulationRunning)
	{
		// Get the current robot command
		CartesianPosition currentCommand;
		MyGetCartesianCommand(currentCommand);

		// Get master position
		cVector3d pos = hapticDevicePosition[0];

		// Set the commanded position
		pointToSend.Position.CartesianPosition.X = initialCommand.Coordinates.X + pos.x();
		pointToSend.Position.CartesianPosition.Y = initialCommand.Coordinates.Y + pos.y();
		pointToSend.Position.CartesianPosition.Z = initialCommand.Coordinates.Z + pos.z();
		pointToSend.Position.CartesianPosition.ThetaX = initialCommand.Coordinates.ThetaX;
		pointToSend.Position.CartesianPosition.ThetaY = initialCommand.Coordinates.ThetaY;
		pointToSend.Position.CartesianPosition.ThetaZ = initialCommand.Coordinates.ThetaZ;

		// Send it
		MySendBasicTrajectory(pointToSend);

		// Update frequency counter
		freqCounterJACO.signal(1);

		// Run Jaco inner loop at 200Hz
		Sleep(5);
	}

	JACOSimulationFinished = true;
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

	// JACO setup
	if (!LoadSymbols(JACOSDK_USB)) return 0;

	int result = (*MyInitAPI)();
	KinovaDevice list[MAX_KINOVA_DEVICE];
	int kinovaDeviceCount = MyGetDevices(list, result);

	if (kinovaDeviceCount == 0)
	{
		cerr << "Couldn't find any JACO devices" << endl;
		return 0;
	}

	cout << "Using Kinova device " << list[0].SerialNumber << " on USB bus" << endl;
	MySetActiveDevice(list[0]);

    // Haptics setup
    world = new cWorld();
    handler = new cHapticDeviceHandler();
	hapticDeviceCount = handler->getNumDevices();

	if (hapticDeviceCount == 0)
	{
		cerr << "Couldn't find any haptic devices" << endl;
		return 0;
	}

    for (int i=0; i<hapticDeviceCount; i++)
    {
		// Initialize device
        handler->getDevice(hapticDevice[i], i);
        hapticDevice[i]->open();
        hapticDevice[i]->calibrate();
        cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

		// Add a cursor and velocity visulaisation
        cursor[i] = new cShapeSphere(0.01);
        world->addChild(cursor[i]);
		velocity[i] = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
		world->addChild(velocity[i]);

        // Show a frame if haptic device supports orientations
        if (info.m_sensedRotation == true)
        {
            cursor[i]->setShowFrame(true);
            cursor[i]->setFrameSize(0.05);
        }

        // If the device has a gripper, enable the gripper to simulate a user switch
        hapticDevice[i]->setEnableGripperUserSwitch(true);
    }

	// Setup callback when application exits
	atexit(close);

    // Launch threads
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	JACOThread = thread(updateJACO);

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

		// Get master position
		cVector3d pos = hapticDevicePosition[0];

		// Output current pos
		cout << '\r';
		cout << "Master position: " << hapticDevicePosition[0].str(3);

		/*
		printf(
			" Robot position: %f %f %f",
			currentCommand.Coordinates.X,
			currentCommand.Coordinates.Y,
			currentCommand.Coordinates.Z
		);
		*/

		// Don't slam the CPU
		Sleep(100);
    }

    // exit
    return (0);
}
