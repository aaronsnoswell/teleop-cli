// Simple CLI app to tele operate the jaco arm from the omega.7

#include <conio.h>
#include <thread>

// CHAI3D header
#include "chai3d.h"

// Custom Jaco wrapper
#include "jacowrapper.h"

// Some util functions
#include "utils.h"


// Escape key code
#define KEY_ESCAPE			27

// Maximum haptic devices supported by this application
#define MAX_HAPTIC_DEVICES	16

// JACO2 SDK communication mode
#define JACOSDK_USB			false
#define JACOSDK_ETHERNET	true


using namespace chai3d;
using namespace std;
using namespace jacowrapper;


/**
 * A simple struct to contain the state of a haptic device
 */
struct HapticDevicePose
{
	cVector3d position;
	cMatrix3d rotation;
	cVector3d linearVelocity;
	cVector3d angularVelocity;
	cVector3d force;
	cVector3d torque;

	double gripperAngle = 0;
	double gripperAngularVelocity = 0;
	double gripperForce = 0;

	bool button0 = false;
	bool button1 = false;
	bool button2 = false;
	bool button3 = false;

	void ReadFromDevice(cGenericHapticDevicePtr d)
	{
		d->getPosition(position);
		d->getRotation(rotation);
		d->getLinearVelocity(linearVelocity);
		d->getAngularVelocity(angularVelocity);
		d->getForce(force);
		d->getTorque(torque);

		d->getGripperAngleRad(gripperAngle);
		d->getGripperAngularVelocity(gripperAngularVelocity);
		d->getGripperForce(gripperForce);

		d->getUserSwitch(0, button0);
		d->getUserSwitch(1, button1);
		d->getUserSwitch(2, button2);
		d->getUserSwitch(3, button3);
	};

};


#pragma region Declarations

// Global variables
cWorld* world;
cHapticDeviceHandler* handler;
cGenericHapticDevicePtr hapticDevice[MAX_HAPTIC_DEVICES];
cShapeSphere* cursor[MAX_HAPTIC_DEVICES];
cShapeLine* velocity[MAX_HAPTIC_DEVICES];
HapticDevicePose hapticDevicePose[MAX_HAPTIC_DEVICES];
int kinovaDeviceCount = 0;
int hapticDeviceCount = 0;


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
			hapticDevicePose[i].ReadFromDevice(hapticDevice[i]);

			// Update visualisations
			cursor[i]->setLocalPos(hapticDevicePose[i].position);
			cursor[i]->setLocalRot(hapticDevicePose[i].rotation);

			// Adjust the  color of the cursor according to user switch
			if (hapticDevicePose[i].button0)
			{
				cursor[i]->m_material->setGreenMediumAquamarine();
			}
			else if (hapticDevicePose[i].button1)
			{
				cursor[i]->m_material->setYellowGold();
			}
			else if (hapticDevicePose[i].button2)
			{
				cursor[i]->m_material->setOrangeCoral();
			}
			else if (hapticDevicePose[i].button3)
			{
				cursor[i]->m_material->setPurpleLavender();
			}
			else
			{
				cursor[i]->m_material->setBlueRoyal();
			}

			// Compute forces and torques
			cVector3d force(0, 0, 0);
			cVector3d torque(0, 0, 0);
			double gripperForce = 0.0;

			// Desired state
			cVector3d desiredPosition;
			desiredPosition.set(0.0, 0.0, 0.0);

			cMatrix3d desiredRotation;
			desiredRotation.identity();

			// Apply force field
			if (useForceField)
			{
				// Compute linear force
				double Kp = 25; // [N/m]
				cVector3d forceField = Kp * (desiredPosition - hapticDevicePose[i].position);
				force.add(forceField);

				// Compute angular torque
				double Kr = 0.05; // [N/m.rad]
				cVector3d axis;
				double angle;
				cMatrix3d deltaRotation = cTranspose(hapticDevicePose[i].rotation) * desiredRotation;
				deltaRotation.toAxisAngle(axis, angle);
				torque = hapticDevicePose[i].rotation * ((Kr * angle) * axis);
			}

			// Apply damping term
			if (useDamping)
			{
				cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

				// Compute linear damping force
				double Kv = 1.0 * info.m_maxLinearDamping;
				cVector3d forceDamping = -Kv * hapticDevicePose[i].linearVelocity;
				force.add(forceDamping);

				// Compute angular damping force
				double Kvr = 1.0 * info.m_maxAngularDamping;
				cVector3d torqueDamping = -Kvr * hapticDevicePose[i].angularVelocity;
				torque.add(torqueDamping);

				// Compute gripper angular damping force
				double Kvg = 1.0 * info.m_maxGripperAngularDamping;
				gripperForce = gripperForce - Kvg * hapticDevicePose[i].gripperAngularVelocity;
			}

			// Apply forces and torques (if supported)
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

	// Main JACO2 loop
	while (JACOSimulationRunning)
	{
		// Get master pose
		// XXX TODO ajs 07/Dec/17 Race condition here
		HapticDevicePose masterPose = hapticDevicePose[0];

		// Rotation matrix to go from the haptic master to the JACO base
		cMatrix3d masterToJACOBase(
			0,
			0,
			180,
			cEulerOrder::C_EULER_ORDER_XYZ,
			false,
			true
		);

		// Transform the master pose to desired robot pose
		// XXX ajs 7/Dec/2017 might need to invert masterToJACOBase here
		cVector3d desiredPosition = masterToJACOBase * masterPose.position;
		cMatrix3d desiredRotation = masterToJACOBase * masterPose.rotation;
		cVector3d dediredEulerAnglesXYZ = rotationMatrixToEulerAngles(desiredRotation);

		// Get the current robot command
		CartesianPosition currentCommand;
		MyGetCartesianCommand(currentCommand);

		// Set the commanded pose
		pointToSend.Position.CartesianPosition.X = initialCommand.Coordinates.X + desiredPosition.x();
		pointToSend.Position.CartesianPosition.Y = initialCommand.Coordinates.Y + desiredPosition.y();
		pointToSend.Position.CartesianPosition.Z = initialCommand.Coordinates.Z + desiredPosition.z();
		pointToSend.Position.CartesianPosition.ThetaX = dediredEulerAnglesXYZ.x();
		pointToSend.Position.CartesianPosition.ThetaY = dediredEulerAnglesXYZ.y();
		pointToSend.Position.CartesianPosition.ThetaZ = dediredEulerAnglesXYZ.z();

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
		/* XXX ajs 07/Dec/17 CHAI3D examples seem to imply this
		 * copy operation is cThread safe, but the documentation doesn't
	     * mention this at all. Race condition here?
		 */
		cVector3d pos = hapticDevicePose[0].position;

		// Output current pos
		cout << '\r';
		cout << "Master position: " << pos.str(3);

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
