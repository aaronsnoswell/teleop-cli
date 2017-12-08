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

	double gripperAngleRad = 0;
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

		d->getGripperAngleRad(gripperAngleRad);
		d->getGripperAngularVelocity(gripperAngularVelocity);
		d->getGripperForce(gripperForce);

		d->getUserSwitch(0, button0);
		d->getUserSwitch(1, button1);
		d->getUserSwitch(2, button2);
		d->getUserSwitch(3, button3);
	};

};


// A simple enum for selecting the current JACO control mode
enum ControlMode
{
	E_POSITION,
	E_VELOCITY,
	E_FORCE
};


#pragma region Declarations

// Global variables
cWorld* world;
cHapticDeviceHandler* handler;
cGenericHapticDevicePtr hapticDevice[MAX_HAPTIC_DEVICES];
cHapticDeviceInfo hapticDeviceSpecification[MAX_HAPTIC_DEVICES];
HapticDevicePose hapticDevicePose[MAX_HAPTIC_DEVICES];
int kinovaDeviceCount = 0;
int hapticDeviceCount = 0;

// Demo configuration flags
bool useDamping = false;
bool useForceField = true;
ControlMode JACOControlMode = ControlMode::E_POSITION;
bool useGripperControl = false;
double masterWorkspaceScaling = 1.0f;

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
	}

	// Delete heap resources
	delete hapticsThread;
	delete world;
	delete handler;
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
				double Kp = 100; // [N/m]
				cVector3d forceField = Kp * (desiredPosition - hapticDevicePose[i].position);
				force.add(forceField);

				// Compute angular torque
				double Kr = 0.2; // [N/m.rad]
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

	cout << "Initializing JACO arm" << endl;

	// JACO initialisation
	MySetCartesianControl();
	MySendBasicTrajectory(zeroPose);

	cout << endl << "JACO initialization done" << endl;

	// Main JACO2 loop
	while (JACOSimulationRunning)
	{
		// Update frequency counter
		freqCounterJACO.signal(1);

		// Get master pose
		// XXX TODO ajs 07/Dec/17 Race condition here
		HapticDevicePose masterPose = hapticDevicePose[0];

		// Rotation matrix to go from the haptic master to the JACO base
		/* XXX ajs 08/Dec/2017 This rotation works for a right-hand haptic master,
		 * but might need to be inverted (-90 degrees) for a left-handed one?
		 */
		cMatrix3d masterToJACOBase(
			0,
			0,
			90,
			cEulerOrder::C_EULER_ORDER_XYZ,
			false,
			true
		);

		// Get the current robot command
		switch (JACOControlMode)
		{
			case E_FORCE:
			{
				cerr << "Force control not yet implemented" << endl;
				break;
			}
			case E_VELOCITY:
			{
				// XXX ajs 7/Dec/2017 Velocity control not yet tested
				cerr << "Velocity control not yet tested" << endl;
				break;

				/*
				// Transform the master velocity to desired robot velocity
				cVector3d desiredVelocity = masterToJACOBase * masterPose.linearVelocity;
				cVector3d desiredAngularVelocity = masterToJACOBase * masterPose.angularVelocity;
				
				// Set the commanded pose
				pointToSend.Position.Type = CARTESIAN_VELOCITY;
				pointToSend.Position.CartesianPosition.X = desiredVelocity.x();
				pointToSend.Position.CartesianPosition.Y = desiredVelocity.y();
				pointToSend.Position.CartesianPosition.Z = desiredVelocity.z();
				pointToSend.Position.CartesianPosition.ThetaX = desiredAngularVelocity.x();
				pointToSend.Position.CartesianPosition.ThetaY = desiredAngularVelocity.y();
				pointToSend.Position.CartesianPosition.ThetaZ = desiredAngularVelocity.z();

				float fingerCommand = 0;
				if (useGripperControl)
				{
					// XXX ajs 7/Dec/2017 We arbitrarily assume the gripper can't move faster than hapticDeviceSpecification[0].m_gripperMaxAngleRad/second
					float GRIPPER_MAX_ANGULAR_VELOCITY = hapticDeviceSpecification[0].m_gripperMaxAngleRad / 1.0f;

					// XXX ajs 7/Dec/2017 We arbitarily limit the velocity to FINGER_MAX_TURN/second
					float FINGER_MAX_VELOCITY = FINGER_MAX_DIST / 1.0f;

					// Convert gripper angular velocity to [-1 to 1] (0=still, 1=max velocity in closing direction)
					float gripperClosingVelocity = masterPose.gripperAngularVelocity / GRIPPER_MAX_ANGULAR_VELOCITY;

					// Compute finger velocity
					fingerCommand = gripperClosingVelocity * FINGER_MAX_VELOCITY;
				}

				pointToSend.Position.HandMode = HAND_MODE::VELOCITY_MODE;
				pointToSend.Position.Fingers.Finger1 = fingerCommand;
				pointToSend.Position.Fingers.Finger2 = fingerCommand;
				pointToSend.Position.Fingers.Finger3 = fingerCommand;

				// Send the command
				MySendBasicTrajectory(pointToSend);

				// Velocity control loop runs at 200Hz
				Sleep(5);

				break;
				*/
			}
			case E_POSITION:
			default:
			{
				// Apply workspace scaling
				cVector3d desiredPositionMetersMaster = masterPose.position * masterWorkspaceScaling;
				
				// Transform to robot base frame
				cVector3d desiredPositionMeters = masterToJACOBase * desiredPositionMetersMaster;

				// Get desired Euler angles in master frame
				cVector3d desiredEulerAnglesXYZRadMaster = rotationMatrixToEulerAngles(masterPose.rotation);

				// Transform to robot end effector frame
				cVector3d desiredEulerAnglesXYZRad(
					desiredEulerAnglesXYZRadMaster.y() * -1,
					desiredEulerAnglesXYZRadMaster.z() * +1,
					desiredEulerAnglesXYZRadMaster.x() * -1
				);

				// Set the commanded pose
				TrajectoryPoint pointToSend;
				pointToSend.InitStruct();
				pointToSend.Position.Type = CARTESIAN_POSITION;
				pointToSend.Position.CartesianPosition.X = zeroPose.Position.CartesianPosition.X + desiredPositionMeters.x();
				pointToSend.Position.CartesianPosition.Y = zeroPose.Position.CartesianPosition.Y + desiredPositionMeters.y();
				pointToSend.Position.CartesianPosition.Z = zeroPose.Position.CartesianPosition.Z + desiredPositionMeters.z();

				// JACO Euler Angles are in the end-effector local frame, XYZ order
				pointToSend.Position.CartesianPosition.ThetaX = zeroPose.Position.CartesianPosition.ThetaX + desiredEulerAnglesXYZRad.x();
				pointToSend.Position.CartesianPosition.ThetaY = zeroPose.Position.CartesianPosition.ThetaY + desiredEulerAnglesXYZRad.y();
				pointToSend.Position.CartesianPosition.ThetaZ = zeroPose.Position.CartesianPosition.ThetaZ + desiredEulerAnglesXYZRad.z();

				float fingerCommand = zeroPose.Position.Fingers.Finger1;
				/*
				if (useGripperControl)
				{
					// Convert gripper angle to [0 to 1] (0=open, 1=closed)
					float gripperClosedAmount = masterPose.gripperAngleRad / hapticDeviceSpecification[0].m_gripperMaxAngleRad;

					// Compute and send finger position
					fingerCommand = gripperClosedAmount * FINGER_MAX_DIST;
				}
				*/

				pointToSend.Position.HandMode = HAND_MODE::POSITION_MODE;
				pointToSend.Position.Fingers.Finger1 = fingerCommand;
				pointToSend.Position.Fingers.Finger2 = fingerCommand;
				pointToSend.Position.Fingers.Finger3 = fingerCommand;

				// Send the command
				MySendBasicTrajectory(pointToSend);

				// Position control loop runs at 5Hz
				Sleep(200);

				break;
			}
		}
	}

	//cout << endl << "Go home, JACO..." << endl;
	//MyMoveHome();

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
