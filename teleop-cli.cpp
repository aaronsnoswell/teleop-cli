// Simple CLI app to tele operate the jaco arm from the omega.7

#include <thread>
#include <chrono>
#include <conio.h>

// CHAI3D header
#include "chai3d.h"

// Custom Jaco wrapper
#include "jacowrapper.h"

// Some util functions
#include "utils.h"


// Escape key code
#define KEY_ESCAPE					27

// Maximum haptic devices supported by this application
#define MAX_HAPTIC_DEVICES			16

// Radius of the Omega.7 gripper rotational axis (empirically measured)
#define OMEGA_7_GRIPPER_RADIUS_MM	54

// JACO2 SDK communication mode
#define JACOSDK_USB					false
#define JACOSDK_ETHERNET			true


using namespace chai3d;
using namespace std;
using namespace jacowrapper;


// We use chrono::high_resolution_clock for timing
typedef std::chrono::high_resolution_clock Clock;


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


#pragma region Declarations

// Global variables
cWorld* world;
cHapticDeviceHandler* handler;
cGenericHapticDevicePtr hapticDevice[MAX_HAPTIC_DEVICES];
cHapticDeviceInfo hapticDeviceSpecification[MAX_HAPTIC_DEVICES];
HapticDevicePose hapticDevicePose[MAX_HAPTIC_DEVICES];
int kinovaDeviceCount = 0;
int hapticDeviceCount = 0;
ClientConfigurations robotConfig;

// Average virtual penetration distance (in 'turns') of the robot fingers
float averageFingerDelta = 0;

// Demo configuration flags
bool useDamping = false;
bool useForceField = true;
bool useGripperControl = true;

double masterRateWorkspaceScaling = 5.0f;
double masterRateWorkspaceAngularRateScaling = 1.0f;
float RateControlLoopHz = 200;

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
		// Update frequency counter
		freqCounterHaptics.signal(1);

		for (int i = 0; i<hapticDeviceCount; i++)
		{
			// Read status
			hapticDevicePose[i].ReadFromDevice(hapticDevice[i]);

			// Compute forces and torques
			cVector3d force(0, 0, 0);
			cVector3d torque(0, 0, 0);
			double gripperForce = 0.0;

			// Get robot finger position delta and caclulate virtual force feedback
			if (useGripperControl)
			{
				/* XXX ajs 11/Dec/2017 The virtual force model, when based on
				 * average virtual finer penetration, gets into oscillatory modes
				 * around the zero point. With large master gripper force values,
				 * this can quickly cause the master gripper cable to come untangled
				 * from the motor drive barrel. Need to find a better way of implementing
				 * virtual force feedback
				 */
				//double Kp = hapticDeviceSpecification[i].m_maxGripperForce * 0.4; // [N]
				//gripperForce += Kp * averageFingerDelta;
				gripperForce = 0;
			}

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
	MyGetClientConfigurations(robotConfig);
	MySendBasicTrajectory(zeroPose);

	cout << endl << "JACO initialization done" << endl;

	CartesianPosition lastPosition;
	FingersPosition estimatedFingerPosition;
	MyGetCartesianPosition(lastPosition);
	estimatedFingerPosition = lastPosition.Fingers;

	auto startTime = Clock::now();
	auto prevTime = startTime;
	double runTimeSeconds = 0;

	/* Matrix that converts from master to EEF frame
	 * (the master frame is a rotated child of the EEF frame,
	 * hence this is constant)
	 */
	cMatrix3d masterToEEF(
		+0, -1, +0,
		+0, +0, +1,
		-1, +0, +0
	);

	// Main JACO2 loop
	while (JACOSimulationRunning)
	{
		auto nowTime = Clock::now();
		runTimeSeconds = chrono::duration<double>(nowTime - startTime).count();
		chrono::seconds deltaSeconds = chrono::duration_cast<chrono::seconds>(nowTime - prevTime);
		
		// Update frequency counter
		freqCounterJACO.signal(1);

		// Get master pose
		// XXX TODO ajs 07/Dec/17 Race condition here
		HapticDevicePose masterPose = hapticDevicePose[0];

		// Scale master position down by maximum workspace radius
		masterPose.position /= hapticDeviceSpecification[0].m_workspaceRadius;

		// Get robot pose
		CartesianPosition currentPose;
		MyGetCartesianCommand(currentPose);

		// We use 'rate' control - the position of the haptic master
		// drives the velocity of the arm
		cVector3d vFinal(0, 0, 0);
		cVector3d rFinal(0, 0, 0);
		
		// Apply square scaling to create a dead zone
		cVector3d masterPosition = cVector3d(
			masterPose.position.x() * fabs(masterPose.position.x()),
			masterPose.position.y() * fabs(masterPose.position.y()),
			masterPose.position.z() * fabs(masterPose.position.z())
		);

		// Apply workspace scaling
		masterPosition = masterPosition * masterRateWorkspaceScaling;

		// Convert to velocity
		cVector3d masterVelocity = masterPosition * robotConfig.MaxTranslationVelocity;
		
		// Matrix that converts from EEF to Base frame
		cMatrix3d EEFToBase(
			currentPose.Coordinates.ThetaX,
			currentPose.Coordinates.ThetaY,
			currentPose.Coordinates.ThetaZ,
			cEulerOrder::C_EULER_ORDER_XYZ,
			true,
			false
		);

		// Convert velocity to EEF frame
		cVector3d velocityInEEFFrame = masterToEEF * masterVelocity;

		// Convet velocity to base frame
		cVector3d velocityInBaseFrame = EEFToBase * velocityInEEFFrame;
		vFinal = velocityInBaseFrame;

		/*
		// Convert master rotation to EEF rotation
		cMatrix3d masterRotationInEEFFrame = masterToEEF * masterPose.rotation;

		// Convert rotation to Euler angles
		cVector3d rotationInEEFFrame = rotationMatrixToEulerAngles(masterRotationInEEFFrame);

		// Apply workspace scaling to get rates
		cVector3d ratesInEEFFrame = rotationInEEFFrame * masterRateWorkspaceAngularRateScaling;

		// Apply square scaling to create a dead zone
		ratesInEEFFrame = cVector3d(
			ratesInEEFFrame.x() * fabs(ratesInEEFFrame.x()),
			ratesInEEFFrame.y() * fabs(ratesInEEFFrame.y()),
			ratesInEEFFrame.z() * fabs(ratesInEEFFrame.z())
		);
		*/

		// Get desired Euler angles in master frame
		cVector3d desiredEulerAnglesXYZRadMaster = rotationMatrixToEulerAngles(masterPose.rotation) * masterRateWorkspaceAngularRateScaling;

		// Transform to robot end effector frame
		cVector3d desiredEulerAnglesXYZRad(
			desiredEulerAnglesXYZRadMaster.y() * -1,
			desiredEulerAnglesXYZRadMaster.z() * +1,
			desiredEulerAnglesXYZRadMaster.x() * -1
		);

		desiredEulerAnglesXYZRad = cVector3d(
			desiredEulerAnglesXYZRad.x() * fabs(desiredEulerAnglesXYZRad.x()),
			desiredEulerAnglesXYZRad.y() * fabs(desiredEulerAnglesXYZRad.y()),
			desiredEulerAnglesXYZRad.z() * fabs(desiredEulerAnglesXYZRad.z())
		);

		rFinal = desiredEulerAnglesXYZRad;
		cout << rFinal.str(3) << endl;

		// Set the commanded pose
		TrajectoryPoint pointToSend;
		pointToSend.InitStruct();
		pointToSend.Position.Type = CARTESIAN_VELOCITY;
		pointToSend.Position.CartesianPosition.X = vFinal.x();
		pointToSend.Position.CartesianPosition.Y = vFinal.y();
		pointToSend.Position.CartesianPosition.Z = vFinal.z();

		// JACO Euler Angles are in the end-effector local frame, XYZ order
		pointToSend.Position.CartesianPosition.ThetaX = rFinal.x();
		pointToSend.Position.CartesianPosition.ThetaY = rFinal.y();
		pointToSend.Position.CartesianPosition.ThetaZ = rFinal.z();

		float fingerCommand = 0;
		if (useGripperControl)
		{
			// Convert gripper angle to [0 to 1] (0=closed, 1=open)
			float gripperOpenAmount = masterPose.gripperAngleRad / hapticDeviceSpecification[0].m_gripperMaxAngleRad;

			// Apply square scaling to gripperOpenAmount to create a dead zone
			gripperOpenAmount *= fabs(gripperOpenAmount);

			// Compute finger velocity based on master position
			// JACO positions are finger-closed = high commands
			fingerCommand = (gripperOpenAmount * -2.0f + 1.0f) * FINGER_MAX_TURN;
		}

		pointToSend.Position.HandMode = HAND_MODE::VELOCITY_MODE;
		pointToSend.Position.Fingers.Finger1 = fingerCommand;
		pointToSend.Position.Fingers.Finger2 = fingerCommand;
		pointToSend.Position.Fingers.Finger3 = fingerCommand;

		// Calculate delta with estimated position this frame
		FingersPosition fingerDelta;
		fingerDelta.Finger1 = estimatedFingerPosition.Finger1 - currentPose.Fingers.Finger1;
		fingerDelta.Finger2 = estimatedFingerPosition.Finger2 - currentPose.Fingers.Finger2;
		fingerDelta.Finger3 = estimatedFingerPosition.Finger3 - currentPose.Fingers.Finger3;
		averageFingerDelta = (fingerDelta.Finger1 + fingerDelta.Finger2 + fingerDelta.Finger3) / 3.0f;
				
		// Normalize penetration delta to +- 1.0
		averageFingerDelta /= FINGER_MAX_TURN;

		// Integrate finger velocities to estimate next frame's finger positions
		estimatedFingerPosition.Finger1 = currentPose.Fingers.Finger1 + fingerCommand;
		estimatedFingerPosition.Finger2 = currentPose.Fingers.Finger2 + fingerCommand;
		estimatedFingerPosition.Finger3 = currentPose.Fingers.Finger3 + fingerCommand;

		// Send the command
		MySendBasicTrajectory(pointToSend);

		// Rate-limit the control loop
		Sleep((DWORD)(1.0f / (RateControlLoopHz * 1e-3)));

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
        hapticDeviceSpecification[i] = hapticDevice[i]->getSpecifications();

        // If the device has a gripper, enable the gripper to simulate a user switch
        hapticDevice[i]->setEnableGripperUserSwitch(false);
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
		//cout << '\r';
		//cout << "Master position: " << pos.str(3);

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
