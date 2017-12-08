#pragma once

// A simple wrapper for the (ugly) JACO2 API

#include <Windows.h>
#include <iostream>

// JACO2 Headers
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include "KinovaTypes.h"


// The names of the command layer DLLs
#define COMMAND_LAYER_PATH				L"CommandLayerWindows.dll"
#define COMMAND_LAYER_ETHERNET_PATH		L"CommandLayerEthernet.dll"

// 1 rad is this many degrees
#define RAD2DEG	57.29578f;


namespace jacowrapper
{
	using namespace std;

	// These magic numbers are copied from the Kinova ROS drivers
	// See https://github.com/Kinovarobotics/kinova-ros/blob/c1fa13c22626aca41d87491bbb72a3d108fcf509/kinova_demo/nodes/kinova_demo/fingers_action_client.py#L93

	// Maximum distance for one finger in meters
	static float FINGER_MAX_DIST = 18.9f / 2.0f / 1000.0f;

	// Maximum distance for one finger in percent
	static float FINGER_MAX_PERCENT = 100.0f;

	// Maximum thread turn for one finger
	// Used for angular command presumably?
	static float FINGER_MAX_TURN = 6800;

	// A handle to the API.
	HINSTANCE commandLayer_handle;

	// Function pointers to the functions we need
	int(*MyInitAPI)();
	int(*MyCloseAPI)();
	int(*MySendBasicTrajectory)(TrajectoryPoint command);
	int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
	int(*MySetActiveDevice)(KinovaDevice device);
	int(*MyMoveHome)();
	int(*MyInitFingers)();
	int(*MyGetCartesianCommand)(CartesianPosition &);


	// Define a TrajectoryPoint that we use as our zero pose
	// Designated Initializers didn't make it into C++11 ;(
	struct ZeroPoseTrajectoryPoint : TrajectoryPoint
	{
		ZeroPoseTrajectoryPoint()
		{
			InitStruct();
			Position.HandMode = HAND_MODE::POSITION_MODE;
			Position.Type = CARTESIAN_POSITION;
			Position.CartesianPosition.X = 0.1f;
			Position.CartesianPosition.Y = -0.55f;
			Position.CartesianPosition.Z = 0.5f;
			Position.CartesianPosition.ThetaX = 90.0f / RAD2DEG;
			Position.CartesianPosition.ThetaY = 0;
			Position.CartesianPosition.ThetaZ = 0;
			Position.Fingers.Finger1 = FINGER_MAX_TURN / 2.0f;
			Position.Fingers.Finger2 = FINGER_MAX_TURN / 2.0f;
			Position.Fingers.Finger3 = FINGER_MAX_TURN / 2.0f;
		}
	} zeroPose;
	

	// Loads the Jaco API symbols from the DLL
	bool LoadSymbols(bool UseEthernet = false)
	{
		// We load the API
		commandLayer_handle = LoadLibraryW(
			(UseEthernet) ? COMMAND_LAYER_ETHERNET_PATH : COMMAND_LAYER_PATH
		);
		
		if (commandLayer_handle == NULL)
		{
			cerr << "Couldn't load JACO2 DLL - Win32 error " << GetLastError() << endl;
			cerr << "See https://msdn.microsoft.com/en-us/library/windows/desktop/ms681381(v=vs.85).aspx" << endl;
			return false;
		}

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
			cerr << "Couldn't find JACO2 SDK function symbols" << endl;
			return false;
		}

		return true;
	}


	// Unloads the Jaco 2 API symbols
	void UnloadSymbols()
	{
		FreeLibrary(commandLayer_handle);
	}

}
