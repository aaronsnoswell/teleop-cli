#pragma once

// A simple wrapper for the (ugly) Jaco Win32 API

#include <Windows.h>
#include <iostream>

// JACO2 Headers
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include "KinovaTypes.h"

namespace jacowrapper
{

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


	// Loads the Jaco API symbols from the DLL
	bool LoadSymbols()
	{
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
