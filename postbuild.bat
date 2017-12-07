
echo "Copying DLLs to build folder %1"
copy "%JACO2SDKROOT%\API\CommandLayerWindows.dll" "%1"
copy "%JACO2SDKROOT%\API\CommandLayerEthernet.dll" "%1"
copy "%JACO2SDKROOT%\API\CommunicationLayerWindows.dll" "%1"
copy "%JACO2SDKROOT%\API\CommunicationLayerEthernet.dll" "%1"
