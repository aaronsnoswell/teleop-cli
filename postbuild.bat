
echo "Copying DLLs to build folder %1"
copy "%JACO2SDKROOT%\CommandLayerWindows.dll" "%1"
copy "%JACO2SDKROOT%\CommandLayerEthernet.dll" "%1"
copy "%JACO2SDKROOT%\CommunicationLayerWindows.dll" "%1"
copy "%JACO2SDKROOT%\CommunicationLayerEthernet.dll" "%1"
