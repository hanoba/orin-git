cls
set SCRIPT=C:\DriveX\GitHub\orin-git\HostSim\%1 --/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error
pushd \bin\isaac-sim
call python.bat %SCRIPT%
popd
