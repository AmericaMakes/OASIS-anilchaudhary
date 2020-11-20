REM This batch file will build the scan path generation code in the docker container created using the instructions in README.txt

ECHO **Building AO Oasis Challenge Code**

docker run -v %CD%/:c:/Users/ContainerUser/oasis_AO -it oasissubmission_ao ./compileBaseCode.bat

XCOPY /y ".\OASIS baseline source\x64\Release\*.exe" .