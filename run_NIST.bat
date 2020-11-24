@ECHO OFF

ECHO Running AO Oasis Challenge for NIST Plate

docker run -v %CD%/:c:/Users/ContainerUser/oasis_AO -it oasissubmission_ao ./createScanPaths.exe OASIS_Input_Config_NIST_Plate_AppliedOptimization.xls
