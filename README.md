# Oasis Submission - Applied Optimization, Inc.

## Cloning and building code
This project can be cloned using the following command:  
`git clone --recurse-submodules http://gitlab/ME-Team/america-makes-oasis-challenge.git`

If the program is downloaded from GitHub manually, please run  
`git submodule update --init --recursive` from the base folder to pull the [stl_reader](https://github.com/sreiter/stl_reader) dependency.

An additional runtime dependency of this program inherited from the Oasis codebase is [Slic3r](https://dl.slic3r.org/win/).

This project uses MSBuild tools to build the executables, which can be done automatically using the _build.bat_ file in the base folder. In order to run this build in a Docker container, please set Docker to use Windows containers, then use the following commands:
```cmd
docker build . --tag oasissubmission_ao
.\buildOasisSubmission.bat
```
The Docker container uses shared volumes to share files between the container and the host system. Once the docker build is successful, the executables will be placed in the base directory of this project.
## Running route generation
Route generation can be run through docker using two of the scripts in this directory (_run_Build_Plate.bat_ and _run_NIST.bat_). Calling these scripts from the host system will run the previously built Docker container with the appropriate XLS configuration file. This utilizes a change to the Oasis baseline code which bypasses the interactive user selection process when the first input argument to createScanpaths is a configuration file.

The outputs are saved into subfolders in the current host folder as specified in the configuration files.
## Dependencies and License files
The software in this repository uses the Oasis baseline code as the foundation for the updated route generation algorithms. The newly developed functionality is primarily contained in _Oasis baseline source/AO_source/_ along with the license file for these new changes. There are some additional changes in the Oasis codebase to enable these functions which can be identified through the git history.

The only new dependency for this code is [stl_reader](https://github.com/sreiter/stl_reader).
