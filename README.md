# READY RAL2 Sample Driver

This repository contains a sample ral2-driver implementation for ForgeOS and meta-data including its configuration,
visualization, and device package details.

## Introduction

The code in this repository is meant to serve only as a basis for implementing a ForgeOS robot driver functionality. The
ready::SampleDriver class implementation stubs all of functions and emits all of the Qt signals necessary to get the
robot driver implementation to compile, install, configure, and run in ForgeOS. Using this implementation permits
relatively quick iteration of the robot driver using ForgeOS applications to incrementally verify the implemented
behaviors.

## Key Implementation Files

The following files and directories will be high-touch areas when implementing a RAL2 robot driver instance against a
particular make and model of robot arm and controller:

* CMakeLists.txt
* src/SampleDriver.cpp
* src/SampleDriver.hpp
* configuration/Sample.json
* configurations/robot-models/*.json
* configurations/controller-models/*/*.json
* manifest.json

### CMakeLists.txt

This file will need to be updated when adding/removing/renaming source files, updating configurations, adding/removing
robot models or controller models, adding USB transfer files, adding dependent libraries, and more.

It should be updated to create a unique driver library name by changing the `add_library` CMake directive. A
corresponding change will need to be made to the "robot-driver-library" attribute found in configuration/Sample.json.

### SampleDriver.cpp, SampleDriver.hpp

These source files contain the stubbed ral2::Driver interface implementation in the class `ready::SampleDriver`. These
files should be renamed to match the vendor driver name and should update namespace names to match the vendor.
Additional files can be added to the `src` tree to contain other implementation details as needed.

Changing the names of these files, namespaces, and classes will require changes in the CMakeLists.txt file and the
DriverLib.cpp file.

### configuration/Sample.json

This source file contains high-level configuration of the robot driver. It contains ral2-service attributes, robot model configuration, robot controller configuration, faults, warnings, and supported robot modes.

This file will need to be updated when adding or removing robot models and robot controllers, updating fault messages,
changing jogging behavior, and modifying high-level ral2-service behaviors. The default values are a reasonable starting
point.

### configurations/robot-models/*.json

Each of the .json files in the robot-models directory corresponds to settings for a specific robot model. Adding and
removing these files will require changes to CMakeLists.txt and the following attributes in "configuration/Sample.json":

* controller.*.supported-robot-models
* attribute-list - robot attributes - model values

### configurations/controller-models/*/*.json

Each of the .json files in the controller-models directory tree corresponds to a specific operating mode for a
controller. Adding and removing these files/directories will require changes to CMakeLists.txt and the following
attributes in the "configuration/Sample.json":

* controller.*.supported-robot-models
* controller.*.safety-configuration
* attribute-list - controller attributes - model values
* attribute-list - controller attributes - name ARRAY

### Manifest.json

The manifest.json file is used to describe the ForgeOS device package. The following fields will need to be updated:
* "guid" -- This field is a globally unique string identifying the package in ForgeOS. The value is typically of the form "ready-ral-<robot-vendor>-driver"
* "name" -- This field is the name of the device package.
* "vendor" -- This is the vendor of the package. For robot drivers this typically maps to the robot vendor name.
* "vendor-icon" -- This is the local path of the vendor logo .svg file.
* "description" -- A brief text description of the package.
* "version" -- The semantic version of the driver library implementation. This typically needs to be updated when there
               are changes to other configuration files, additions and modifications of translations, or when runtime
               files of the driver have changed.
* "deviceConfigurations" -- If the `configurations/Sample.json` file is renamed to match the actual robot vendor, then
                            this field will need to be updated to match the renamed file.
