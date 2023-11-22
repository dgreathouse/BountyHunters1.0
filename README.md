# BountyHunters Team FRC 8517
## Overview
This code is a remake of the CTRE swerve module code. The code has been integrated into the command based programming framework.

## Robot 
The robot used for this code is our typical 3 wheel swerve drive using Swerve Xs from WCProducts.net. All motors are Falcon FXs with CANCoders.
A CANivore is used with a PRO license to get FOC out of the motors.

## Change for your use
To use this code you must understand the code and adjust all changes for your system. 
Changes needed that I understand at this point are:
- Add another motor for typical 4 wheel swerve. We are the only foolish team to do 3 wheel swerve.
- Change your drive base length and width.
- Tune your PID variables.
- Change all your CANIDs.
- Change gear ratios
- Debug what I have not tested yet...

## Future
Upcoming changes that are planned.
- Debug after robot is built. Expect bugs at this point.
- Integrate 2024 robot subsystems into codebase.

## Autonomous
- Our basic drive strategy is to drive at a velocity in Angle based field centric mode. This allows us to drive in any angle and have the robot PID to an angle. This prevents us from using a PID loop to drive to distance. Since the drive is in velociy mode the time we use is actual distance.

## Pull/Push Requests
Please add a Pull Request for changes that you would like to see
If you have code fixes or addition you want in, please make a push request with the changes.
More people working on code makes it better if we coordinate and agree on changes/additions.
There are lots of ways to solve a problem when coding, so please understand if changes are not accepted.
