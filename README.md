# Wolfpack Machina Inspired Drive Code
This is a mini project programmed by Letian Li, based off of Wolf Machina's Movement Breakdown [video](https://www.youtube.com/watch?v=ri06orPFaKo) for the Freight Frenzy season.

This project optimizes for max power movement and smooths movement by using the curvature of the last 3 recorded positions.

# Roadrunner based

This project uses roadrunner's localization and contains an incomplete copy of their [quickstart](https://github.com/acmerobotics/road-runner-quickstart/tree/master).

If you plan to use the rest of roadrunner for autonomous, then copy just the TeamCode/java/.../WolfDrive folder into your existing roadrunner based repo to test, otherwise proceed to roadrunner tuning.

## Roadrunner Tuning process (Not recommended: only for running as a standalone project)
1. Setup MecanumDrive with your motor names and reverse motors as necessary. Set the IMU orientation here as well.
2. Choose a localizer (wheel encoders by default): DriveLocalizer, TwoDeadWheelLocalizer, or ThreeDeadWheelLocalizer
3. Setup the respective localizer class with your odometry encoder names (if using dead wheel)
4. Run MecanumDirectionDebugger and/or DeadWheelDirectionDebugger to ensure everything is reversed correctly
5. Run ForwardPushTest and record "inPerTick" in MecanumDrive
6. Run LateralPushTest (if not using dead wheel) and record "lateralInPerTick" in MecanumDrive
7. For more details see the [docs](https://rr.brott.dev/docs/v1-0/tuning/) and only the following sections: [Hardware Setup](https://rr.brott.dev/docs/v1-0/tuning/#drive-classes), [ForwardPushTest](https://rr.brott.dev/docs/v1-0/tuning/#forwardpushtest), and [LateralPushTest](https://rr.brott.dev/docs/v1-0/tuning/#lateralpushtest-mecanum--drive-encoders-only)

# WolfDrive Tuning process

### 1. Collecting max velocities (or movement bias)

Make sure you have enough space in the drive direction as the robot will move in said direction at full power for a given amount of time.

1. Run MaxVelStraightTest to get the robot's max velocity forwards. Record this as maxVelocityX in WolfDrive.java.
2. Run MaxVelStrafeTest to get the robot's max velocity strafing. Record this as maxVelocityY (positive) in WolfDrive.java.
    - Make sure the robot strafes in the correct direction.
    - Also recommended to strafe and record both left and right.

### 2. Test and tune centripetal correction

1. Run WolfDriveTest and adjust the centripetalWeighting constant in WolfDrive.java
    - This number should be extremely small.

### 3. Use the WolfDrive class in your code
1. Copy the Standalone WolfDrive folder into your code
2. Instantiate the class
3. Remember to updatePoseEstimate, feed the pose into trackPosition(), and then driveWithCorrection(). See WolfDriveTest for example.
