// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.IntPredicate;
import frc.robot.codebases.SonarSensor;

import org.littletonrobotics.junction.Logger;
import frc.robot.limelightlib.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;


public class LimelightSubsystem extends SubsystemBase {
 
  // --------------------
  // Adjustable variables
  // --------------------
  public int redTagId = 10;   // AprilTag ID for red
  public int blueTagId = 26;  // AprilTag ID for blue
  public double txSetpoint = 0.0;  // desired horizontal offset (degrees)
  public double tySetpoint = 0.0;  // desired vertical offset (degrees)
  public double taSetpoint = 1.0;  // desired target area (or 0 if ignoring)
  public double yawSetpoint = 0.0; // desired yaw (degrees)

  private String tableName = "limelight";

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    LimelightHelpers.setCameraPose_RobotSpace("limelight", .325, .14, 0, 0, 0, 0);

  }

  @Override
  public void periodic() {
  
   
  }

 /** Returns the NetworkTables "tx" value (horizontal offset) */
    public double getTx() {
        return NetworkTableInstance.getDefault()
                .getTable(tableName)
                .getEntry("tx").getDouble(0);
    }

    /** Returns the NetworkTables "ty" value (vertical offset) */
    public double getTy() {
        return NetworkTableInstance.getDefault()
                .getTable(tableName)
                .getEntry("ty").getDouble(0);
    }

    /** Returns the NetworkTables "ta" value (target area) */
    public double getTa() {
        return NetworkTableInstance.getDefault()
                .getTable(tableName)
                .getEntry("ta").getDouble(0);
    }

    /** Returns the NetworkTables "yaw" value from your Limelight lib */
    public double getYaw() {
        return NetworkTableInstance.getDefault()
                .getTable(tableName)
                .getEntry("yaw").getDouble(0);
    }

    /** Returns the current detected AprilTag ID */
    public int getTargetId() {
        return (int) NetworkTableInstance.getDefault()
                .getTable(tableName)
                .getEntry("tid").getDouble(-1);
    }

    /** Returns the tag ID for the current alliance (red/blue) */
    public int getTargetIdForAlliance(boolean red) {
        return red ? redTagId : blueTagId;
    }

    /** Check if we see the desired target for our alliance */
    public boolean hasTarget(boolean redAlliance) {
        return getTargetId() == getTargetIdForAlliance(redAlliance);
    }

}

 
