// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.IntPredicate;

import org.littletonrobotics.junction.Logger;
import frc.robot.limelightlib.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;


public class LimelightSubsystem extends SubsystemBase {
 
  Pose3d tagPose = new Pose3d();
  int counter = 0;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    LimelightHelpers.setCameraPose_RobotSpace("limelight", .325, .14, 0, 0, 0, 0);

  }

  @Override
  public void periodic() {
    //update tag pose every cycle
    tagPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");

    counter++;
    if (counter > 50) {
    counter = 0;
    System.out.println(
        "Tag?:"+hasTarget()
        +", X: "+getX3d()
        +", Y: "+getY3d()
        +", Yaw: "+getYaw3d()
        );
    }
   
  }

  public Pose3d getTagPose() {
      return tagPose;
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV("limelight");
  }

  public double getX3d() {
    return getTagPose().getTranslation().getX();
  }

  public double getY3d() {
    return getTagPose().getTranslation().getY();
  }

  public double getZ3d() {
    return getTagPose().getTranslation().getZ();
  }

  public double getYaw3d() {
    return Math.atan2(getY3d(), getX3d());
  }
}

 
