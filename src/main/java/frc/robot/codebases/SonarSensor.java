// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.codebases;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.Constants.SonarConstants;
import edu.wpi.first.wpilibj.AnalogInput;

public class SonarSensor {

  private final AnalogInput m_sonarInput;

  /** Creates a new SonarSensor. */
  public SonarSensor(int port) {
    m_sonarInput = new AnalogInput(port);
  
  }

  public double getRawVoltage() {
    return m_sonarInput.getVoltage();
  }

  public double getDistanceCentimeters() {
    double voltage = m_sonarInput.getVoltage();
    //double vcc = RobotController.getVoltage5V();
    return voltage * SonarConstants.kVoltageToCentimeters;
  }

  public double getDistanceInches() {
    return getDistanceCentimeters() / 2.54;
  }


}

 
