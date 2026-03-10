// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import frc.robot.Configs;
import frc.robot.Constants;





public class climberSubsystem extends SubsystemBase {
  
  private final SparkMax m_climberSpark;
  
  /** Creates a new climberSubsystem. */
  public climberSubsystem() {

    m_climberSpark = new SparkMax(11, SparkMax.MotorType.kBrushless); //is it brushless or brushed? check the wiring and the motor
    m_climberSpark.configure(Configs.climberSparkMax.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
   
  }

  public void setclimberSpeed(double speed) {
    m_climberSpark.set(speed);
  }
  public void stopClimber() {
    m_climberSpark.set(0);
  }

}

 
