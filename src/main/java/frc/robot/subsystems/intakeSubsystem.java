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





public class IntakeSubsystem extends SubsystemBase {
  

  private final SparkMax m_intakeSpark;

  
  /** Creates a new folderSubsystem. */
  public IntakeSubsystem() {
    
      //intake:

    m_intakeSpark = new SparkMax(13, SparkMax.MotorType.kBrushless);
    m_intakeSpark.configure(Configs.intakeSparkMax.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {

    
   
  }

  

  //intake:

  public void stopIntake() {
    m_intakeSpark.set(0);
  }

  public void setIntakeSpeed(double speed) {
    m_intakeSpark.set(speed);
  }

}

 
