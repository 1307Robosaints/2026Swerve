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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import frc.robot.Configs;
import frc.robot.Constants;





public class intakeSubsystem extends SubsystemBase {

    private final SparkMax m_intakeSpark;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;

    public intakeSubsystem() {
        m_intakeSpark = new SparkMax(12, SparkMax.MotorType.kBrushless);
        m_intakeSpark.configure(Configs.intakeSparkMax.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_intakeSpark.getEncoder();
        m_pidController = m_intakeSpark.getClosedLoopController();
    }

    //Spin the intake a fixed number of rotations
    public void rotateIntake(double rotationsWanted) {
        double targetPosition = m_encoder.getPosition() + rotationsWanted;
        m_pidController.setReference(targetPosition, SparkMax.ControlType.kPosition);
    }

    //Stop the motor
    public void stopIntake() {
        m_intakeSpark.set(0);
    }
}

 
