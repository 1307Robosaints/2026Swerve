package frc.robot.codebases.controllers;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.GenericHID;

public interface Controller {

    // Axes
    double getLeftX();
    double getLeftY();
    double getRightX();
    double getRightY();
    double getL2Axis();
    double getR2Axis();

    // Square
    boolean getSquareButton();
    boolean getSquareButtonPressed();
    boolean getSquareButtonReleased();
    BooleanEvent square(EventLoop loop);

    // Cross
    boolean getCrossButton();
    boolean getCrossButtonPressed();
    boolean getCrossButtonReleased();
    BooleanEvent cross(EventLoop loop);

    // Circle
    boolean getCircleButton();
    boolean getCircleButtonPressed();
    boolean getCircleButtonReleased();
    BooleanEvent circle(EventLoop loop);

    // Triangle
    boolean getTriangleButton();
    boolean getTriangleButtonPressed();
    boolean getTriangleButtonReleased();
    BooleanEvent triangle(EventLoop loop);

    // L1
    boolean getL1Button();
    boolean getL1ButtonPressed();
    boolean getL1ButtonReleased();
    BooleanEvent L1(EventLoop loop);

    // R1
    boolean getR1Button();
    boolean getR1ButtonPressed();
    boolean getR1ButtonReleased();
    BooleanEvent R1(EventLoop loop);

    // Share
    boolean getShareButton();
    boolean getShareButtonPressed();
    boolean getShareButtonReleased();
    BooleanEvent share(EventLoop loop);

    // Options
    boolean getOptionsButton();
    boolean getOptionsButtonPressed();
    boolean getOptionsButtonReleased();
    BooleanEvent options(EventLoop loop);

    // L3
    boolean getL3Button();
    boolean getL3ButtonPressed();
    boolean getL3ButtonReleased();
    BooleanEvent L3(EventLoop loop);

    // R3
    boolean getR3Button();
    boolean getR3ButtonPressed();
    boolean getR3ButtonReleased();
    BooleanEvent R3(EventLoop loop);

} 