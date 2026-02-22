package frc.robot.codebases.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.GenericHID;

public class XboxControllerWrapper extends XboxController implements Controller {


    public XboxControllerWrapper(int port) {
        super(port);
        
    }

    // ---------------- Axes ----------------

    @Override
    public double getLeftX() {
        return super.getLeftX();
    }

    @Override
    public double getLeftY() {
        return super.getLeftY();
    }

    @Override
    public double getRightX() {
        return super.getRightX();
    }

    @Override
    public double getRightY() {
        return super.getRightY();
    }

    @Override
    public double getL2Axis() {
        return super.getLeftTriggerAxis();
    }

    @Override
    public double getR2Axis() {
        return super.getRightTriggerAxis();
    }

    // ---------------- Square (→ X) ----------------

    @Override
    public boolean getSquareButton() {
        return super.getXButton();
    }

    @Override
    public boolean getSquareButtonPressed() {
        return super.getXButtonPressed();
    }

    @Override
    public boolean getSquareButtonReleased() {
        return super.getXButtonReleased();
    }

    @Override
    public BooleanEvent square(EventLoop loop) {
        return super.x(loop);
    }

    // ---------------- Cross (→ A) ----------------

    @Override
    public boolean getCrossButton() {
        return super.getAButton();
    }

    @Override
    public boolean getCrossButtonPressed() {
        return super.getAButtonPressed();
    }

    @Override
    public boolean getCrossButtonReleased() {
        return super.getAButtonReleased();
    }

    @Override
    public BooleanEvent cross(EventLoop loop) {
        return super.a(loop);
    }

    // ---------------- Circle (→ B) ----------------

    @Override
    public boolean getCircleButton() {
        return super.getBButton();
    }

    @Override
    public boolean getCircleButtonPressed() {
        return super.getBButtonPressed();
    }

    @Override
    public boolean getCircleButtonReleased() {
        return super.getBButtonReleased();
    }

    @Override
    public BooleanEvent circle(EventLoop loop) {
        return super.b(loop);
    }

    // ---------------- Triangle (→ Y) ----------------

    @Override
    public boolean getTriangleButton() {
        return super.getYButton();
    }

    @Override
    public boolean getTriangleButtonPressed() {
        return super.getYButtonPressed();
    }

    @Override
    public boolean getTriangleButtonReleased() {
        return super.getYButtonReleased();
    }

    @Override
    public BooleanEvent triangle(EventLoop loop) {
        return super.y(loop);
    }

    // ---------------- L1 (→ LB) ----------------

    @Override
    public boolean getL1Button() {
        return super.getLeftBumperButton();
    }

    @Override
    public boolean getL1ButtonPressed() {
        return super.getLeftBumperButtonPressed();
    }

    @Override
    public boolean getL1ButtonReleased() {
        return super.getLeftBumperButtonReleased();
    }

    @Override
    public BooleanEvent L1(EventLoop loop) {
        return super.leftBumper(loop);
    }

    // ---------------- R1 (→ RB) ----------------

    @Override
    public boolean getR1Button() {
        return super.getRightBumperButton();
    }

    @Override
    public boolean getR1ButtonPressed() {
        return super.getRightBumperButtonPressed();
    }

    @Override
    public boolean getR1ButtonReleased() {
        return super.getRightBumperButtonReleased();
    }

    @Override
    public BooleanEvent R1(EventLoop loop) {
        return super.rightBumper(loop);
    }

    // ---------------- Share (→ Back) ----------------

    @Override
    public boolean getShareButton() {
        return super.getBackButton();
    }

    @Override
    public boolean getShareButtonPressed() {
        return super.getBackButtonPressed();
    }

    @Override
    public boolean getShareButtonReleased() {
        return super.getBackButtonReleased();
    }

    @Override
    public BooleanEvent share(EventLoop loop) {
        return super.back(loop);
    }

    // ---------------- Options (→ Start) ----------------

    @Override
    public boolean getOptionsButton() {
        return super.getStartButton();
    }

    @Override
    public boolean getOptionsButtonPressed() {
        return super.getStartButtonPressed();
    }

    @Override
    public boolean getOptionsButtonReleased() {
        return super.getStartButtonReleased();
    }

    @Override
    public BooleanEvent options(EventLoop loop) {
        return super.start(loop);
    }

    // ---------------- L3 ----------------

    @Override
    public boolean getL3Button() {
        return super.getLeftStickButton();
    }

    @Override
    public boolean getL3ButtonPressed() {
        return super.getLeftStickButtonPressed();
    }

    @Override
    public boolean getL3ButtonReleased() {
        return super.getLeftStickButtonReleased();
    }

    @Override
    public BooleanEvent L3(EventLoop loop) {
        return super.leftStick(loop);
    }

    // ---------------- R3 ----------------

    @Override
    public boolean getR3Button() {
        return super.getRightStickButton();
    }

    @Override
    public boolean getR3ButtonPressed() {
        return super.getRightStickButtonPressed();
    }

    @Override
    public boolean getR3ButtonReleased() {
        return super.getRightStickButtonReleased();
    }

    @Override
    public BooleanEvent R3(EventLoop loop) {
        return super.rightStick(loop);
    }
}