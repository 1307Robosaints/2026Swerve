package frc.robot.codebases.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.codebases.controllers.Controller;

public class PS4ControllerWrapper extends PS4Controller implements Controller {


    public PS4ControllerWrapper(int port) {
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
        return super.getL2Axis();
    }

    @Override
    public double getR2Axis() {
        return super.getR2Axis();
    }

    // ---------------- Square ----------------

    @Override
    public boolean getSquareButton() {
        return super.getSquareButton();
    }

    @Override
    public boolean getSquareButtonPressed() {
        return super.getSquareButtonPressed();
    }

    @Override
    public boolean getSquareButtonReleased() {
        return super.getSquareButtonReleased();
    }

    @Override
    public BooleanEvent square(EventLoop loop) {
        return super.square(loop);
    }

    // ---------------- Cross ----------------

    @Override
    public boolean getCrossButton() {
        return super.getCrossButton();
    }

    @Override
    public boolean getCrossButtonPressed() {
        return super.getCrossButtonPressed();
    }

    @Override
    public boolean getCrossButtonReleased() {
        return super.getCrossButtonReleased();
    }

    @Override
    public BooleanEvent cross(EventLoop loop) {
        return super.cross(loop);
    }

    // ---------------- Circle ----------------

    @Override
    public boolean getCircleButton() {
        return super.getCircleButton();
    }

    @Override
    public boolean getCircleButtonPressed() {
        return super.getCircleButtonPressed();
    }

    @Override
    public boolean getCircleButtonReleased() {
        return super.getCircleButtonReleased();
    }

    @Override
    public BooleanEvent circle(EventLoop loop) {
        return super.circle(loop);
    }

    // ---------------- Triangle ----------------

    @Override
    public boolean getTriangleButton() {
        return super.getTriangleButton();
    }

    @Override
    public boolean getTriangleButtonPressed() {
        return super.getTriangleButtonPressed();
    }

    @Override
    public boolean getTriangleButtonReleased() {
        return super.getTriangleButtonReleased();
    }

    @Override
    public BooleanEvent triangle(EventLoop loop) {
        return super.triangle(loop);
    }

    // ---------------- L1 ----------------

    @Override
    public boolean getL1Button() {
        return super.getL1Button();
    }

    @Override
    public boolean getL1ButtonPressed() {
        return super.getL1ButtonPressed();
    }

    @Override
    public boolean getL1ButtonReleased() {
        return super.getL1ButtonReleased();
    }

    @Override
    public BooleanEvent L1(EventLoop loop) {
        return super.L1(loop);
    }

    // ---------------- R1 ----------------

    @Override
    public boolean getR1Button() {
        return super.getR1Button();
    }

    @Override
    public boolean getR1ButtonPressed() {
        return super.getR1ButtonPressed();
    }

    @Override
    public boolean getR1ButtonReleased() {
        return super.getR1ButtonReleased();
    }

    @Override
    public BooleanEvent R1(EventLoop loop) {
        return super.R1(loop);
    }

    // ---------------- Share ----------------

    @Override
    public boolean getShareButton() {
        return super.getShareButton();
    }

    @Override
    public boolean getShareButtonPressed() {
        return super.getShareButtonPressed();
    }

    @Override
    public boolean getShareButtonReleased() {
        return super.getShareButtonReleased();
    }

    @Override
    public BooleanEvent share(EventLoop loop) {
        return super.share(loop);
    }

    // ---------------- Options ----------------

    @Override
    public boolean getOptionsButton() {
        return super.getOptionsButton();
    }

    @Override
    public boolean getOptionsButtonPressed() {
        return super.getOptionsButtonPressed();
    }

    @Override
    public boolean getOptionsButtonReleased() {
        return super.getOptionsButtonReleased();
    }

    @Override
    public BooleanEvent options(EventLoop loop) {
        return super.options(loop);
    }

    // ---------------- L3 ----------------

    @Override
    public boolean getL3Button() {
        return super.getL3Button();
    }

    @Override
    public boolean getL3ButtonPressed() {
        return super.getL3ButtonPressed();
    }

    @Override
    public boolean getL3ButtonReleased() {
        return super.getL3ButtonReleased();
    }

    @Override
    public BooleanEvent L3(EventLoop loop) {
        return super.L3(loop);
    }

    // ---------------- R3 ----------------

    @Override
    public boolean getR3Button() {
        return super.getR3Button();
    }

    @Override
    public boolean getR3ButtonPressed() {
        return super.getR3ButtonPressed();
    }

    @Override
    public boolean getR3ButtonReleased() {
        return super.getR3ButtonReleased();
    }

    @Override
    public BooleanEvent R3(EventLoop loop) {
        return super.R3(loop);
    }
}