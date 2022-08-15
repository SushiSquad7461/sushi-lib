// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package SushiFrcLib.SwerveModule;

import com.ctre.phoenix.sensors.CANCoder;

import SushiFrcLib.Constants.SushiConstants;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Math.Rotation2;
import SushiFrcLib.Math.Vector2;
import SushiFrcLib.Motor.SwerveModuleConfig;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
  private final CANCoder canCoder;
  private final double angleOffest;

  private double angle;
  private int turnInversion = 1;
  private int driveInversion = 1;
  private double errorBound = 0;
  private double turnSpeed = 0;
  private boolean configMode;
  private double driveVelocity = 0;

  public SwerveModule(SwerveModuleConfig config) {
    canCoder = new CANCoder(config.encoderId);
    this.angle = getModuleAngle();
    this.angleOffest = config.angleOfset;
    this.configMode = false;
  }

  public SwerveModule(SwerveModuleConfig config, double errorBound, double turnSpeed) {
    this.canCoder = new CANCoder(config.encoderId);
    this.angle = getModuleAngle();
    this.angleOffest = config.angleOfset;
    this.errorBound = errorBound;
    this.turnSpeed = turnSpeed;
    this.configMode = true;
  }

  // in degrees
  public double getModuleAngle() {
    return Conversion.normalizeAngle(canCoder.getPosition() - angleOffest);
  }

  private double getErrorBound() {
      return configMode ? SushiConstants.SWERVE_MODULE.ERROR_BOUND : errorBound;
  }

  // Method uselss now, was used when turn motor was set to set speed until reached postion
  // however now absoulte since switched to PID
  private double getTurnSpeed() {
    return configMode ? SushiConstants.SWERVE_MODULE.TURN_SPEED : turnSpeed;
  }

  public void updateModule(Vector2 newPos) {
    if (Math.abs(newPos.getAngle().toDegrees() - getModuleAngle()) >= getErrorBound()) {
        angle = newPos.getAngle().toDegrees();

        // Optimization 1
        if (Math.abs(angle - getModuleAngle()) > 180) {
            turnInversion = -1;
        } else {
            turnInversion = 1;
        }

        // Optimiazation 2
        if (Math.abs(angle - getModuleAngle()) > 90) {
            if (angle > getModuleAngle()) {
                angle = getModuleAngle() - (Math.abs(angle - getModuleAngle()) - 90);
            } else {
                angle = getModuleAngle() + (Math.abs(angle - getModuleAngle()) - 90);
            }
            driveInversion = -1;
        } else {
            driveInversion = 1;
        }

    }
    driveVelocity = newPos.length * driveInversion;
  }

  public void updateModule(SwerveModuleState state) {
    updateModule(Vector2.fromAngle(Rotation2.fromRadians((state.angle).getRadians()), state.speedMetersPerSecond));
  }

  public void start() {}

  public void stop() { 
    setTurn(0);
    setDrive(0);
  }

  public void periodic() {
    turn(angle, turnInversion);
    drive(driveVelocity);
  }

  abstract protected void drive(double velocity);

  abstract protected void turn(double angle, int inversion);

  abstract protected void setTurn(double speed);
  abstract protected void setDrive(double speed);
}