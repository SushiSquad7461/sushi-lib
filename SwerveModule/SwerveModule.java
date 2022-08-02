// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package SushiFrcLib.SwerveModule;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import SushiFrcLib.Constants.SushiConstants;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Math.Vector2;
import SushiFrcLib.Motor.SwerveModuleConfig;

public abstract class SwerveModule extends SubsystemBase {
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

  @Override
  public void periodic() {
    turn(angle, turnInversion);
    drive(driveVelocity);
  }

  abstract protected void drive(double velocity);

  abstract protected void turn(double angle, int inversion);

  @Override
  public void simulationPeriodic() { }
}