package SushiFrcLib.Swerve.SwerveTemplates;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.Swerve.SwerveModules.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

abstract public class BaseSwerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final Gyro gyro;

    protected final Field2d field;

    private final double maxSpeed;
    private final double maxAngularVelocity;

    private final boolean tuningMode;

    protected Pose2d prevPose;
    protected double prevPoseTimeStamp;

    protected PIDTuning angleTuning;
    protected PIDTuning driveTuning;

    public BaseSwerve(SwerveModule[] swerveMods, Gyro gyro) {
        this.tuningMode = swerveMods[0].swerveModuleConstants.swerveTuningMode;

        this.gyro = gyro;
        gyro.zeroGyro();

        this.swerveMods = swerveMods;

        this.field = new Field2d();

        this.maxSpeed = swerveMods[0].swerveModuleConstants.moduleInfo.maxSpeed;
        this.maxAngularVelocity = swerveMods[0].swerveModuleConstants.moduleInfo.maxAngularVelocity;

        angleTuning = new PIDTuning("Swerve Angle", swerveMods[0].swerveModuleConstants.angleConfig.pid,  swerveMods[0].swerveModuleConstants.swerveTuningMode);
        driveTuning = new PIDTuning("Swerve Drive",  swerveMods[0].swerveModuleConstants.driveConfig.pid,  swerveMods[0].swerveModuleConstants.swerveTuningMode);

        SmartDashboard.putData("Field", field);
    }

    public void driveChassis(ChassisSpeeds chassisSpeeds) {
        driveRobotOriented(
                new Translation2d(
                        chassisSpeeds.vxMetersPerSecond,
                        chassisSpeeds.vyMetersPerSecond
                ),
                chassisSpeeds.omegaRadiansPerSecond);
    }

    public abstract void driveRobotOriented(Translation2d vector, double rot);

    public void driveRobotOriented(SwerveModuleState[] states) {
        // TODO: FIX SHITY CODE
        // https://github.com/frc1678/C2023-Public/blob/main/src/main/java/com/team1678/lib/swerve/SwerveDriveKinematics.java
        for (SwerveModuleState i : states) {
            if (i.speedMetersPerSecond > maxSpeed) {
                i.speedMetersPerSecond = maxSpeed;
            }
        }

        for (SwerveModule i : swerveMods) {
            if (tuningMode) {
                SmartDashboard.putString("Swerve Module State " + i.swerveModuleConstants.moduleNumber,
                        states[i.swerveModuleConstants.moduleNumber].speedMetersPerSecond + ", "
                                + states[i.swerveModuleConstants.moduleNumber].angle.getDegrees());
            }
            i.setDesiredState(states[i.swerveModuleConstants.moduleNumber]);
        }
    }

    public Command zeroGyro() {
        return runOnce(() -> gyro.zeroGyro());
    }

    // Vector is in mps, and rot is in radians per sec
    public void drive(Translation2d vector, double rot, Alliance color) {
        vector = vector.times(maxSpeed);
        rot *= maxAngularVelocity;

        vector = vector.rotateBy(
                        //Rotation2d.fromRadians(getDriveVelo() * 0.0).plus 
                        Rotation2d.fromDegrees(color == Alliance.Red ? 180 : 0).minus(gyro.getAngle())); // We are adding a value for
                                                                                           // latency conpensation,
                                                                                           // currently untuned

        driveRobotOriented(vector, rot);
    }

    public void updateEncoders() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModulePosition[] getPose() {
        SwerveModulePosition[] ret = new SwerveModulePosition[] { null, null, null, null };

        for (SwerveModule i : swerveMods) {
            ret[i.swerveModuleConstants.moduleNumber] = i.getPose();
        }

        return ret;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] ret = new SwerveModuleState[] { null, null, null, null };

        for (SwerveModule i : swerveMods) {
            ret[i.swerveModuleConstants.moduleNumber] = i.getState();
        }

        return ret;
    }

    protected void setGyro(Pose2d pose) {
        gyro.setAngle(pose.getRotation());
    }

    abstract public Pose2d getOdomPose();

    public Gyro getGyro() {
        return gyro;
    }

    public void resetGyro() {
        gyro.zeroGyro();
    }

    public Command resetGyroCommand() {
        return runOnce(() -> {
            resetGyro();
        });
    }

    public double getAngleVelo() {
        return 1000 * (getOdomPose().getRotation().getRadians() - prevPose.getRotation().getRadians())
                / (System.currentTimeMillis() - prevPoseTimeStamp); // in radians per milisecond
    }

    public double getDriveVelo() {
        return 1000 * (getOdomPose().getTranslation().getNorm() - prevPose.getTranslation().getNorm())
                / (System.currentTimeMillis() - prevPoseTimeStamp); // in meters per milisecond
    }

    protected void setPrevPose(Pose2d newPose) {
        prevPose = newPose;
        prevPoseTimeStamp = System.currentTimeMillis();
    }

    public void periodic() {
        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getAngle().getDegrees(), 0, 360));

        if (tuningMode) {
            SmartDashboard.putNumber("Robot Angle Velo", getAngleVelo());
            SmartDashboard.putNumber("Robot Velo", getDriveVelo());
        }

        Pose2d newOdomPose = getOdomPose();

        field.setRobotPose(newOdomPose);

        for (SwerveModule i : swerveMods) {
            if (tuningMode) {
                i.publishTelemetry();
                i.updatePID(angleTuning, driveTuning);
            }
            i.log();
        }

        setPrevPose(newOdomPose);
    }

    abstract public ChassisSpeeds getChassisSpeeds();
}
