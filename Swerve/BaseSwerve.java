package SushiFrcLib.Swerve;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.Swerve.SwerveModules.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

abstract public class BaseSwerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final Gyro gyro;

    private final Field2d field;

    private final double maxSpeed;

    private final boolean tuningMode;

    private Pose2d oldPose;
    private double oldTimeStamp;

    public BaseSwerve(SwerveModule[] swerveMods, Gyro gyro, double maxSpeed,  boolean tuningMode) {
        this.tuningMode = tuningMode;

        this.gyro = gyro;
        gyro.zeroGyro();

        this.swerveMods = swerveMods;

        this.field = new Field2d();

        this.maxSpeed = maxSpeed;

        SmartDashboard.putData("Field", field);
    }

    public BaseSwerve(SwerveModule[] swerveMods, Gyro gyro, double maxSpeed) { this(swerveMods, gyro, maxSpeed, false); }

    public void drive(ChassisSpeeds chassisSpeeds) {
        driveRobotOriented(
            new Translation2d(
                chassisSpeeds.vxMetersPerSecond,
                -chassisSpeeds.vyMetersPerSecond // TODO: FIX SHITY NEGATION 
            ),
            chassisSpeeds.omegaRadiansPerSecond
        );
    }

    abstract void driveRobotOriented(Translation2d vector, double rot);

    public void driveRobotOriented(SwerveModuleState[] states) {
        // TODO: FIX SHITY CODE https://github.com/frc1678/C2023-Public/blob/main/src/main/java/com/team1678/lib/swerve/SwerveDriveKinematics.java
        for (SwerveModuleState i : states) {
            if (i.speedMetersPerSecond > maxSpeed) {
                i.speedMetersPerSecond = maxSpeed;
            }
        }

        for (SwerveModule i : swerveMods) {
            if (tuningMode) {
                SmartDashboard.putString("Swerve Module State " + i.moduleNumber, states[i.moduleNumber].speedMetersPerSecond + ", " + states[i.moduleNumber].angle.getDegrees());
            }
            i.setDesiredState(states[i.moduleNumber]);
        }
    }

    // Vector is in mps, and rot is in radians per sec
    public void drive(Translation2d vector, double rot) {
        if (tuningMode) {
            SmartDashboard.putString("Input: ", vector.getX() + ", " + vector.getY() + ", " + rot);
        }

        vector = vector.rotateBy(gyro.getAngle().plus(Rotation2d.fromRadians(getDriveVelo() * 0.0)));

        if (tuningMode) {
            SmartDashboard.putString("Input Post Rotate : ", vector.getX() + ", " + vector.getY() + ", " + rot);
        }

        driveRobotOriented(vector, rot);
    }

    public void updateEncoders() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModulePosition[] getPose() {
        SwerveModulePosition[] ret = new SwerveModulePosition[]{null, null, null, null};

        for (SwerveModule i : swerveMods) {
            ret[i.moduleNumber] = i.getPose();
        } 

        return ret;
    }

    protected void setGyro(Pose2d pose) {
        gyro.setAngle(pose.getRotation());
    }

    abstract public Pose2d getOdomPose();

    public Gyro getGyro() { return gyro; }

    public void resetGyro() { gyro.zeroGyro(); }

    public Command resetGyroCommand() {
        return runOnce(() -> {
            resetGyro();
        });
    }

    public double getAngleVelo() {
        return 1000 * (getOdomPose().getRotation().getRadians() - oldPose.getRotation().getRadians()) / (System.currentTimeMillis() - oldTimeStamp); // in radians per milisecond
    }

    public double getDriveVelo() {
        return 1000 * (getOdomPose().getTranslation().getNorm() - oldPose.getTranslation().getNorm()) / (System.currentTimeMillis() - oldTimeStamp); // in meters per milisecond 
    }

    public void periodic() { 
        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getAngle().getDegrees(), 0, 360));

        if (tuningMode) {
            SmartDashboard.putNumber("Robot Angle Velo", getAngleVelo());
            SmartDashboard.putNumber("Robot Velo", getDriveVelo());
        }

        oldPose = getOdomPose();
        oldTimeStamp = System.currentTimeMillis();

        field.setRobotPose(getOdomPose());

        for (SwerveModule i : swerveMods) {
            if (tuningMode) {
                SmartDashboard.putNumber("Swerve Module Angle " + i.moduleNumber, i.getAngle());
            }
            i.log();
        }

        oldPose = getOdomPose();
        oldTimeStamp = System.currentTimeMillis();
    }
}
