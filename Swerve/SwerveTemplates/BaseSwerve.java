package SushiFrcLib.Swerve.SwerveTemplates;

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
    private final double maxAngularVelocity;

    private final boolean tuningMode;

    protected Pose2d oldPose;
    protected double oldTimeStamp;

    public BaseSwerve(SwerveModule[] swerveMods, Gyro gyro) {
        this.tuningMode = swerveMods[0].swerveModuleConstants.swerveTuningMode;

        this.gyro = gyro;
        gyro.zeroGyro();

        this.swerveMods = swerveMods;

        this.field = new Field2d();

        this.maxSpeed = swerveMods[0].swerveModuleConstants.moduleInfo.maxSpeed;
        this.maxAngularVelocity = swerveMods[0].swerveModuleConstants.moduleInfo.maxAngularVelocity;

        SmartDashboard.putData("Field", field);
    }

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
                SmartDashboard.putString("Swerve Module State " + i.swerveModuleConstants.moduleNumber, states[i.swerveModuleConstants.moduleNumber].speedMetersPerSecond + ", " + states[i.swerveModuleConstants.moduleNumber].angle.getDegrees());
            }
            i.setDesiredState(states[i.swerveModuleConstants.moduleNumber]);
        }
    }

    // Vector is in mps, and rot is in radians per sec
    public void drive(Translation2d vector, double rot) {
        vector = vector.times(maxSpeed);
        rot *= maxAngularVelocity;

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
            ret[i.swerveModuleConstants.moduleNumber] = i.getPose();
        } 

        return ret;
    }

    public SwerveModuleState[] getState() {
        SwerveModuleState[] ret = new SwerveModuleState[]{null, null, null, null};

        for (SwerveModule i : swerveMods) {
            ret[i.swerveModuleConstants.moduleNumber] = i.getState();
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

    protected void setOldPose(Pose2d newPose) {
        oldPose = newPose;
        oldTimeStamp = System.currentTimeMillis();   
    }

    public void periodic() { 
        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getAngle().getDegrees(), 0, 360));

        if (tuningMode) {
            SmartDashboard.putNumber("Robot Angle Velo", getAngleVelo());
            SmartDashboard.putNumber("Robot Velo", getDriveVelo());
        }

        setOldPose(getOdomPose());

        field.setRobotPose(getOdomPose());

        for (SwerveModule i : swerveMods) {
            if (tuningMode) {
                SmartDashboard.putNumber("Swerve Module Angle " + i.swerveModuleConstants.moduleNumber, i.getAbsoluteAngleDegrees());
            }
            i.log();
        }

        oldPose = getOdomPose();
        oldTimeStamp = System.currentTimeMillis();
    }

    abstract public ChassisSpeeds getChassisSpeeds();
}
