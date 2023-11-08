package SushiFrcLib.Swerve;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.Sensors.gyro.Pigeon;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

abstract public class BaseSwerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final Gyro gyro;
    private final SwerveOdom odom;
    private final Field2d field;
    private final SwerveKinematics kinematics;
    private final double maxSpeed;

    private final boolean tuningMode;

    private Pose2d oldPose;
    private double oldTimeStamp;

    public BaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveKinematics kinematics, double maxSpeed,  boolean tuningMode) {
        this.tuningMode = tuningMode;

        this.gyro = gyro;
        gyro.zeroGyro();

        this.swerveMods = swerveMods;

        this.kinematics = kinematics;
        this.odom = new SwerveOdom(kinematics, getPose());

        this.field = new Field2d();

        this.maxSpeed = maxSpeed;

        this.oldPose = this.odom.getPose();
        this.oldTimeStamp = System.currentTimeMillis();

        SmartDashboard.putData("Field", field);
    }

    public BaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveKinematics kinematics, double maxSpeed) { this(swerveMods, gyro, kinematics, maxSpeed, false); }

    public void drive(ChassisSpeeds chassisSpeeds) {
        driveRobotOriented(
            new Translation2d(
                chassisSpeeds.vxMetersPerSecond,
                -chassisSpeeds.vyMetersPerSecond // TODO: FIX SHITY NEGATION 
            ),
            chassisSpeeds.omegaRadiansPerSecond
        );
    }

    public void driveRobotOriented(Translation2d vector, double rot) {
        SwerveModuleState[] states = kinematics.getStates(vector, rot);

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

    public void setOdomPose(Pose2d pose) { 
        odom.setPose(pose);
        gyro.setAngle(pose.getRotation());
    }

    public Pose2d getOdomPose() { return odom.getPose(); }

    public Gyro getGyro() { return gyro; }

    public void resetGyro() { gyro.zeroGyro(); }

    public double getAngleVelo() {
        return 1000 * (odom.getPose().getRotation().getRadians() - oldPose.getRotation().getRadians()) / (System.currentTimeMillis() - oldTimeStamp); // in radians per milisecond
    }

    public double getDriveVelo() {
        return 1000 * (odom.getPose().getTranslation().getNorm() - oldPose.getTranslation().getNorm()) / (System.currentTimeMillis() - oldTimeStamp); // in meters per milisecond 
    }

    @Override
    public void periodic() { 
        odom.updatePoseWithGyro(getPose(),  gyro.getAngle());

        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getAngle().getDegrees(), 0, 360));

        if (tuningMode) {
            SmartDashboard.putNumber("Robot Angle Velo", getAngleVelo());
            SmartDashboard.putNumber("Robot Velo", getDriveVelo());
        }

        oldPose = odom.getPose();
        oldTimeStamp = System.currentTimeMillis();

        field.setRobotPose(odom.getPose());

        for (SwerveModule i : swerveMods) {
            if (tuningMode) {
                SmartDashboard.putNumber("Swerve Module Angle " + i.moduleNumber, i.getAngle());
            }
            i.log();
        }
    }
}
