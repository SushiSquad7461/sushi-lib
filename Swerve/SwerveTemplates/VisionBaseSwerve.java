package SushiFrcLib.Swerve.SwerveTemplates;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.Swerve.SwerveModules.SwerveModule;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class VisionBaseSwerve extends BaseSwerve {
    private final SwerveDrivePoseEstimator odom;
    private final SwerveDriveKinematics kinematics;

    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics) {
        super(swerveMods, gyro);
        this.kinematics = kinematics;

        this.odom = new SwerveDrivePoseEstimator(
            kinematics, 
            getGyro().getAngle(),
            getPose(),
            new Pose2d(0,0, getGyro().getAngle())
        );
    }

    @Override
    public void driveRobotOriented(Translation2d vector, double rot) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(vector.getX(), vector.getY(), rot));
        driveRobotOriented(states);
    }

    public void setOdomPose(Pose2d pose) { 
        odom.resetPosition(pose.getRotation(), getPose(), pose);
        setGyro(pose);
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getState());
    }

    @Override
    public Pose2d getOdomPose() { return odom.getEstimatedPosition(); }

    @Override
    public void periodic() { 
        super.periodic();
        odom.update(getGyro().getAngle(), getPose());
    } 
}
