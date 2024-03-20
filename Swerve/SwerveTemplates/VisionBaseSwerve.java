package SushiFrcLib.Swerve.SwerveTemplates;

import java.util.List;

import org.photonvision.EstimatedRobotPose;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.Swerve.SwerveModules.SwerveModule;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public abstract class VisionBaseSwerve extends BaseSwerve {
    protected final SwerveDrivePoseEstimator odom;
    private final SwerveDriveKinematics kinematics;

    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics,
            Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(swerveMods, gyro);
        this.kinematics = kinematics;

        this.odom = new SwerveDrivePoseEstimator(
                kinematics,
                getGyro().getAngle(),
                getPose(),
                new Pose2d(0, 0, getGyro().getAngle()),
                stateStdDevs,
                visionMeasurementStdDevs);

        setPrevPose(this.odom.getEstimatedPosition());
    }

    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics) {
        this(swerveMods, gyro, kinematics, VecBuilder.fill(0.1, 0.1, 0.05),
                VecBuilder.fill(0.9, 0.9, 0.9));
    }

    public void addVisionTargets(List<EstimatedRobotPose> poses) {
        for (int i = 0; i < poses.size(); ++i) {
            odom.addVisionMeasurement(poses.get(i).estimatedPose.toPose2d(), poses.get(i).timestampSeconds);
        }
    }

    @Override
    public void driveRobotOriented(Translation2d vector, double rot) {
        SwerveModuleState[] states = kinematics
                .toSwerveModuleStates(new ChassisSpeeds(vector.getX(), vector.getY(), rot));
        driveRobotOriented(states);
    }

    @Override
    public void driveChassis(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        driveRobotOriented(states);
    }

    public void setOdomPose(Pose2d pose) {
        odom.resetPosition(pose.getRotation(), getPose(), pose);
        setGyro(pose);
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    @Override
    public Pose2d getOdomPose() {
        return odom.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        super.periodic();
        odom.update(getGyro().getAngle(), getPose());
    }
}
