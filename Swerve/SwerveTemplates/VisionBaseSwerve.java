package SushiFrcLib.Swerve.SwerveTemplates;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.SmartDashboard.TunableNumber;
import SushiFrcLib.Swerve.SwerveModules.SwerveModule;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class VisionBaseSwerve extends BaseSwerve {
    protected final SwerveDrivePoseEstimator odom;
    private final SwerveDriveKinematics kinematics;
    protected final PhotonCamera camera;
    private final PhotonPoseEstimator camFilter;
    private final TunableNumber maxDistanceCamToTarget;


    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics,
            Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(swerveMods, gyro);
        this.kinematics = kinematics;

        this.odom = new SwerveDrivePoseEstimator(
                kinematics,
                getGyro().getAngle(),
                getModulePositions(),
                new Pose2d(0, 0, getGyro().getAngle()),
                stateStdDevs,
                visionMeasurementStdDevs);

        setPrevPose(this.odom.getEstimatedPosition());
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        camFilter = new PhotonPoseEstimator(
                AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d());
        maxDistanceCamToTarget = new TunableNumber("Max Cam->Target (m)", 6, true);

    }

    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics) {
        this(swerveMods, gyro, kinematics, VecBuilder.fill(0.1, 0.1, 0.05),
                VecBuilder.fill(0.9, 0.9, 0.9));
    }

    @Override
    public void drive(Translation2d vector, double rot) {
        vector = vector.times(maxSpeed);
        rot *= maxAngularVelocity;
        final var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vector.getX(), 
            vector.getY(), 
            rot, 
            odom.getEstimatedPosition().getRotation());
        driveChassis(chassisSpeeds);
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
        odom.resetPosition(gyro.getAngle(), getModulePositions(), pose);
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    @Override
    public Pose2d getOdomPose() {
        return odom.getEstimatedPosition();
    }

    public Command autoAlign() {
        return runOnce(() -> {
            camera.getAllUnreadResults();
        });
    }
    @Override
    public void periodic() {
        super.periodic();
        final var pose = odom.update(getGyro().getAngle(), getModulePositions());
        if(tuningMode) {
            SmartDashboard.putString("Odometry", pose.toString());
        }

        if (!camera.isConnected())
            return;
        
        List<PhotonPipelineResult> camResults = camera.getAllUnreadResults();
        if (camResults.size() == 0) {
            return;
        }
        PhotonPipelineResult camPhotonPipelineResult = camResults.get(camResults.size()-1); //set idx to wanted target
        
        var camPose = camFilter.update(camPhotonPipelineResult);

        if (!camPose.isPresent() || camPose.get().targetsUsed.size() < 2)
            return;

        var targetsCloseEnough = true;
        for (var target : camPose.get().targetsUsed) {
            var transform = target.getBestCameraToTarget();
            SmartDashboard.putString(transform.toString(), "transform");
            double cameraToTagDistance = new Pose3d().transformBy(transform).getTranslation().getNorm();
            if (cameraToTagDistance > maxDistanceCamToTarget.get()) {
                targetsCloseEnough = false;
                break;
            }
        }

        if (targetsCloseEnough) {
            addVisionTargets(List.of(camPose.get()));
        }

    }
}
