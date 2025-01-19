package SushiFrcLib.Swerve.SwerveTemplates;

import SushiFrcLib.Sensors.gyro.Gyro;
import SushiFrcLib.Swerve.CustomSwerve.SwerveKinematics;
import SushiFrcLib.Swerve.CustomSwerve.SwerveOdom;
import SushiFrcLib.Swerve.SwerveModules.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class CustomBaseSwerve extends BaseSwerve {
    private final SwerveOdom odom;
    private final SwerveKinematics kinematics;

    public CustomBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveKinematics kinematics) {
        super(swerveMods, gyro);
        this.kinematics = kinematics;
        this.odom = new SwerveOdom(kinematics, getPose());
        setPrevPose(this.odom.getPose());
    }

    public void driveRobotOriented(Translation2d vector, double rot) {
        SwerveModuleState[] states = kinematics.getStates(vector, rot);
        driveRobotOriented(states);
    }

    public void setOdomPose(Pose2d pose) {
        odom.setPose(pose);
        setGyro(pose);
    }

    @Override
    public Pose2d getOdomPose() {
        return odom.getPose();
    }

    @Override
    public Command zeroGyro() {
        return (runOnce(() -> setOdomPose(new Pose2d(odom.getPose().getTranslation(), new Rotation2d(0)))));
    }

    @Override
    public void periodic() {
        super.periodic();
        odom.updatePoseWithGyro(getPose(), getGyro().getAngle());
    }
}
