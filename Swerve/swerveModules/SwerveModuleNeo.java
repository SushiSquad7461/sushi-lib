package SushiFrcLib.Swerve.SwerveModules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeo extends SwerveModule {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;

    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkMaxPIDController anglePID;


    private double lastAngle;

    public SwerveModuleNeo(SwerveModuleConstants constants) {
        super(constants);

        driveMotor = constants.getDriveNeo();
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        lastAngle = getState().angle.getRadians();
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        // state = SwerveModuleState.optimize(state, getState().angle);

        double targetAngle = state.angle.getRadians();

        // double targetSpeed = state.velocity;

        // double delta = targetAngle - lastAngle;

        // if (Math.abs(delta) > 90) {
        //     targetSpeed = -targetSpeed;
        //     targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);        
        // }

        drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        double angle = Math.abs(state.speedMetersPerSecond) <= swerveModuleConstants.maxSpeed * 0.01
            ? lastAngle
            :  targetAngle;

        anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        lastAngle = angle;

        SmartDashboard.putNumber("Current Mod Encoder Angle: " + moduleNumber, Rotation2d.fromRadians(angleEncoder.getPosition()).getDegrees());
    }

    public SwerveModuleState getState() {
        double velocity = driveEncoder.getVelocity();
        Rotation2d rot = Rotation2d.fromRadians(angleEncoder.getPosition());
        return new SwerveModuleState(velocity, rot);
    }

    @Override
    public SwerveModulePosition getPose() {
        double distance = driveEncoder.getPosition();
        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModulePosition(distance, rot);
    }

    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(Units.degreesToRadians(getAngle())); 
    }
}
