package SushiFrcLib.Swerve;

import SushiFrcLib.Controllers.ControllerMath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.Supplier;

/**
 * Command that controls teleop swerve.
 */
public class TeleopSwerveDrive extends CommandBase {
    private final BaseSwerve swerve;

    private final Supplier<Double> xAxisSupplier; 
    private final Supplier<Double> yAxisSupplier;
    private final Supplier<Double> rotSupplier;
    private final Supplier<Double> speedMultiplier;

    /**
     * Pass in defualt speed multiplier of 1.0
     */
    public TeleopSwerveDrive(BaseSwerve swerve,
        Supplier<Double> xAxisSupplier, 
        Supplier<Double> yAxisSupplier, 
        Supplier<Double> rotSupplier
    ) {
        this(
            swerve,
            xAxisSupplier,
            yAxisSupplier,
            rotSupplier,
            () -> 1.0
        );
    }

    /**
     * Set swerve subsytem, controlers, axis's, and other swerve paramaters.
     */
    public TeleopSwerveDrive(BaseSwerve swerve,
        Supplier<Double> xAxisSupplier, 
        Supplier<Double> yAxisSupplier, 
        Supplier<Double> rotSupplier,
        Supplier<Double> speedMultiplier
    ) {
        this.swerve = swerve;

        this.xAxisSupplier = xAxisSupplier;
        this.yAxisSupplier = yAxisSupplier;
        this.rotSupplier = rotSupplier;
        this.speedMultiplier = speedMultiplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forwardBack = yAxisSupplier.get() * speedMultiplier.get();
        double leftRight = xAxisSupplier.get()  * speedMultiplier.get();
        double rot = rotSupplier.get() * speedMultiplier.get();

        forwardBack = ControllerMath.applyDeadband(forwardBack, Constants.OI.STICK_DEADBAND);
        leftRight = ControllerMath.applyDeadband(leftRight, Constants.OI.STICK_DEADBAND);

        Translation2d translation = new Translation2d(forwardBack, leftRight);

        rot = ControllerMath.cube(rot);
        rot *= Constants.Swerve.MAX_ANGULAR_VELOCITY;

        swerve.drive(
            (new Translation2d(ControllerMath.cube(translation.getNorm()), translation.getAngle())).times(Constants.Swerve.MAX_SPEED), 
            rot
        );
    }
}
