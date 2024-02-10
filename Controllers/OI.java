package SushiFrcLib.Controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** 
 * Handels xbox controllers.
*/
public class OI {
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    private static OI instance;


    public static final class OIConstants  {
        public static final int DRIVE_TRANSLATION_Y = XboxController.Axis.kLeftY.value;
        public static final int DRIVE_TRANSLATION_X = XboxController.Axis.kLeftX.value;
        public static final int DRIVE_ROTATE = XboxController.Axis.kRightX.value;


        public static final int DRIVE_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    /**
     * Makes class a singelton.
     */
    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    private OI() {
        driverController = new CommandXboxController(OIConstants.DRIVE_PORT);
        operatorController = new CommandXboxController(OIConstants.OPERATOR_PORT);

    }
    public double getDriveTrainRotation() {
        return getRawAxis(OIConstants.DRIVE_ROTATE);
    }

    public double getDriveTrainTranslationY() {
        return getRawAxis(OIConstants.DRIVE_TRANSLATION_Y);
    }

    public double getDriveTrainTranslationX() {
        return getRawAxis(OIConstants.DRIVE_TRANSLATION_X);
    }

    private double getRawAxis(int id) {
        return driverController.getHID().getRawAxis(id) * -1;
    }

    public CommandXboxController getDriverController() {
        return driverController;
    }

    public CommandXboxController getOperatorController() {
        return operatorController;
    }
}
