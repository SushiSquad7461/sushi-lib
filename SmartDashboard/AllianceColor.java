package SushiFrcLib.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// TODO see notes in Robot.java of futomaki-2023
public class AllianceColor {
    private Boolean isRedAlliance;
    private NetworkTable table;

   public static AllianceColor instance; 

   public static AllianceColor getInstance() {
        if (instance == null) {
            instance = new AllianceColor();
        }
        return instance;
   }

   private AllianceColor() {
        table = NetworkTableInstance.getDefault().getTable("FMSInfo");
        isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
   }

   public void updateAllianceColor() {
    isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
   }

   public boolean isRed() {
    return isRedAlliance;
   }

   public boolean isBlue() {
     return !isRedAlliance;
   }
}
