package SushiFrcLib.DependencyInjection;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import SushiFrcLib.Constants.SushiConstants.DEPENDENCY_INJECTION;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotName {
    private static RobotName sInstance = null;
    private String name = null;

    public static RobotName getInstance() {
        if (sInstance == null) {
            sInstance = new RobotName();
        }
        return sInstance;
    }

    public RobotName() {
        readRobotName(null);
    }

    // Path is path in directory in /home/lvuser
    public RobotName(String path) {
        readRobotName(path);
    }

    public String readRobotName(String path) {
        // Map<String, String> env = System.getenv();
        // SmartDashboard.putString("Home", env.get("HOME"));
        String filePath = path == null ? DEPENDENCY_INJECTION.FILE_PATH : "/home/lvuser/" + path;
        File f = new File(filePath);

        try {
          Scanner reader = new Scanner(f);
          name = reader.nextLine();
          reader.close();
          SmartDashboard.putString("Robot Name: ", name);
        } catch (FileNotFoundException e) {
          SmartDashboard.putString("Robot Name: ", "file not found exception");
          System.out.println("\n\nERROR: THIS FILE DOES NOT EXSIST: " + filePath + " DEFAULTING TO COMP MODE\n\n");
          name = DEPENDENCY_INJECTION.COMP_NAME;
        }

        return name;
    }

    public String getName() {
        return name == null ? DEPENDENCY_INJECTION.COMP_NAME : name;
    }

    public boolean isComp() {
        return name == null || name == DEPENDENCY_INJECTION.COMP_NAME;
    }
}
