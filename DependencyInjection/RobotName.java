package SushiFrcLib.DependencyInjection;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotName {
    private static RobotName sInstance = null;
    private String name = null;

    private static final String FILE_PATH = "/home/lvuser/name.txt";
    private static final String COMP_NAME = "comp";

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
        String filePath = path == null ? RobotName.FILE_PATH : "/home/lvuser/" + path;
        File f = new File(filePath);

        try {
          Scanner reader = new Scanner(f);
          name = reader.nextLine();
          reader.close();
          SmartDashboard.putString("Robot Name: ", name);
        } catch (FileNotFoundException e) {
          SmartDashboard.putString("Robot Name: ", "file not found exception");
          System.out.println("\n\nERROR: THIS FILE DOES NOT EXSIST: " + filePath + " DEFAULTING TO COMP MODE\n\n");
          name = RobotName.COMP_NAME;
        }

        SmartDashboard.putString("Robot Name", name);
        return name;
    }

    public String getName() {
        return name == null ? RobotName.COMP_NAME : name;
    }

    public boolean isComp() {
        return name == null || name == RobotName.COMP_NAME;
    }
}
