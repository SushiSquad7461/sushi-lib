package SushiFrcLib.Swerve;

enum SDSModules {
    MK4i((150.0 / 7.0), 6.75),
    MK4(0,0);

    private double angleGearRatio;
    private double driveGearRatio;


    private SDSModules(double angleGearRatio, double driveGearRatio) {
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
    }

    public double getAngleGearRatio() {
        return angleGearRatio;
    }

    public double getDriveGearRatio() {
        return driveGearRatio;
    }
}