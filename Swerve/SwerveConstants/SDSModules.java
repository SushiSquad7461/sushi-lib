package SushiFrcLib.Swerve.SwerveConstants;

public enum SDSModules {
    MK4i((150.0/7.0), 6.12, 5, 2 * Math.PI),
    MK4(0,0,0,0);

    public double angleGearRatio;
    public double driveGearRatio;
    public double maxSpeed;
    public double maxAngularVelocity;


    private SDSModules(double angleGearRatio, double driveGearRatio, double maxSpeed, double maxAngularVelocity) {
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxSpeed = maxSpeed;
    }
}