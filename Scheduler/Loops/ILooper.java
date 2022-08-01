package SushiFrcLib.Scheduler.Loops;

public interface ILooper {
    int register(Loop loop);

    void start();

    void stop();

    void outputToSmartDashboard();
}
