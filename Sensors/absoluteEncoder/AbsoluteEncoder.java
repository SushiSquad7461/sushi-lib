package SushiFrcLib.Sensors.absoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteEncoder {
    private DutyCycleEncoder encoder;
    private double offset;
    private int inversion;

    public AbsoluteEncoder(int id, double offset, boolean inversion) {
        encoder = new DutyCycleEncoder(id);
        this.offset = offset;

        if (inversion) {
            this.inversion = -1;
        } else {
            this.inversion = 1;
        }
    }

    public double getPosition() {
        return encoder.get() * 360 * inversion - offset;
    }

    public double getNormalizedPosition() {
        return MathUtil.inputModulus(getPosition(), -180, 180);
    }
}
