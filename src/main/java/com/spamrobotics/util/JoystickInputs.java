package com.spamrobotics.util;

import edu.wpi.first.math.MathUtil;

public class JoystickInputs {

    public static final DoubleFunction SQUARE_KEEP_SIGN = (x) -> Math.copySign(x * x, x);
    public static final DoubleFunction CUBE_KEEP_SIGN = (x) -> Math.copySign(x * x * x, x);

    public double x = 0;
    public double y = 0;

    public JoystickInputs() {}

    public JoystickInputs(double x, double y) {
        of(x, y);
    }

    public double getPolarDistance() {
        return Math.sqrt((x * x) + (y * y));
    }

    public double getPolarTheta() {
        return Math.atan2(y, x);
    }
    
    public JoystickInputs of(double x, double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    public JoystickInputs transform(DoubleFunction transform) {
        x = transform.apply(x);
        y = transform.apply(y);
        return this;
    }

    public JoystickInputs deadband(double deadband) {
        x = MathUtil.applyDeadband(x, deadband);
        y = MathUtil.applyDeadband(y, deadband);
        return this;
    }

    public JoystickInputs clamp(double clamp) {
        x = MathUtil.clamp(x, -clamp, clamp);
        y = MathUtil.clamp(y, -clamp, clamp);
        return this;
    }

    public JoystickInputs polarDistanceTransform(DoubleFunction transform) {
        double theta = getPolarTheta();
        double distance = transform.apply(getPolarDistance());
        x = distance * Math.cos(theta);
        y = distance * Math.sin(theta);
        return this;
    }
}

