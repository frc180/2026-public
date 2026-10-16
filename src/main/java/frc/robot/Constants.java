package frc.robot;

import com.ctre.phoenix6.CANBus;

// Named constant values for the robot, including electrical mappings
public class Constants {

    public static final String CANIVORE = "Pseudo-Tesla";
    public static final CANBus CANIVORE_BUS = new CANBus(CANIVORE);

    public static final double LOOP_TIME = 0.02;

    // =================== CAN IDs - CANivore ===================

    // Motor controllers
    public static final int TURRET = 15;
    public static final int INTAKE = 55;
    public static final int INTAKE_EXTENSION = 50;
    public static final int SHOOTER_PRIMARY = 18;
    public static final int SHOOTER_FOLLOWER_A = 19;
    public static final int KICKER = 51;
    public static final int SHOOTER_HOOD = 21;

    public static final int INDEXER_A = 13;
    public static final int INDEXER_B = 14;

    public static final int PREKICKER = 52;

    public static final int INTAKE_SECONDARY = 16;

    // CANcoders

    public static final int SHOOTER_HOOD_ENCODER = 22;
    public static final int TURRET_ENCODER_A = 23; // "10:1"
    public static final int TURRET_ENCODER_B = 24; // "11:1"

    // Misc
    public static final int CANRANGE = 53;
    public static final int SHOOTER_CANRANGE = 54;
}
