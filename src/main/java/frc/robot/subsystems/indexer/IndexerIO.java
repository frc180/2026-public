package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IndexerIO {

    @Logged
    public class IndexerInputs {
        public double velocityRPS = 0;
        public double positionDegrees = 0;
        public double supplyCurrent = 0;
        public double supplyCurrentFollower = 0;
        public double statorCurrent = 0;
        public double hopperDistance = 0;
        public double distanceStrength = 0;
    }

    public void setPower(double power);

    public  void setPositionDegrees(double degrees);

    public void setVelocity(double rps);

    public void update(IndexerInputs inputs);

    public void simulationPeriodic();
}
