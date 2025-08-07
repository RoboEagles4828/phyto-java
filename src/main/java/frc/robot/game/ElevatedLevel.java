package frc.robot.game;

/**
 * A marker interface for levels the elevator needs to drive to.
 * 
 * <p>
 * This interface also provides the {@link ElevatedLevelTracker} for the robot.
 */
public interface ElevatedLevel {
    /**
     * The {@link ElevatedLevelTracker} for the robot's central level tracking.
     */
    public final static ElevatedLevelTracker TRACKER = new ElevatedLevelTracker();
}
