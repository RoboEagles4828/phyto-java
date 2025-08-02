package frc.robot.game;

import java.util.concurrent.atomic.AtomicReference;

import frc.robot.util.ValueChangeEvent;

/**
 * A marker interface for levels the elevator needs to drive to.
 * 
 * <p>
 * This interface also provides {@link ValueChangeEvent} for the current elevated level. That is the event will fire
 * each time the current elevated level changes.
 */
public interface ElevatedLevel {
    /**
     * The {@link ElevatedLevelTracker} for the robot's central level tracking.
     */
    public final static ElevatedLevelTracker TRACKER = new ElevatedLevelTracker();
}
