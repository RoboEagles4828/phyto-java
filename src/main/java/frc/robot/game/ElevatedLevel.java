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
    /** The current level. Has to be a reference since all interface variables must be static and final. */
    final static AtomicReference<ElevatedLevel> currentLevel = new AtomicReference<>(ReefLevel.L1);
    /** The {@link ValueChangeEvent} for the current elevated level. */
    final static ValueChangeEvent<ElevatedLevel> changeEvent = new ValueChangeEvent<>(ElevatedLevel::getCurrentLevel);

    /**
     * @return the current elevated level (never null).
     */
    static ElevatedLevel getCurrentLevel() {
        return currentLevel.get();
    }

    /**
     * Sets a new current elevated level.
     * 
     * @param nextLevel
     *            the elevated level to set. If null, the level does not change.
     */
    static void setCurrentLevel(final ElevatedLevel nextLevel) {
        if (nextLevel != null) {
            currentLevel.set(nextLevel);
        }
    }

    /**
     * This event fires when the current level changes from one value to any other value.
     * 
     * @return the trigger for the current elevated level.
     */
    static ValueChangeEvent<ElevatedLevel> getChangeEvent() {
        return changeEvent;
    }
}
