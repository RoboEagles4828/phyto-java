package frc.robot.game;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Defines the interesting levels for algae.
 */
public enum AlgaeLevel implements ElevatedLevel {
    /** Algae removal low. */
    DEALGAE_LOW,
    /** Algae removal high. */
    DEALGAE_HIGH,
    /** Score algae into barge. */
    SCORE_BARGE;

    /** The current target algae level. */
    private static AlgaeLevel currentLevel = DEALGAE_HIGH;
    /** A trigger for each enumeration value. */
    private final Trigger trigger;

    /**
     * Called to create each enumeration value. A trigger for each value.
     */
    private AlgaeLevel() {
        this.trigger = new Trigger(() -> getCurrentLevel() == this);
    }

    /**
     * @return the trigger for this enumeration value.
     */
    public Trigger getTrigger() {
        return this.trigger;
    }

    /**
     * @return the current target algae level (never null).
     */
    public static AlgaeLevel getCurrentLevel() {
        return currentLevel;
    }

    /**
     * Sets the next target algae level.
     * 
     * @param nextLevel
     *            the algae level to set. If null, the level does not change.
     */
    public static void setCurrentLevel(final AlgaeLevel nextLevel) {
        if (nextLevel != null) {
            currentLevel = nextLevel;
        }
    }
}
