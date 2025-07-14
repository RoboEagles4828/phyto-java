package frc.robot.game;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Defines the scoring levels of the reef.
 */
public enum ReefLevel {
    /** Reef level 1, aka the trough. */
    L1,
    /** Reef level 2. */
    L2,
    /** Reef level 3. */
    L3,
    /** Reef level 4. */
    L4;

    /** The current target reef level. */
    private static ReefLevel currentLevel = L1;
    /** A trigger for each enumeration value. */
    private final Trigger trigger;

    /**
     * Called to create each enumeration value. A trigger for each value.
     */
    private ReefLevel() {
        this.trigger = new Trigger(() -> getCurrentLevel() == this);
    }

    /**
     * @return the trigger for this enumeration value.
     */
    public Trigger getTrigger() {
        return this.trigger;
    }

    /**
     * @return the current target reef level (never null).
     */
    public static ReefLevel getCurrentLevel() {
        return currentLevel;
    }

    /**
     * Sets the next target reef level.
     * 
     * @param nextLevel
     *            the reef level to set. If null, the level does not change.
     */
    public static void setCurrentLevel(final ReefLevel nextLevel) {
        if (nextLevel != null) {
            currentLevel = nextLevel;
        }
    }
}
