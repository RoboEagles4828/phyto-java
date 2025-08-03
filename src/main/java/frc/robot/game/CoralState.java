package frc.robot.game;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Defines the various states a coral game piece can enter while the robot interacts with it.
 */
public enum CoralState {
    /** There is no coral in the robot (that we know of). */
    EMPTY,
    /** The robot will actively intake coral from the human player. */
    INTAKE,
    /** The robot has a properly loaded coral and will carry it. */
    CARRY,
    /** The robot is preparing to score (elevator to proper level). */
    PREPARE_TO_SCORE,
    /** The robot is ready to score its coral. */
    READY_TO_SCORE,
    /** The robot will actively score the coral onto the reef. */
    SCORE,
    /** The coral is jammed in the hopper. Agitation will be performmed. */
    HOPPER_JAMMED,
    /** The coral is jammed below the cannon. Raise elevator and driver should work to free it. */
    ELEVATOR_JAMMED;

    /** The current coral state. We always start a match with one coral. */
    private static CoralState currentState = CARRY;
    /** A trigger for each enumeration value. */
    private final Trigger trigger;

    /**
     * Called to create each enumeration value. A trigger for each value.
     */
    private CoralState() {
        this.trigger = new Trigger(this::isCurrent);
    }

    /**
     * @return true if this enum value is the current state, or false otherwise.
     */
    public boolean isCurrent() {
        return getCurrentState() == this;
    }

    /**
     * @return the trigger for this enumeration value.
     */
    public Trigger getTrigger() {
        return this.trigger;
    }

    /**
     * @return the current coral state (never null).
     */
    public static CoralState getCurrentState() {
        return currentState;
    }

    /**
     * Sets the next coral state.
     * 
     * @param nextState
     *            the state to set. If null, the state does not change.
     */
    public static void setCurrentState(final CoralState nextState) {
        if (nextState != null) {
            currentState = nextState;
        }
    }
}
