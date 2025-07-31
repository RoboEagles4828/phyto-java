package frc.robot.game;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Defines the various states an algae game piece can enter while the robot interacts with it.
 * 
 * <p>
 * Note that we do not track algae containment. We just dealgae the reef and if the drive team sees that the robot held
 * onto the algae, they can choose to go score it. The {@link CoralState} enumerations that affect elevator movement are
 * always used to get the elevator to the proper level, even for algae.
 * 
 * <p>
 * TODO this probably needs more work.
 */
public enum AlgaeState {
    /** The robot starts without an algae and the manipulator is stowed. */
    INACTIVE,
    /** An algae is to be removed from the reef. */
    REMOVE_FROM_REEF,
    /** Algae removal done, may have it. Drive team can cancel if not held. */
    MAY_HAVE_ALGAE,
    /** The robot will actively score the algae into the barge. */
    LOAD_BARGE;

    /** The current algae state. We always start a match without an algae. */
    private static AlgaeState currentState = INACTIVE;
    /** A trigger for each enumeration value. */
    private final Trigger trigger;

    /**
     * Called to create each enumeration value. A trigger for each value.
     */
    private AlgaeState() {
        this.trigger = new Trigger(() -> getCurrentState() == this);
    }

    /**
     * @return the trigger for this enumeration value.
     */
    public Trigger getTrigger() {
        return this.trigger;
    }

    /**
     * @return the current algae state (never null).
     */
    public static AlgaeState getCurrentState() {
        return currentState;
    }

    /**
     * Sets the next algae state.
     * 
     * @param nextState
     *            the state to set. If null, the state does not change.
     */
    public static void setCurrentState(final AlgaeState nextState) {
        if (nextState != null) {
            currentState = nextState;
        }
    }
}
