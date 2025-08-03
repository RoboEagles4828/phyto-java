package frc.robot.game;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.ValueChangeEvent;

/**
 * This class tracks the current {@link ElevatedLevel} for the robot. It also provides events and triggers to help other
 * parts of the system to track the current level.
 */
public class ElevatedLevelTracker {
    /** The current level. */
    private ElevatedLevel currentLevel = CoralLevel.L1;
    /** The {@link ValueChangeEvent} for the current elevated level. */
    private ValueChangeEvent<ElevatedLevel> changeEvent = new ValueChangeEvent<>(this::getCurrentLevel);
    /** Trigger that is true when the current level is a coral scoring level. */
    private Trigger isCoralLevelTrigger = new Trigger(this::isCurrentCoralLevel);
    /** Trigger that is true when the current level is an algae level. */
    private Trigger isAlgaeLevelTrigger = new Trigger(this::isCurrentAlgaeLevel);

    /**
     * Creates the tracker. Formally declared to be package scoped.
     */
    ElevatedLevelTracker() {
    }

    /**
     * @return the current elevated level (never null).
     */
    public ElevatedLevel getCurrentLevel() {
        return this.currentLevel;
    }

    /**
     * Sets a new current elevated level.
     * 
     * @param nextLevel
     *            the elevated level to set. If null, the level does not change.
     */
    public void setCurrentLevel(final ElevatedLevel nextLevel) {
        if (nextLevel != null) {
            this.currentLevel = nextLevel;
        }
    }

    /**
     * This event fires when the current level changes from one value to any other value.
     * 
     * @return the trigger for the current elevated level.
     */
    public ValueChangeEvent<ElevatedLevel> getChangeEvent() {
        return this.changeEvent;
    }

    /**
     * @return true if the current elevated level is a coral scoring level.
     */
    public boolean isCurrentCoralLevel() {
        return getCurrentLevel().getClass() == CoralLevel.class;
    }

    /**
     * @return trigger that is true when the current level is a coral scoring level.
     */
    public Trigger getIsCurrentCoralLevelTrigger() {
        return this.isCoralLevelTrigger;
    }

    /**
     * @return true if the current elevated level is an algae level.
     */
    public boolean isCurrentAlgaeLevel() {
        return getCurrentLevel().getClass() == AlgaeLevel.class;
    }

    /**
     * @return trigger that is true when the current level is an algae level.
     */
    public Trigger getIsCurrentAlgaeLevelTrigger() {
        return this.isAlgaeLevelTrigger;
    }
}
