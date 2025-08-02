package frc.robot.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class provides an easy way to link actions to the change of a value. Each object represents a value to which
 * callback actions can be bound using {@link #onChange(Runnable)}.
 */
public class ValueChangeEvent<V> {
    /** Functional interface for the body of a value change binding. */
    @FunctionalInterface
    private interface BindingBody<V> {
        /**
         * Executes the body of the binding.
         *
         * @param previous
         *            The previous state of the condition.
         * @param current
         *            The current state of the condition.
         */
        void run(V previous, V current);
    }

    /** The current value supplier. */
    private final Supplier<V> m_condition;
    /** Poller loop. */
    private final EventLoop m_loop;

    /**
     * Creates a new value change event based on the given condition.
     *
     * @param loop
     *            The loop instance that polls this change event.
     * @param condition
     *            the value represented by this change event.
     */
    public ValueChangeEvent(final EventLoop loop, final Supplier<V> condition) {
        m_loop = requireNonNullParam(loop, "loop", "ValueChangeEvent");
        m_condition = requireNonNullParam(condition, "condition", "ValueChangeEvent");
    }

    /**
     * Creates a new value change event based on the given condition.
     *
     * <p>
     * Polled by the default scheduler button loop.
     *
     * @param condition
     *            the value represented by this change event.
     */
    public ValueChangeEvent(final Supplier<V> condition) {
        this(CommandScheduler.getInstance().getDefaultButtonLoop(), condition);
    }

    /**
     * Adds a binding to the EventLoop.
     *
     * @param body
     *            The body of the binding to add.
     */
    private void addBinding(final BindingBody<V> body) {
        m_loop.bind(
                new Runnable() {
                    private V m_previous = m_condition.get();

                    @Override
                    public void run() {
                        final V current = m_condition.get();

                        body.run(m_previous, current);

                        m_previous = current;
                    }
                });
    }

    /**
     * Bind an action to this change event.
     *
     * @param action
     *            the action to run if this value changes.
     */
    public ValueChangeEvent<V> onChange(final Runnable action) {
        requireNonNullParam(action, "action", "onChange");
        addBinding(
                (previous, current) -> {
                    if (!previous.equals(current)) {
                        action.run();
                    }
                });
        return this;
    }
}
