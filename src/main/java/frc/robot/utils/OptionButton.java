package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * 
 */
public class OptionButton {

    private final CommandJoystick joystick;
    private final int buttonId;
    private final ActivationMode mode;

    private boolean isToggled;

    public static enum ActivationMode {
        /** Like toggle, but getting the state toggles off the button. Probably just use Trigger.onTrue() instead */
        TAP,

        /** standard button behavior, on when pressed, off when released */
        HOLD,

        /** makes button toggle on/off */
        TOGGLE
    }

    /**
     * Create Option button set to hold mode.
     * 
     * @param joystick Joystick that button is on
     * @param button Button number
     */
    public OptionButton(CommandJoystick joystick, int button) {
        this(joystick, button, ActivationMode.HOLD);
    }
    
    /**
     * Create Option button.
     * 
     * @param joystick Joystick that button is on
     * @param button Button number
     * @param mode Whether or not we want button to act as toggle or hold button
     */
    public OptionButton(CommandJoystick joystick, int button, ActivationMode mode) {
        this.joystick = joystick;
        this.buttonId = button;
        this.mode = mode;

        if (mode != ActivationMode.HOLD) {
            joystick.button(buttonId).onTrue(new InstantCommand(this::toggleOn));
            joystick.button(buttonId).onFalse(new InstantCommand(this::toggleOff));
        }
    }

    /** @return state of button before change */
    public boolean toggleOn() {
        final boolean startingState = isToggled;
        isToggled = true;
        return startingState;
    }
    
    /** @return state of button before change */
    public boolean toggleOff() {
        final boolean startingState = isToggled;
        isToggled = false;
        return startingState;
    }

    public boolean getState() {
        switch (mode) {  
            case TAP: return toggleOff();
            case HOLD: return joystick.button(buttonId).getAsBoolean();       
            case TOGGLE: return isToggled;
            default: return false; 
        }
    }

    public int getStateInt() {
        return getState() ? 1 : 0;
    }
}
