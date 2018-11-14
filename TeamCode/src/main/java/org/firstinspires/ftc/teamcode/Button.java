
/**
 * Created by ryanjin on 20/02/2018.
 */
package org.firstinspires.ftc.teamcode;
public class Button {
    private boolean previousState;
    private boolean currentState;

    public void init() {
        this.previousState = false;
    }

    public boolean rebound(boolean cs) {
        currentState = cs;
        if (previousState && !currentState) {
            previousState = currentState;
            return true;
        } else {
            previousState = currentState;
            return false;
        }
    }
}
