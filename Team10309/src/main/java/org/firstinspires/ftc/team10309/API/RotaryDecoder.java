package org.firstinspires.ftc.team10309.API;

import java.util.HashMap;

/**
 * A class responsible for decoding quadrature encoder values into degrees
 */
public class RotaryDecoder {

    /**
     * Represents the type of odometer wheel being used
     */
    public enum OdoType {
        DRIVE,
        STRAFE
    }

    /**
     * Represents clockwise or counterclockwise
     */
    public enum Direction {
        CW,
        CCW,
        UNSET
    }

    private final HashMap<OdoType, Direction> directions = new HashMap<>();
    private final HashMap<OdoType, Boolean> hasTicked = new HashMap<>();

    public RotaryDecoder() {
        for(OdoType type : OdoType.values()) {
            this.directions.put(type, Direction.UNSET);
            this.hasTicked.put(type, false);
        }
    }

    /**
     * Decoded A and B channels into degrees
     * @param channelA the digital signal of channel A
     * @param channelB the digital signal of channel B
     * @param type which type of odometer wheel is being decoded
     * @return the number of degrees the encoder has turned since the last frame
     */
    public float decodeToDeg(boolean channelA, boolean channelB, OdoType type) {
        if(this.directions.get(type) == Direction.UNSET && (channelA || channelB)) {
            this.directions.put(type,
                channelA ? Direction.CW : Direction.CCW
            );
        }
        else if(!channelA && !channelB) {
            this.directions.put(type, Direction.UNSET);
            this.hasTicked.put(type, false);
        }
        else if(channelA && channelB && this.directions.get(type) != Direction.UNSET && !this.hasTicked.get(type)) {
            float degPerTick = RobotInfo.ticksPerRevolution / 360f;
            this.hasTicked.put(type, true);

            return degPerTick * (this.directions.get(type) == Direction.CW ? 1 : -1);
        }

        return 0;
    }
}
