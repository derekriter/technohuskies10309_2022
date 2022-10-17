package org.firstinspires.ftc.team10309.API;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    private final HashMap<OdoType, Float> rotations = new HashMap<>();

    private int ticks = 0;

    public RotaryDecoder() {
        for(OdoType type : OdoType.values()) {
            this.directions.put(type, Direction.UNSET);
            this.hasTicked.put(type, false);
            this.rotations.put(type, 0f);
        }
    }

    /**
     * Decoded A and B channels into degrees
     * @param channelA the digital signal of channel A
     * @param channelB the digital signal of channel B
     * @param type which type of odometer wheel is being decoded
     * @return the number of degrees the encoder has turned since the last frame
     */
    public float decodeToDeg(boolean channelA, boolean channelB, OdoType type, LinearOpMode opMode) {
        if(this.directions.get(type) == Direction.UNSET && (channelA || channelB)) {
            this.directions.put(type, channelA ? Direction.CW : Direction.CCW);
        }
        else if(!channelA && !channelB) {
            this.directions.put(type, Direction.UNSET);
            this.hasTicked.put(type, false);
        }

        if(channelA && this.directions.get(type) != Direction.UNSET && !this.hasTicked.get(type)) {
            float degPerTick = 360f / RobotInfo.ticksPerRevolution;
            float angle = degPerTick * (this.directions.get(type) == Direction.CW ? 1 : -1);

            this.rotations.put(type, this.rotations.get(type) + angle);
            this.hasTicked.put(type, true);

            ticks += this.directions.get(type) == Direction.CW ? 1 : -1;
        }

//        return this.rotations.get(type);
        opMode.telemetry.addData("Direction", this.directions.get(type));

        return ticks;
    }
}
