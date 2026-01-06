package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode scans a single REV 2M Distance Sensor and rumbles the
 * driver station gamepad when an object is detected within 2cm.
 * It assumes the sensor is configured with the name "sensor_distance".
 */
@TeleOp(name = "Sensor Distance Rumble", group = "Sensor")
public class distance extends LinearOpMode {

    private DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {
        // You will need to use the hardware map to get the distance sensor.
        // Ensure your sensor is configured in the hardware map with the name "sensor_distance".
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // get the distance in centimeters
            double distanceCM = sensorDistance.getDistance(DistanceUnit.CM);

            // check if the distance is less than 2 cm
            // Note: The REV color-range sensor can saturate at ~5cm, returning 5cm for
            // anything closer. The REV 2m distance sensor is generally better
            // for short distances.
            if (!Double.isNaN(distanceCM) && distanceCM < 2.0) {
                // If within range, rumble the controller for a short duration.
                // This call takes left motor power, right motor power, and duration in milliseconds.
                gamepad1.rumble(1.0, 1.0, 200); // Max power rumble for 200ms
            } else {
                gamepad1.stopRumble();
            }

            // Send the distance data to the Driver Station for debugging
            telemetry.addData("Distance (cm)", "%.02f", distanceCM);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
