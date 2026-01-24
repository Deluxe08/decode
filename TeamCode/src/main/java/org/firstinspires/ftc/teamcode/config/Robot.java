
package org.firstinspires.ftc.teamcode.config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.Turret;

public class Robot {

    public static final int BLUE_TARGET_TAG_ID = 20;
    public static final int PIPELINE_ID_BLUE = 8;
    public static final int RED_TARGET_TAG_ID = 24;
    public static final int PIPELINE_ID_RED = 2;
    public static int current_pipeline_id = PIPELINE_ID_RED;
    public static int current_tag_id = RED_TARGET_TAG_ID;

    private Shooter shooterSubsystem;

    private DcMotorEx shooterMotor;
    public Turret turret;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Robot (HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        shooterSubsystem = new Shooter(hardwareMap);
        turret= new Turret(hardwareMap);
        this.telemetry = telemetry;
    }
}
