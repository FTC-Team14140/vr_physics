package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "two wheel tank drive", group = "TwoWheel")
public class TwoWheelTankDrive extends OpMode {

    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor armExtensionMotor = null;
    private DcMotor armRotationMotor = null;
    private GyroSensor gyro = null;
    private Servo fingerServo = null;
    private ColorSensor colorSensor = null;
    private DistanceSensor frontDistance = null;
    private DistanceSensor leftDistance = null;
    private DistanceSensor backDistance = null;
    private DistanceSensor rightDistance = null;

    private ElapsedTime et = null;
    private int waitForStartTime = 0;

    public void init(){
        left = hardwareMap.dcMotor.get("left_motor");
        right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);
        armExtensionMotor = hardwareMap.dcMotor.get("arm_extension_motor");
        armRotationMotor = hardwareMap.dcMotor.get("arm_rotation_motor");
        gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        fingerServo = hardwareMap.servo.get("finger_servo");
        gyro.init();
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");

        et = new ElapsedTime();
    }

    public void init_loop(){
        if (et.milliseconds() >= 1000) {
            waitForStartTime++;
            et.reset();
        }
        telemetry.addData("Press Start to Continue"," %d", waitForStartTime);
    }

    public void loop(){
        left.setPower(-gamepad1.left_stick_y);
        right.setPower(-gamepad1.right_stick_y);

        armExtensionMotor.setPower(-gamepad2.left_stick_y);
        armRotationMotor.setPower(gamepad2.right_stick_y);
        if (gamepad2.x) fingerServo.setPosition(1);
        else if (gamepad2.b) fingerServo.setPosition(0);

        telemetry.addData("Gamepad 1 sticks control drive","");
        telemetry.addData("Gamepad 2 sticks control arm", "");
        telemetry.addData("Gamepad 2 X and B control fingers.","");
        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("Heading"," %.1f", gyro.getHeading());
        telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
        telemetry.addData("Distance", " Fr %.1f  Lt %.1f  Rt %.1f  Bk %.1f  ",
                frontDistance.getDistance(DistanceUnit.CM), leftDistance.getDistance(DistanceUnit.CM),
                rightDistance.getDistance(DistanceUnit.CM), backDistance.getDistance(DistanceUnit.CM)
        );

    }

}
