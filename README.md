# NEW:  Now using ODE4J (Java port of Open Dynamics Engine) to make the 3D simulator phyics based.

![](/readme_image.jpg)

**Some older hardware doesn't support Java FX 3D scenes.**  This runs on several circa 2015-16 Windows 10 and 8 systems
we have tested on, but not on a 2008 laptop (which was originally Vista, now Windows 10). If your system does not
support the 3D version, the original 2D version is available here: [virtual_robot](https://github.com/Beta8397/virtual_robot).

A 3D simulator to help beginning Java programmers learn to program for FTC Robotics.

This is a JavaFX application developed using the (free) IntelliJ IDEA Community Edition IDE. The repository can be downloaded
and unzipped, then opened with IntelliJ.

Four robot configurations are available, the first three of which are: a simple two-wheeled robot, a robot with
four mechanum wheels, and an X-Drive robot with four OmniWheels mounted at 45 degrees at each corner of the robot.
Each of these three robots has an rotating/extending arm attached to the back, controlled by DC motors. There is a
grabber at the end of the arm that is controlled by a servo. The fourth robot configuration is "BetaBot". This
is a mechanum-wheeled robot with a vertical lift, horizontal slider, and grabbing mechanism.

Each robot can be thought of as 18 inches wide.  For the two-wheel bot and mecanum wheel bots, the distance between
the centers of the right and left wheels is 16 inches. For the mecanum wheel bots, the distance between the centers
of the front and back wheels is 14 inches, and the mecanum wheels (when viewed from the top) have an "X" configuration.
For the X-Drive bot, the distance between the centers of any two adjacent wheels is 14.5 inches. Each motor has an
encoder. There is a downward-facing color sensor in the center of the robot. A gyro sensor (or BNO055 imu) is also included.
Each robot also has distance sensors on the front, left, right and back sides. Wheel diameters are all 4 inches.

The field can be thought of as 12 feet wide. The field graphic (currently the Skystone field)
is obtained from a bitmap (.bmp) image. The color sensor detects the field color beneath the center of the
robot. The field graphic is easily changed by providing a different .bmp image in the virtual_robot.config.Config class.
The .bmp image is the skysone_field648.bmp file in the virtual_robot.assets folder. As is, the 3D display scene is
800x800 pixels. This can be increased or decreased by changing the value of SUBSCENE_WIDTH in the
virtual_robot.config.Config.java class. Note: changing the subscene width does not require a change in the field graphic.

An abridged approximation of the FTC SDK is provided.

User-defined OpModes must be placed in the org.firstinspires.ftc.teamcode package, and must extend OpMode (or LinearOpMode). OpModes are
registered by placing a @TeleOp or @Autonomous annotation immediately above the class declaration.

The OpMode (and therefore LinearOpMode) class in the simulator provides access to:

  1. A HardwareMap object, which in turn provides access to the DCMotor objects, the gyro sensor, distance sensors,
     the servo, and the color sensor;
  2. Two GamePads(actual hardware gamepads, though there is an option to use a "virtual gamepad -- see Log of Changes below");
  3. A Telemetry object.

Several example OpModes are provided in the org.firstinspires.ftc.teamcode package.

New robot configurations can be created by extending the VirtualBot class; this requires some familiarity with the
JavaFX 3D API. Questions about this are welcome (via "Issues").

To use:

  1. Make sure you have the Java 8 JDK installed on your PC. Also, install the free Community Edition of JetBrains
     IntelliJ IDEA.
  2. Download the virtual_robot_3D.zip, and extract contents. Open the project in IntelliJ. You'll see three modules in
     the project (Controller, TeamCode, and virtual_robot) -- the only module you'll need to touch is TeamCode. It
     contains the org.firstinspires.ftc.teamcode package.
  3. Write your OpModes in the org.firstinspires.ftc.teamcode package; make sure to include a @TeleOp or @Autonomous annotation.
     These must extend the OpMode class (may either extend OpMode OR LinearOpMode). OpMode must provide init() and loop() methods;
     LinearOpMode must provide runOpMode() method.
  4. Make sure at least one gamepad is plugged in to the computer.
  5. Run the application (by clicking the green arrowhead at the toolbar).
  6. Press start-A or start-B on gamepad(s) to select which is gamepad1 vs. gamepad2.
  7. Use Configuration dropdown box to select a robot configuration. The configuration will be displayed.
  8. Use the Op Mode drop down box to select the desired OpMode.
  9. Prior to initialization, position the robot on the field by left-mouse-clicking the field (for robot position),
     and right-mouse-clicking (for robot orientation). This must be done with 3D camera in overhead view (center button).
  10. Use the INIT/START/STOP button as you would on the FTC Driver Station.
  11. Use the Camera buttons on the left to change the position of the 3D camera.
  12. If desired use the sliders to introduce random and systematic motor error, and inertia.


LOG OF CHANGES

CHANGES 1/7/2020
    Added 3D Utilities to make creation of robot configurations easier. Changed appearance of mechanum bot. Changed
    latency of IMU and MR-style gyro to 10 ms (it was 175 ms, much longer than is now seen with the "real" IMU).
    Added 3D SkyStone bridge to the scene.

CHANGES 12/31/2019
    Modified the original Virtual Robot application to use JavaFX 3D graphics. It is possible to create new robot
    configurations, but for the 3D version this is done completely in Java code.

CHANGES 12/16/2019
    Further changes to facilitate creation of new robot configurations. The robot configuration classes (e.g., 
    MechanumBot) still extend VirtualBot. But now, these classes are also the JavaFX Controller classes for 
    the fxml markup files that define the robot's graphical representation in the UI. The robot configuration class 
    must have a @BotConfig annotation that indicates the name of this config (as it will be displayed to the user)
    and the filename of its corresponding fxml file. The fxml file must have a Group object as its root, and must
    set the fx:controller attribute of that group to the name of the robot config class. Individual nodes
    in the group can be given fx:id attributes, which make them accessible in the robot config class by using
    a @FXML annotation. The easiest way to create a new configuration is to copy, then modify, the ".java" and ".fxml"
    files from an existing configuration (for example, MechanumBot.java and mechanum_bot.fxml). See extensive comments
    in the virtual_robot.controller.VirtualBot and virtual_robot.controller.robots.ArmBot classes and the
    virtual_robot.controller.robots.fxml.arm_bot.fxml file for more explanation.

CHANGES 12/12/2019
    Changes made to all more versatile building of new robot configurations. A transparent robot base layer (equal in
    width to the field) was added. This makes it possible for the robot to have accessories that extend well beyond
    the chassis in all four directions. A new robot configuration, ArmBot, was added. It has an extensible arm with a
    grabber at the end. The arm is DC Motor-operated. The grabber is Servo-operated. It is a mecanum-wheeled bot.

CHANGES 11/29/2019
    Range class and additional op modes contributed by FTC Team 16072. Servo interface (and ServoImpl class)
    enhanced with more features of the actual FTC SDK: ability to reverse direction and to scale position range.

CHANGES 10/6/2019
    Added the option of using "Virtual GamePad" instead of real GamePad. To do this, go to the Config.java class in the
    virtual_robot.config package (within the Controller module), and assign "true" to the USE_VIRTUAL_GAMEPAD constant.
    Other constants in this class include the field image (BACKGROUND) and the field width in pixels (FIELD_WIDTH). If
    changing FIELD_WIDTH, need to supply a square bitmap (.bmp) field image that is FIELD_WIDTH pixels wide.

CHANGES 8/17/2019
    RUN_TO_POSITION mode is now available for DcMotor, with setTargetPosition, getTargetPosition, and isBusy methods.
    Added 175 ms of latency to the BNO055IMU.

CHANGES 8/4/2019
    To better approximate real robot behavior, latency of 175ms added to the standard gyro sensor (used only on the
    Two-Wheel Bot). That is, updated values are available only every 175ms. The amount of latency can be changed
    easily in the createHardwareMap method of the virtual_robot.controller.robots.TwoWheelBot class. Will probably make a
    similar change to the BNO055IMU soon.

CHANGES 7/10/2019
    To improve plug and play with OpModes copied and pasted from Android Studio, multiple packages were renamed. In
    addition, Continuous Rotation Servo capability was added. The XDrive Bot now has a CR Servo in the back rather
    than a standard servo. The XDriveBotDemo op mode demonstrates the use of this servo, using gamepad2.

    NOTE: OpModes copied directly from Android Studio to Virtual Robot do not automatically compile when pasted into
    Virtual Robot in IntelliJ, and won't show up in the OpModes dropdown box until they are compiled. Three different
    methods to force compilation are: 1) Right click the file and select "Recompile"; 2) From the "Build" menu,
    select "Rebuild Project"; or, 3) Make any change at all to the OpMode file (e.g., add a comment). Any one of these
    methods is sufficient.

CHANGES 7/06/2019
    Now uses @TeleOp, @Autonomous, and @Disabled class annotations to control the display of OpModes in the OpMode
    combobox. For @TeleOp and @Autonomous, a name parameter must be specified. The group parameter is optional (default
    group is "default"). GamePad setJoystickDeadzone capability contributed by FTC team 16072.

CHANGES 7/01/2019
    Now supports two GamePads instead of just one. Use start-A and start-B to select gamepad1 and gamepad2, as
    you would in the FTC SDK. Two op modes for Mecanum Bot contributed by FTC team 16072, including a nice
    demonstration of field-centric drive using the IMU. These are in the org.firstinspires.ftc.teamcode.ftc16072 package.

CHANGES 6/25/2019
    Contribution from Alan Smith (alan412): now supports "regular" op modes in addition to linear op modes.

CHANGES 4/3/2019
    1. Added BNO055IMU interface to simulate (in a limited way) coding for the IMU in the REV Expansion Hub.
    2. The Mecanum Bot and X Drive Bot now have a BNO055IMU rather than the original gyro.
    3. The Two-Wheel Bot still has the original gyro.
    4. DCMotor interface renamed to DcMotor, in keeping the the FTC SDK.
    5. New utility classes: enum AngleUnit, enum AxesOrder, enum AxesReference, class Orientation

CHANGES 3/23/2019
    1. Uses real game pad (instead of the original "virtual" game pad.
    2. Added an X-Drive robot configuration.
    3. Tweaks to opModeIsActive() and addition of isStopRequested() to allow while() loop before START.
    4. Added Color class with single static method: RGBtoHSV(red, green, blue, hsv).
    5. Added distance sensors to all robot configurations to measure distance from walls.
    6. Replaced LineFollow example opMode with MechBotAutoDemo, a line follower that actually works.

