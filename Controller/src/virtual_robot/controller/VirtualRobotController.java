package virtual_robot.controller;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.studiohartman.jamepad.ControllerManager;
import com.studiohartman.jamepad.ControllerState;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.input.MouseDragEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.util.Callback;
import odefx.CBits;
import odefx.FxBody;
import odefx.FxBodyHelper;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.ode4j.ode.*;
import org.reflections.Reflections;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.config.Config;
import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.image.Image;
import javafx.scene.image.PixelReader;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import virtual_robot.controller.robots.BetaBot;
import virtual_robot.ftcfield.FtcField;
import virtual_robot.ftcfield.SkyStoneField;

import java.io.IOException;
import java.lang.annotation.Annotation;
import java.util.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static org.ode4j.ode.OdeConstants.*;

/**
 * For internal use only. Controller class for the JavaFX application.
 */
public class VirtualRobotController {

    //User Interface
    @FXML private ComboBox<Class<?>> cbxConfig;
    @FXML private Button driverButton;
    @FXML private ComboBox<Class<?>> cbxOpModes;
    @FXML private Slider sldRandomMotorError;
    @FXML private Slider sldSystematicMotorError;
    @FXML private Slider sldMotorInertia;
    @FXML private TextArea txtTelemetry;
    @FXML private CheckBox checkBoxGamePad1;
    @FXML private CheckBox checkBoxGamePad2;
    @FXML private BorderPane borderPane;
    @FXML private GridPane cameraGrid;
    @FXML private Button btnResetField;

    //JavaFx subscene and the group that will hold all of the subscene content
    private SubScene subScene;
    private Group subSceneGroup;

    //ODE world, botSpace, and contact group for holding all of the DBody and DGeom objects
    private DWorld world;
    public DWorld getWorld(){ return world; }

    private DSpace space;
    public DSpace getSpace() { return space; }

    private DJointGroup contactGroup;

    //Test Block
//    FxBody testBlock;
//    FxBody[] testBlocks = new FxBody[12];

    private FtcField ftcField;

    //Callback object for collision detection; it just calls the nearCallback method.
    private DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(data, o1, o2);
        }
    };

    //Used to follow mouse while dragging
    private double mouseX = 0;
    private double mouseY = 0;

    //Virtual Hardware
    private HardwareMap hardwareMap = null;
    private VirtualBot bot = null;
    GamePad gamePad1 = new GamePad();
    GamePad gamePad2 = new GamePad();
    GamePadHelper gamePadHelper = null;
    ScheduledExecutorService gamePadExecutorService = Executors.newSingleThreadScheduledExecutor();

    VirtualGamePadController virtualGamePadController = null;

    //Background Image and Field
    private final Image backgroundImage = Config.BACKGROUND;
    private PixelReader pixelReader = backgroundImage.getPixelReader();

    public static final double FIELD_WIDTH = 365.76;    // meters
    public static final double HALF_FIELD_WIDTH = FIELD_WIDTH / 2.0;
    private final double CAMERA_DISTANCE = 762;    // meters

    //Camera and Lighting
    private final PerspectiveCamera camera = new PerspectiveCamera(true);
    private final Rotate cameraElevationTransform = new Rotate(0, Rotate.X_AXIS);
    private final Rotate cameraAzimuthTransform = new Rotate(0, Rotate.Z_AXIS);
    private final Rotate cameraSteerXTransform = new Rotate(0, Rotate.X_AXIS);
    private final Rotate cameraSteerYTransform = new Rotate(0, Rotate.Y_AXIS);
    private boolean topView = true;
    private PointLight[][] lightArray = new PointLight[3][3];
    Button currentCameraButton;

    //Lists of OpMode classes and OpMode Names
    private ObservableList<Class<?>> nonDisabledOpModeClasses = null;

    //OpMode Control
    private OpMode opMode = null;
    private volatile boolean opModeInitialized = false;
    private volatile boolean opModeStarted = false;
    private Thread opModeThread = null;

    //Virtual Robot Control Engine
    ScheduledExecutorService executorService = null;
    public static final double TIMER_INTERVAL_MILLISECONDS = 33;

    //Telemetry
    private volatile String telemetryText;
    private volatile boolean telemetryTextChanged = false;

    //Random Number Generator
    private Random random = new Random();

    //Motor Error Slider Listener
    private ChangeListener<Number> sliderChangeListener = new ChangeListener<Number>() {
        @Override
        public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
            for (DcMotor motor: hardwareMap.dcMotor) {
                ((DcMotorImpl)motor).setRandomErrorFrac(sldRandomMotorError.getValue());
                ((DcMotorImpl)motor).setSystematicErrorFrac(sldSystematicMotorError.getValue() * 2.0 * (0.5 - random.nextDouble()));
                ((DcMotorImpl)motor).setInertia(1.0 - Math.pow(10.0, -sldMotorInertia.getValue()));
            }
        }
    };

    boolean getOpModeInitialized(){ return opModeInitialized; }

    public void initialize() {
        setupODE();
        setUp3DSubScene();
//        createField();
        ftcField = new SkyStoneField(subSceneGroup, world, space);
        ftcField.setup();
        currentCameraButton = (Button)getNodeByGridPaneIndex(cameraGrid, 1, 1);
        OpMode.setVirtualRobotController(this);
        VirtualBot.setController(this);
        setupCbxOpModes();
        setupCbxRobotConfigs();
        sldRandomMotorError.valueProperty().addListener(sliderChangeListener);
        sldSystematicMotorError.valueProperty().addListener(sliderChangeListener);
        sldMotorInertia.valueProperty().addListener(sliderChangeListener);
        if (Config.USE_VIRTUAL_GAMEPAD){
            checkBoxGamePad1.setVisible(false);
            checkBoxGamePad2.setVisible(false);
            FXMLLoader loader = new FXMLLoader(getClass().getResource("virtual_gamepad.fxml"));
            try{
                HBox hbox = (HBox)loader.load();
                virtualGamePadController = loader.getController();
                virtualGamePadController.setVirtualRobotController(this);
                borderPane.setBottom(hbox);
            } catch (IOException e){
                System.out.println("Virtual GamePad UI Failed to Load");
            }
            gamePadHelper = new VirtualGamePadHelper();
        } else {
            checkBoxGamePad1.setDisable(true);
            checkBoxGamePad1.setStyle("-fx-opacity: 1");
            checkBoxGamePad2.setDisable(true);
            checkBoxGamePad2.setStyle("-fx-opacity: 1");
            gamePadHelper = new RealGamePadHelper();
        }
        gamePadExecutorService.scheduleAtFixedRate(gamePadHelper, 0, 20, TimeUnit.MILLISECONDS);
    }

    private void setupCbxRobotConfigs(){
        Reflections reflections = new Reflections("virtual_robot.controller.robots");
        Set<Class<?>> configClasses = new HashSet<>();
        configClasses.addAll(reflections.getTypesAnnotatedWith(BotConfig.class));
        ObservableList<Class<?>> validConfigClasses = FXCollections.observableArrayList();
        for (Class<?> c: configClasses){
            if (!c.getAnnotation(BotConfig.class).disabled() && VirtualBot.class.isAssignableFrom(c))
                validConfigClasses.add(c);
        }
        cbxConfig.setItems(validConfigClasses);
        cbxConfig.setValue(BetaBot.class);

        cbxConfig.setCellFactory(new Callback<ListView<Class<?>>, ListCell<Class<?>>>() {
            @Override
            public ListCell<Class<?>> call(ListView<Class<?>> param) {
                final ListCell<Class<?>> cell = new ListCell<Class<?>>(){
                    @Override
                    protected void updateItem(Class<?> cl, boolean bln){
                        super.updateItem(cl, bln);
                        if (cl == null){
                            setText(null);
                            return;
                        }
                        Annotation a = cl.getAnnotation(BotConfig.class);
                        setText(((BotConfig)a).name());
                    }
                };
                return cell;
            }
        });

        cbxConfig.setButtonCell(new ListCell<Class<?>>(){
            @Override
            protected void updateItem(Class<?> cl, boolean bln) {
                super.updateItem(cl, bln);
                if (cl == null) {
                    setText(null);
                    return;
                }
                Annotation a = cl.getAnnotation(BotConfig.class);
                setText(((BotConfig) a).name());
            }
        });
    }


    public VirtualBot getVirtualBotInstance(Class<?> c){
        try {
            VirtualBot bot = (VirtualBot)c.newInstance();
            bot.init();
            return bot;
        } catch (Exception e){
            System.out.println("Unable to load robot configuration.");
            System.out.println(e.getMessage());
            return null;
        }
    }

    private void setupCbxOpModes(){
        //Reflections reflections = new Reflections(VirtualRobotApplication.class.getClassLoader());
        Reflections reflections = new Reflections("org.firstinspires.ftc.teamcode");
        Set<Class<?>> opModes = new HashSet<>();
        opModes.addAll(reflections.getTypesAnnotatedWith(TeleOp.class));
        opModes.addAll(reflections.getTypesAnnotatedWith(Autonomous.class));
        nonDisabledOpModeClasses = FXCollections.observableArrayList();
        for (Class<?> c : opModes){
            if (c.getAnnotation(Disabled.class) == null && OpMode.class.isAssignableFrom(c)){
                nonDisabledOpModeClasses.add(c);
            }
        }

        nonDisabledOpModeClasses.sort(new Comparator<Class<?>>() {
            @Override
            public int compare(Class<?> o1, Class<?> o2) {
                String group1 = null;
                Annotation a1 = o1.getAnnotation(TeleOp.class);
                if (a1 != null) group1 = ((TeleOp)a1).group();
                else{
                    a1 = o1.getAnnotation(Autonomous.class);
                    if (a1 != null) group1 = ((Autonomous)a1).group();
                }

                String group2 = null;
                Annotation a2 = o2.getAnnotation(TeleOp.class);
                if (a2 != null) group2 = ((TeleOp)a2).group();
                else{
                    a2 = o2.getAnnotation(Autonomous.class);
                    if (a2 != null) group2 = ((Autonomous)a2).group();
                }

                if (group1 == null) return -1;
                else if (group2 == null) return 1;
                else return group1.compareToIgnoreCase(group2);
            }
        });

        cbxOpModes.setItems(nonDisabledOpModeClasses);

        cbxOpModes.setCellFactory(new Callback<ListView<Class<?>>, ListCell<Class<?>>>() {
            @Override
            public ListCell<Class<?>> call(ListView<Class<?>> param) {
                final ListCell<Class<?>> cell = new ListCell<Class<?>>(){
                    @Override
                    protected void updateItem(Class<?> cl, boolean bln){
                        super.updateItem(cl, bln);
                        if (cl == null){
                            setText(null);
                            return;
                        }
                        Annotation a = cl.getAnnotation(TeleOp.class);
                        if (a != null) setText(((TeleOp)a).group() + ": " + ((TeleOp)a).name());
                        else {
                            a = cl.getAnnotation(Autonomous.class);
                            if (a != null) setText(((Autonomous)a).group() + ": "  + ((Autonomous)a).name());
                            else setText("No Name");
                        }

                    }
                };
                return cell;
            }
        });

        cbxOpModes.setButtonCell(new ListCell<Class<?>>(){
            @Override
            protected void updateItem(Class<?> cl, boolean bln) {
                super.updateItem(cl, bln);
                if (cl == null) {
                    setText(null);
                    return;
                }
                Annotation a = cl.getAnnotation(TeleOp.class);
                if (a != null) setText(((TeleOp) a).name());
                else {
                    a = cl.getAnnotation(Autonomous.class);
                    if (a != null) setText(((Autonomous) a).name());
                    else setText("No Name");
                }
            }
        });

        cbxOpModes.setValue(cbxOpModes.getItems().get(0));
    }


    @FXML
    public void setConfig(ActionEvent event){
        if (opModeInitialized || opModeStarted) return;
        if (bot != null) bot.removeFromDisplay();
        bot = getVirtualBotInstance(cbxConfig.getValue());
        if (bot == null) System.out.println("Unable to get VirtualBot Object");
        hardwareMap = bot.getHardwareMap();
        bot.addToDisplay();
        bot.setPosition(0, 0, 0);
        initializeTelemetryTextArea();
        sldRandomMotorError.setValue(0.0);
        sldSystematicMotorError.setValue(0.0);
        sldMotorInertia.setValue(0.0);
    }

    public Group getSubSceneGroup(){ return subSceneGroup; }

    @FXML
    private void handleDriverButtonAction(ActionEvent event){
        if (!opModeInitialized){
            if (!initOpMode()) return;
            txtTelemetry.setText("");
            driverButton.setText("START");
            opModeInitialized = true;
            cbxConfig.setDisable(true);
            Runnable runOpMode = new Runnable() {
                @Override
                public void run() {
                    runOpModeAndCleanUp();
                }
            };
            opModeThread = new Thread(runOpMode);
            opModeThread.setDaemon(true);

            Runnable singleCycle = new Runnable() {
                @Override
                public void run() {
                    singlePhysicsCycle();
                    Platform.runLater(new Runnable() {
                        @Override
                        public void run() {
                            updateDisplay();
                        }
                    });
                }
            };
            executorService = Executors.newSingleThreadScheduledExecutor();
            executorService.scheduleAtFixedRate(singleCycle, 0, 33, TimeUnit.MILLISECONDS);
            opModeThread.start();
        }
        else if (!opModeStarted){
            driverButton.setText("STOP");
            opModeStarted = true;
        }
        else{
            driverButton.setText("INIT");
            opModeInitialized = false;
            opModeStarted = false;
            //if (opModeThread.isAlive() && !opModeThread.isInterrupted()) opModeThread.interrupt();
            if (!executorService.isShutdown()) executorService.shutdown();
            try{
                opModeThread.join(500);
            } catch(InterruptedException exc) {
                Thread.currentThread().interrupt();
            }
            if (opModeThread.isAlive()) System.out.println("OpMode Thread Failed to Terminate.");
            bot.powerDownAndReset();
            if (Config.USE_VIRTUAL_GAMEPAD) virtualGamePadController.resetGamePad();
            initializeTelemetryTextArea();
            cbxConfig.setDisable(false);
        }
    }

    private synchronized void updateDisplay(){
        bot.updateDisplay();
//        for (int i=0; i<testBlocks.length; i++) testBlocks[i].updateNodeDisplay();
        ftcField.updateDisplay();
        updateTelemetryDisplay();
    }

    private synchronized void singlePhysicsCycle(){
        /**
         * Update bot sensors
         */
        bot.updateSensors();

        /**
         * Update forces and speeds of bot motor joints based on states of motors, bot position, etc.
         */
        bot.updateState(TIMER_INTERVAL_MILLISECONDS);

        /**
         * Check for collisions between geoms in space. The nearCallback will assign contact joints
         * between bodies based on the collisions detected. The joints are stored in contactGroup.
         */
        space.collide(null, nearCallback);

        /**
         * Do a physics simulation step (i.e., an integration step)
         */
        world.quickStep(TIMER_INTERVAL_MILLISECONDS/1000.0);

        /**
         * Empty the contractGroup. This contains the contact joints that were created during the previous
         * collision detection. Emptying it destroys those joints.
         */
        contactGroup.empty();
    }

    private void runOpModeAndCleanUp(){

        try {
            //For regular opMode, run user-defined init() method. For Linear opMode, init() starts the execution of
            //runOpMode on a helper thread.
            opMode.init();

            while (!opModeStarted && !Thread.currentThread().isInterrupted()) {
                //For regular opMode, run user-defined init_loop() method. For Linear opMode, init_loop checks whether
                //runOpMode has exited; if so, it interrupts the opModeThread.
                opMode.init_loop();
                //For regular op mode, update telemetry after each iteration of init_loop()
                //For linear op mode, do-nothing
                opMode.internalPostInitLoop();

                try {
                    Thread.sleep(0, 1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

            }

            //For regular opMode, run user-defined stop() method, if any. For Linear opMode, the start() method
            //will allow waitForStart() to finish executing.
            if (!Thread.currentThread().isInterrupted()) opMode.start();

            while (opModeStarted && !Thread.currentThread().isInterrupted()) {
                //For regular opMode, run user-defined loop() method. For Linear opMode, loop() checks whether
                //runOpMode has exited; if so, it interrupts the opModeThread.
                opMode.loop();
                //For regular op mode only, update telemetry after each execution of loop()
                //For linear op mode, do-nothing
                opMode.internalPostLoop();

                try {
                    Thread.sleep(0, 1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

            }

            //For regular opMode, run user-defined stop() method, if any. For Linear opMode, shut down the
            //helper thread that runs runOpMode.
            opMode.stop();
        } catch(Exception e){
            System.out.println("Exception thrown by opModeThread.");
            System.out.println(e.getClass().getName());
            System.out.println(e.getLocalizedMessage());
        }

        bot.powerDownAndReset();
        if (!executorService.isShutdown()) executorService.shutdown();
        opModeInitialized = false;
        opModeStarted = false;
        Platform.runLater(new Runnable() {
            public void run() {
                driverButton.setText("INIT");
                //resetGamePad();
                initializeTelemetryTextArea();
                cbxConfig.setDisable(false);
                if (Config.USE_VIRTUAL_GAMEPAD) virtualGamePadController.resetGamePad();
            }
        });

        System.out.println("Finished executing runOpModeAndCleanUp() on opModeThread.");
    }


    private boolean initOpMode() {
        try {
            Class opModeClass = cbxOpModes.getValue();
            opMode = (OpMode) opModeClass.newInstance();
        } catch (Exception exc){
            return false;
        }
        return true;
    }



    private void updateTelemetryDisplay(){
        if (telemetryTextChanged && telemetryText != null) txtTelemetry.setText(telemetryText);
        telemetryTextChanged = false;
    }

    private void initializeTelemetryTextArea(){
        StringBuilder sb = new StringBuilder();
        sb.append("Left-click to position bot.");
        sb.append("\nRight-click to orient bot.");
        sb.append("\n\nCONFIG");
        Set<String> motors = hardwareMap.dcMotor.keySet();
        if (!motors.isEmpty()) {
            sb.append("\n Motors:");
            for (String motor : motors) sb.append("\n   " + motor);
        }
        Set<String> servos = hardwareMap.servo.keySet();
        if (!servos.isEmpty()) {
            sb.append("\n Servos:");
            for (String servo : servos) sb.append("\n   " + servo);
        }
        Set<String> crservos = hardwareMap.crservo.keySet();
        if (!crservos.isEmpty()){
            sb.append("\n CR Servos:");
            for (String crservo : crservos) sb.append("\n   " + crservo);
        }
        Set<String> colorSensors = hardwareMap.colorSensor.keySet();
        if (!colorSensors.isEmpty()) {
            sb.append("\n Color Sensors:");
            for (String colorSensor : colorSensors) sb.append("\n   " + colorSensor);
        }
        Set<String> gyroSensors = hardwareMap.gyroSensor.keySet();
        if (!gyroSensors.isEmpty()) {
            sb.append("\n Gyro Sensors:");
            for (String gyroSensor : gyroSensors) sb.append("\n   " + gyroSensor);
        }
        Set<String> bno055IMUs = hardwareMap.keySet(BNO055IMU.class);
        if (!bno055IMUs.isEmpty()){
            sb.append("\n BNO055IMU Sensors:");
            for (String imuSensor : bno055IMUs) sb.append("\n   " + imuSensor);
        }
        Set<String> distanceSensors = hardwareMap.keySet(DistanceSensor.class);
        if (!distanceSensors.isEmpty()) {
            sb.append("\n Distance Sensors:");
            for (String distance : distanceSensors) sb.append("\n   " + distance);
        }
        txtTelemetry.setText(sb.toString());
    }

    public class ColorSensorImpl implements ColorSensor {
        private int red = 0;
        private int green = 0;
        private int blue = 0;
        public synchronized int red(){ return red; }
        public synchronized int green(){ return green; }
        public synchronized int blue(){ return blue; }

        public synchronized void updateColor(double x, double y){
            int colorX = (int)((x + HALF_FIELD_WIDTH) * backgroundImage.getWidth()/FIELD_WIDTH);
            int colorY = (int)((HALF_FIELD_WIDTH - y) * backgroundImage.getWidth()/FIELD_WIDTH);
            double tempRed = 0.0;
            double tempGreen = 0.0;
            double tempBlue = 0.0;
            for (int row = colorY-4; row < colorY+5; row++)
                for (int col = colorX - 4; col < colorX+5; col++){
                    Color c;
                    try {
                        c = pixelReader.getColor(col, row);
                    } catch (Exception exc){
                        return;
                    }
                    tempRed += c.getRed();
                    tempGreen += c.getGreen();
                    tempBlue += c.getBlue();
                }
            tempRed = Math.floor( tempRed * 256.0 / 81.0 );
            if (tempRed == 256) tempRed = 255;
            tempGreen = Math.floor( tempGreen * 256.0 / 81.0 );
            if (tempGreen == 256) tempGreen = 255;
            tempBlue = Math.floor( tempBlue * 256.0 / 81.0 );
            if (tempBlue == 256) tempBlue = 255;
            red = (int)tempRed;
            green = (int)tempGreen;
            blue = (int)tempBlue;
        }
    }

    public class DistanceSensorImpl implements DistanceSensor {
        private double distanceMM = distanceOutOfRange;
        private static final double MIN_DISTANCE = 50; //mm
        private static final double MAX_DISTANCE = 1000; //mm
        private static final double MAX_OFFSET = 7.0 * Math.PI / 180.0;

        public synchronized double getDistance(DistanceUnit distanceUnit){
            double result;
            if (distanceMM < MIN_DISTANCE) result = MIN_DISTANCE - 1.0;
            else if (distanceMM > MAX_DISTANCE) result = distanceOutOfRange;
            else result = distanceMM;
            switch(distanceUnit){
                case METER:
                    return result / 1000.0;
                case CM:
                    return result / 10.0;
                case MM:
                    return result;
                case INCH:
                    return result / 25.4;
                default:
                    return result;
            }
        }

        public synchronized void updateDistance(double x, double y, double headingRadians){
            final double piOver2 = Math.PI / 2.0;
            double temp = headingRadians / piOver2;
            int side = (int)Math.round(temp); //-2, -1 ,0, 1, or 2 (2 and -2 both refer to the bottom)
            double offset = Math.abs(headingRadians - (side * Math.PI / 2.0));
            if (offset > MAX_OFFSET) distanceMM = distanceOutOfRange;
            else switch (side){
                case 2:
                case -2:
                    distanceMM = (y + HALF_FIELD_WIDTH) * 10.0;
                    break;
                case -1:
                    distanceMM = (HALF_FIELD_WIDTH - x) * 10.0;
                    break;
                case 0:
                    distanceMM = (HALF_FIELD_WIDTH - y) * 10.0;
                    break;
                case 1:
                    distanceMM = (x + HALF_FIELD_WIDTH) * 10.0;
                    break;
            }
        }
    }


    /**
     * Base class for OpMode.
     */
    public class OpModeBase {
        protected final HardwareMap hardwareMap;
        protected final GamePad gamepad1;
        protected final GamePad gamepad2;
        protected final Telemetry telemetry;

        public OpModeBase() {
            hardwareMap = VirtualRobotController.this.hardwareMap;
            gamepad1 = gamePad1;
            this.gamepad2 = gamePad2;
            telemetry = new TelemetryImpl();
        }
    }

    public class TelemetryImpl implements Telemetry {

        public TelemetryImpl(){
            update();
        }

        /**
         * Add data to telemetry (note-must call update() to cause the data to be displayed)
         * @param caption The caption for this telemetry entry.
         * @param fmt Format string, for formatting the data.
         * @param data The data to be formatted by the format string.
         */
        public void addData(String caption, String fmt, Object... data){
            this.data.append(caption + ": ");
            String s = String.format(fmt, data);
            this.data.append(s + "\n");
        }

        /**
         * Add single data object to telemetry, with a caption (note-must call update() to cause the data to be displayed)
         * @param caption The caption for this telemetry entry.
         * @param data The data for this telemetry entry.
         */
        public void addData(String caption, Object data){
            this.data.append(caption + ":" + data.toString() + "\n");
        }


        /**
         * Replace any data currently displayed on telemetry with all data that has been added since the previous call to
         * update().
         */
        public void update(){
            setText(data.toString());
            data.setLength(0);
        }

        private void setText(String text){
            telemetryText = text;
            telemetryTextChanged = true;
        }

    }

    public interface GamePadHelper extends Runnable{
        public void quit();
    }

    public class VirtualGamePadHelper implements GamePadHelper {

        public void run() {
            VirtualGamePadController.ControllerState state = virtualGamePadController.getState();
            gamePad1.update(state);
            gamePad2.resetValues();
        }

        public void quit(){}
    }

    public class RealGamePadHelper implements  GamePadHelper {

        private int gamePad1Index = -1;
        private int gamePad2Index = -1;

        private boolean isConnected0 = true;
        private boolean isConnected1 = true;

        private boolean changingGamePadConfig = false;

        private ControllerManager controller = null;

        public RealGamePadHelper(){
            controller = new ControllerManager(2);
            controller.initSDLGamepad();
        }

        public void run(){
            boolean connectionChanged = false;
            boolean configChanged = false;

            ControllerState state0 = controller.getState(0);
            ControllerState state1 = controller.getState(1);

            if (state0.isConnected != isConnected0 || state1.isConnected != isConnected1){
                isConnected0 = state0.isConnected;
                isConnected1 = state1.isConnected;
                connectionChanged = true;
                System.out.println("isConnected0 = " + isConnected0 + "  isConnected1 = " + isConnected1);
            }

            if (state0.start && (state0.a || state0.b) || state1.start && (state1.a || state1.b)) {
                if (!changingGamePadConfig) {

                    changingGamePadConfig = true;

                    if (state0.start && state0.a) {
                        gamePad1Index = 0;
                        if (gamePad2Index == 0) gamePad2Index = -1;
                    } else if (state0.start && state0.b) {
                        gamePad2Index = 0;
                        if (gamePad1Index == 0) gamePad1Index = -1;
                    }

                    if (state1.start && state1.a) {
                        gamePad1Index = 1;
                        if (gamePad2Index == 1) gamePad2Index = -1;
                    } else if (state1.start && state1.b) {
                        gamePad2Index = 1;
                        if (gamePad1Index == 1) gamePad1Index = -1;
                    }

                    System.out.println("gamepad1 index = " + gamePad1Index + "   gamepad2 index = " + gamePad2Index);

                    configChanged = true;
                }
            } else {
                changingGamePadConfig = false;
            }

            if (configChanged || connectionChanged){
                Platform.runLater(new Runnable() {
                    @Override
                    public void run() {
                        checkBoxGamePad1.setSelected(gamePad1Index == 0 && isConnected0 || gamePad1Index == 1 && isConnected1);
                        checkBoxGamePad2.setSelected(gamePad2Index == 0 && isConnected0 || gamePad2Index == 1 && isConnected1);
                    }
                });
            }


            if (gamePad1Index == 0) gamePad1.update(state0);
            else if (gamePad1Index == 1) gamePad1.update(state1);
            else gamePad1.resetValues();

            if (gamePad2Index == 0) gamePad2.update(state0);
            else if (gamePad2Index == 1) gamePad2.update(state1);
            else gamePad2.resetValues();
        }

        public void quit(){
            controller.quitSDLGamepad();
        }

    }

    @FXML private void handleBtnResetFieldAction(ActionEvent event){
        if (opModeStarted || opModeInitialized) return;
        ftcField.reset();
    }

    @FXML public void handleLightButtonAction(ActionEvent event){
        int row = GridPane.getRowIndex((Button)event.getSource());
        int col = GridPane.getColumnIndex((Button)event.getSource());
        PointLight light = lightArray[row][col];
        if (light.isLightOn()){
            light.setLightOn(false);
            ((Button)event.getSource()).setStyle("-fx-background-color: darkgray");
        } else {
            light.setLightOn(true);
            ((Button)event.getSource()).setStyle("-fx-background-color: white");
        }
    }

    @FXML public void handleCameraButtonAction(ActionEvent event){
        int row = GridPane.getRowIndex((Button)event.getSource());
        int col = GridPane.getColumnIndex((Button)event.getSource());
        cameraSteerXTransform.setAngle(0);
        cameraSteerYTransform.setAngle(0);
        if (row == 1 && col == 1){
            topView = true;
            cameraElevationTransform.setAngle(0);
            cameraAzimuthTransform.setAngle(0);
            camera.setFieldOfView(2 * Math.atan(FIELD_WIDTH/(2.0 * CAMERA_DISTANCE)) * 180.0 / Math.PI);
        } else {
            topView = false;
            cameraElevationTransform.setAngle(60);
            cameraAzimuthTransform.setAngle( Math.atan2(col-1, row-1) * 180.0/Math.PI);
            if (row==1 || col==1) {
                camera.setFieldOfView(2 * Math.atan(FIELD_WIDTH/(2.0 * CAMERA_DISTANCE)) * 180.0 / Math.PI * 1.3);
            } else {
                camera.setFieldOfView(2 * Math.atan(FIELD_WIDTH/(2.0 * CAMERA_DISTANCE)) * 180.0 / Math.PI * 1.4);
            }
        }
        if (currentCameraButton != event.getSource()){
            currentCameraButton.setStyle("-fx-background-color: darkgray");
            currentCameraButton = (Button)event.getSource();
            currentCameraButton.setStyle("-fx-background-color: lightblue");
        }
    }

    public Node getNodeByGridPaneIndex(GridPane grid, int row, int col){
        ObservableList<Node> nodes = grid.getChildren();
        for (Node n: nodes){
            if (GridPane.getRowIndex(n) == row && GridPane.getColumnIndex(n) == col) return n;
        }
        return null;
    }

    PointLight getLamp(double x, double y, double z){
        PointLight light = new PointLight(Color.WHITE);
        light.getTransforms().add(new Translate(x, y, z));
        return light;
    }

    private void handleSubSceneMouseEvents(MouseEvent event){
        if (event.getEventType() == MouseEvent.MOUSE_CLICKED) {

            if (opModeInitialized || opModeStarted || !topView) return;
            bot.positionWithMouseClick(event);
        } else if (event.getEventType() == MouseDragEvent.DRAG_DETECTED){
            if (!topView) {
                mouseX = event.getSceneX();
                mouseY = event.getSceneY();
                subScene.startFullDrag();
                currentCameraButton.setStyle("-fx-background-color: darkgray");
            }
        } else if (event.getEventType() == MouseDragEvent.MOUSE_DRAG_OVER){
            if (!topView) {
                double deltaX = (event.getSceneX() - mouseX) / subScene.getWidth();
                double deltaY = (event.getSceneY() - mouseY) / subScene.getHeight();
                mouseX = event.getSceneX();
                mouseY = event.getSceneY();
                if (event.isPrimaryButtonDown()) {
                    if (!event.isAltDown()) {
                        double elev = cameraElevationTransform.getAngle();
                        elev -= deltaY * 90.0;
                        elev = Math.min(80, Math.max(0, elev));
                        double azim = cameraAzimuthTransform.getAngle();
                        azim -= deltaX * 90.0;
                        cameraElevationTransform.setAngle(elev);
                        cameraAzimuthTransform.setAngle(azim);
                    } else {
                        double fov = camera.getFieldOfView();
                        fov -= 90 * deltaY;
                        fov = Math.min(90, Math.max(10, fov));
                        camera.setFieldOfView(fov);
                    }
                } else {
                    double steerX = cameraSteerXTransform.getAngle();
                    double steerY = cameraSteerYTransform.getAngle();
                    steerX += deltaY * 45;
                    steerY -= deltaX * 45;
                    steerX = Math.min(20, Math.max(-20, steerX));
                    steerY = Math.min(20, Math.max(-20, steerY));
                    cameraSteerXTransform.setAngle(steerX);
                    cameraSteerYTransform.setAngle(steerY);
                }
            }
        }

        event.consume();
    }


    private void setupODE() {
        //Set up world
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = OdeHelper.createHashSpace(null);
        contactGroup = OdeHelper.createJointGroup();
        world.setGravity(0, 0, -980);
        world.setQuickStepNumIterations(12);
        world.setERP(0.8);
        world.setContactSurfaceLayer(0);
        System.out.println("Original damping: " + world.getLinearDamping() + "  " + world.getAngularDamping());
        world.setLinearDamping(0.01);
        world.setAngularDamping(0.1);
    }

    void shutDownODE(){
        space.destroy();
        world.destroy();
        OdeHelper.closeODE();
    }

    /**
     * Handles collisions by establishing attaching contact joints to bodies whose geometries have collided
     * @param data
     * @param o1
     * @param o2
     */
    public void nearCallback(Object data, DGeom o1, DGeom o2){

        assert(o1!=null);
        assert(o2!=null);

        if ( o1 instanceof DSpace || o2 instanceof DSpace )
        {
            /**
             * If either geom is a botSpace, recursively test the bodies within the spaces for collision
             */
            OdeHelper.spaceCollide2(o1,o2,data,nearCallback);
            return;
        }

        /** If neither geom is a botSpace, then test the two geoms against eachother for contact points
         *  Then, for each detected contact, create a contact joint that has the desired properties
         *  (e.g., bounciness and friction coefficient), and attach this joint to the two bodies. This
         *  will apply forces to the bodies during the next ODE integration step.
         */

        final int N = 32;
        DContactBuffer contacts = new DContactBuffer(N);
        int n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());//[0].geom),sizeof(dContact));

        if (n == 0) return;

        if (o1.getSpace() == bot.getBotSpace() || o2.getSpace() == bot.getBotSpace()){
            bot.handleContacts(n, o1, o2, contacts, contactGroup);
        } else {
            for (int i=0; i<n; i++)
            {
                DContact contact = contacts.get(i);
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;      //Enable bounce
                contact.surface.bounce = 0.5;
                contact.surface.bounce_vel = 2.0;
                contact.surface.mu = 0.5;
                contact.surface.soft_cfm = 0;
                contact.surface.soft_erp = 0.1;
                DJoint c = OdeHelper.createContactJoint (world,contactGroup,contact);
                c.attach (contact.geom.g1.getBody(), contact.geom.g2.getBody());
            }
        }
    }

    /**
     * Create and set up SubScene for 3D graphics. This includes creating SubScene, the subscene Group, setting up mouse
     * handling, adding subscene to center of border pane, setting up camera and lighting.
     *
     * Addition of physics should not require this method to be modified.
     */
    private void setUp3DSubScene(){

        subSceneGroup = new Group();
        subScene = new SubScene(subSceneGroup, Config.SUBSCENE_WIDTH, Config.SUBSCENE_WIDTH, true, SceneAntialiasing.DISABLED);
        subScene.setFill(Color.LIGHTSKYBLUE);

        subScene.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent event) {
                handleSubSceneMouseEvents(event);
            }
        });

        subScene.setOnDragDetected(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent event) {
                handleSubSceneMouseEvents(event);
            }
        });

        subScene.setOnMouseDragOver(new EventHandler<MouseDragEvent>() {
            @Override
            public void handle(MouseDragEvent event) {
                handleSubSceneMouseEvents(event);
            }
        });

        borderPane.setCenter(subScene);

        camera.getTransforms().addAll(
                cameraAzimuthTransform,
                cameraElevationTransform,
                new Rotate(180, Rotate.X_AXIS),
                new Translate(0, 0, -CAMERA_DISTANCE),
                cameraSteerYTransform,
                cameraSteerXTransform
        );

        camera.setFieldOfView(2 * Math.atan(FIELD_WIDTH/(2.0 * CAMERA_DISTANCE)) * 180.0 / Math.PI);
        camera.setFarClip(CAMERA_DISTANCE + HALF_FIELD_WIDTH*1.6);
        camera.setNearClip(CAMERA_DISTANCE - HALF_FIELD_WIDTH*1.6);

        for (int i=0; i<3; i++) {
            for (int j = 0; j < 3; j++) {
                lightArray[i][j] = getLamp(183 * (j - 1), 183 * (1 - i), 91);
                lightArray[i][j].setLightOn(false);
            }
        }

        subSceneGroup.getChildren().add(new AmbientLight(Color.WHITE));

        for (int i = 0; i < 3; i++) subSceneGroup.getChildren().addAll(lightArray[i]);

        subSceneGroup.getChildren().add(camera);
        subScene.setCamera(camera);


    }


}
