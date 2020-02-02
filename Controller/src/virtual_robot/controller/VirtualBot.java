package virtual_robot.controller;

import javafx.collections.ObservableList;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import virtual_robot.config.Config;

/**
 *   For internal use only. Abstract base class for all of the specific robot configurations.
 *
 *   A robot config class that extend VirtualBot must:
 *
 *   1) Provide a no-argument init() method whose first statement is super.init();
 *   2) Provide a setupDisplayGroup() method that returns a JavaFX Group object (the graphical representation
 *          of the robot).
 *   3) Provide a createHardwareMap() method;
 *   4) Provide a public synchronized updateStateAndSensors(double millis) method;
 *   5) Provide a public powerDownAndReset() method.
 *
 *   Optionally (and in most cases), it will also be necessary to:
 *
 *   Override the public synchronized updateDisplay() method to update the appearance of accessories.
 *   This override should have super.updateDisplay() as its first statement.
 *
 */
public abstract class VirtualBot {

    protected static VirtualRobotController controller;

    protected HardwareMap hardwareMap;

    protected Group displayGroup = null;

    protected Group subSceneGroup;
    protected double fieldWidth;
    protected double halfFieldWidth;
    protected double halfBotWidth;
    protected double botWidth;

    protected DSpace space;

    public VirtualBot(){
        subSceneGroup = controller.getSubSceneGroup();
        this.fieldWidth = VirtualRobotController.FIELD_WIDTH;
        halfFieldWidth = fieldWidth / 2.0;
        botWidth = fieldWidth / 8.0;
        halfBotWidth = botWidth / 2.0;
    }

    public void init(){
        createHardwareMap();
        setUpDisplayGroup();
    }

    static void setController(VirtualRobotController ctrl){
        controller = ctrl;
    }

    /**
     * Get the display group from the concrete robot class
     */
    protected abstract Group getDisplayGroup();

    /**
     * Set up the Group object that will be displayed as the virtual robot. The resource file should contain
     * a Group with a 75x75 rectangle (The chassis rectangle) as its lowest layer, and other robot components
     * on top of that rectangle.
     *
     */
    protected void setUpDisplayGroup(){

        displayGroup = getDisplayGroup();


        displayGroup.getTransforms().add(new Translate(0, 0));
        displayGroup.getTransforms().add(new Rotate(0, 0, 0));

        subSceneGroup.getChildren().add(displayGroup);
    }

    /**
     * Return the bot's DSpace object.
     * This will be used from the controller for bot-bot and bot-other collide handling.
     * @return
     */
    public DSpace getSpace(){
        return space;
    }

    /**
     * Return the bot's DNearCallback object.
     * This will be used by the controller for bot-bot and bot-other collide handling.
     * @return
     */
    public abstract DGeom.DNearCallback getDNearCallback();



    /**
     *  Update the state of the robot. This includes updating forces and torques on DBody objects of the robot,
     *  as well as updating joint limits and motor-joint speeds. It will not directly update positions and angles,
     *  as that will be handled by the physics engine.
     *
     *  Also, update the robot's sensors by calling the update.. methods of the sensors (e.g., the
     *  updateDistance(...) method of the distance sensors).
     *
     *  updateStateAndSensors is called on a non-UI thread via an ExecutorService object. For that reason,
     *  it SHOULD NOT make changes to the robot's graphical UI. Those changes should be made by
     *  overriding the updateDisplay() method, which is run on the UI thread.
     *
     *  @param millis milliseconds since the previous update
     */
    public abstract void updateStateAndSensors(double millis);

    /**
     * Reposition the bot on the field. This is not for use during active simulation. It is to be used to position the
     * bot before starting the simulation. Note: this is a 2D function. The implementation should position the bot
     * horizontally, with wheels on floor.
     *
     * @param x
     * @param y
     * @param theta
     */
    public abstract void setPosition(double x, double y, double theta);

    /**
     * Get bot's current 2D position (x, y, theta) on the field
     * @return
     */
    public abstract double[] getPosition();

    /**
     * Update bot display based on the current state of its FxBody objects.
     *
     * This must be called from the main application thread, via a Platform.runLater call if needed.
     */
    public abstract void updateDisplay();

    /**
     * Stop all motors; De-initialize or close other hardware (e.g. gyro/IMU) as appropriate.
     */
    public abstract void powerDownAndReset();

    public void positionWithMouseClick(MouseEvent arg){
        double[] pos = getPosition(); //x, y, headingRadians
        if (arg.getButton() == MouseButton.PRIMARY) {
            double argX = Math.min(halfFieldWidth-halfBotWidth,
                    Math.max((arg.getX()- Config.SUBSCENE_WIDTH/2.0)*fieldWidth/Config.SUBSCENE_WIDTH, -(halfFieldWidth-halfBotWidth)));
            double argY = Math.min(halfFieldWidth-halfBotWidth,
                    Math.max(-(arg.getY()- Config.SUBSCENE_WIDTH/2.0)*fieldWidth/Config.SUBSCENE_WIDTH, -(halfFieldWidth-halfBotWidth)));
            setPosition(argX, argY, pos[2]);
            updateDisplay();
        }
        else if (arg.getButton() == MouseButton.SECONDARY){
            double clickX = (arg.getX() - Config.SUBSCENE_WIDTH/2.0) * fieldWidth/Config.SUBSCENE_WIDTH;
            double clickY = (Config.SUBSCENE_WIDTH/2.0 - arg.getY()) * fieldWidth/Config.SUBSCENE_WIDTH;
            double radians = Math.atan2(clickY - pos[1], clickX - pos[0]) - Math.PI/2.0;
            if (radians > Math.PI) radians -= 2.0*Math.PI;
            else if (radians < -Math.PI) radians += 2.0 * Math.PI;
            setPosition(pos[0], pos[1], radians);
            updateDisplay();
        }
    }

    public void removeFromDisplay(){
        subSceneGroup.getChildren().remove(displayGroup);
    }

    public HardwareMap getHardwareMap(){ return hardwareMap; }

    /**
     * Create the HardwareMap object for the specific robot configuration, and assign it to the
     * hardwareMap variable.
     */
    protected abstract void createHardwareMap();

}
