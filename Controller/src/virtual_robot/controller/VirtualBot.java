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
import odefx.FxBody;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DRotation;
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

    protected Group subSceneGroup = null;

    protected FxBody fxBody = null;

    protected double zBase = 0;

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
        setUpFxBody();
        zBase = fxBody.getPosition().get2();
    }

    static void setController(VirtualRobotController ctrl){
        controller = ctrl;
    }

    /**
     * Create the FxBody object that represents the robot. The DBody object belonging to this object should be the
     * robot chassis. The children should be additional FxBody objects representing other potentially-moving
     * robot parts. Immediately after creation, the FxBody should be positioned at (0, 0, zBase), where zBase
     * is the z-value that rests the wheels on the floor.
     *
     */
    protected abstract void setUpFxBody();

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
    public abstract DGeom.DNearCallback getNearCallback();



    /**
     *  Update the state of the robot. This includes updating forces and torques on DBody objects of the robot,
     *  as well as updating joint limits and motor-joint speeds. It will not directly update positions and angles,
     *  as that will be handled by the physics engine.
     *
     *  updateState is called on a non-UI thread via an ExecutorService object. For that reason,
     *  it SHOULD NOT make changes to the robot's graphical UI. Those changes should be made by
     *  overriding the updateDisplay() method, which is run on the UI thread.
     *
     *  @param millis milliseconds since the previous update
     */
    public abstract void updateState(double millis);

    /**
     * Update robot sensors base on its current position and orientation.
     * updateSensors is called on a non-UI thread via an ExecutorService object. For that reason,
     * it SHOULD NOT make changes to the robot's graphical UI. Those changes should be made by
     * overriding the updateDisplay() method, which is run on the UI thread.
     */
    public abstract void updateSensors();

    /**
     * Reposition the bot on the field. This is not for use during active simulation. It is to be used to position the
     * bot before starting the simulation. Note: this is a 2D function. It will. NOTE: this updates the display (via
     * calls to fxBody.setPosition and fxBody.setRotation), so it should only be called from the application thread.
     *
     * @param x
     * @param y
     * @param theta
     */
    public void setPosition(double x, double y, double theta){
        x = Math.max(-halfFieldWidth+halfBotWidth, Math.min(x, halfFieldWidth-halfBotWidth));
        y = Math.max(-halfFieldWidth+halfBotWidth, Math.min(y, halfFieldWidth-halfBotWidth));
        DMatrix3 R = new DMatrix3();
        DRotation.dRFromAxisAndAngle(R, 0, 0, 1, theta);
        fxBody.setPosition(x, y, zBase);
        fxBody.setRotation(R);
    }

    /**
     * Get bot's current 2D position (x, y, theta) on the field
     * @return x, y, theta
     */
    public double[] getPosition(){
        DVector3C pos = fxBody.getPosition();
        DMatrix3C rot = fxBody.getRotation();
        double theta = Math.atan2( rot.get10(), rot.get00());
        return new double[] {pos.get0(), pos.get1(), theta};
    }

    /**
     * Update bot display based on the current state of its FxBody objects.
     *
     * This must be called from the main application thread, via a Platform.runLater call if needed.
     */
    public void updateDisplay(){
        fxBody.updateNodeDisplay();
    }

    /**
     * Stop all motors; De-initialize or close other hardware (e.g. gyro/IMU) as appropriate.
     */
    public abstract void powerDownAndReset();

    /**
     * Position bot on field base on MouseEvent argument
     * @param arg
     */
    public void positionWithMouseClick(MouseEvent arg){
        double[] pos = getPosition(); //x, y, headingRadians
        if (arg.getButton() == MouseButton.PRIMARY) {
            double argX = Math.min(halfFieldWidth-halfBotWidth,
                    Math.max((arg.getX()- Config.SUBSCENE_WIDTH/2.0)*fieldWidth/Config.SUBSCENE_WIDTH, -(halfFieldWidth-halfBotWidth)));
            double argY = Math.min(halfFieldWidth-halfBotWidth,
                    Math.max(-(arg.getY()- Config.SUBSCENE_WIDTH/2.0)*fieldWidth/Config.SUBSCENE_WIDTH, -(halfFieldWidth-halfBotWidth)));
            setPosition(argX, argY, pos[2]);
        }
        else if (arg.getButton() == MouseButton.SECONDARY){
            double clickX = (arg.getX() - Config.SUBSCENE_WIDTH/2.0) * fieldWidth/Config.SUBSCENE_WIDTH;
            double clickY = (Config.SUBSCENE_WIDTH/2.0 - arg.getY()) * fieldWidth/Config.SUBSCENE_WIDTH;
            double radians = Math.atan2(clickY - pos[1], clickX - pos[0]) - Math.PI/2.0;
            if (radians > Math.PI) radians -= 2.0*Math.PI;
            else if (radians < -Math.PI) radians += 2.0 * Math.PI;
            setPosition(pos[0], pos[1], radians);
        }
    }

    /**
     * Not sure if this will be used.
     */
    public void removeFromDisplay(){
        subSceneGroup.getChildren().remove(fxBody.getNode());
        for (FxBody fxChild: fxBody.getChildren()){
            subSceneGroup.getChildren().remove(fxChild.getNode());
        }
    }

    public HardwareMap getHardwareMap(){ return hardwareMap; }

    /**
     * Create the HardwareMap object for the specific robot configuration, and assign it to the
     * hardwareMap variable.
     */
    protected abstract void createHardwareMap();

}
