package virtual_robot.ftcfield;


import javafx.scene.Group;
import org.ode4j.ode.*;

public abstract class FtcField {

    protected Group subSceneGroup;
    protected DWorld world;
    protected DSpace space;

    public FtcField(Group group, DWorld world, DSpace space){
        this.subSceneGroup = group;
        this.world = world;
        this.space = space;
    }

    public abstract void setup();

    public abstract void reset();

    public abstract void updateDisplay();

    public void preStepProcess(){}

    public abstract void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup);

}
