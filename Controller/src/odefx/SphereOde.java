package odefx;

import javafx.scene.shape.Sphere;
import org.ode4j.ode.*;

public class SphereOde extends Sphere implements HasDGeom {
    private DGeom dGeom;

    public SphereOde(double radius, FxBody fb){
        super(radius);
        dGeom = OdeHelper.createSphere(radius);
        fb.addGeom(dGeom);
    }

    @Override
    public DGeom getDGeom() {
        return dGeom;
    }
}
