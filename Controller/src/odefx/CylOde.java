package odefx;

import javafx.scene.shape.Cylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeHelper;

public class CylOde extends Cylinder implements HasDGeom {
    private DGeom dGeom;

    public CylOde(double radius, double height, FxBody fb){
        super(radius, height);
        dGeom = OdeHelper.createCylinder(radius, height);
        fb.addGeom(dGeom);
    }

    @Override
    public DGeom getDGeom() {
        return dGeom;
    }
}
