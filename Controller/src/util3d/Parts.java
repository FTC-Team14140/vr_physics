package util3d;

import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;

public class Parts {

    public static final int BACK_LEFT = 0, FRONT_LEFT = 1, FRONT_RIGHT = 0, BACK_RIGHT = 1;

    public static Group mecanumWheel(double diameter, double width, int type){
        PhongMaterial wheelTreadMaterial = type%2 == 0?
                Util3D.imageMaterial("/virtual_robot/assets/mechwheelA_rotated.jpg") :
                Util3D.imageMaterial("/virtual_robot/assets/mechwheelB_rotated.jpg");
        PhongMaterial wheelSideMaterial = new PhongMaterial(Color.color(0.9, 0.9, 0.9));
        wheelSideMaterial.setSpecularColor(Color.color(0, 0, 0, 0));
        Group wheel = Util3D.cylinder((float)diameter/2.0f, (float)width, 10, 1, 1,
                true, wheelTreadMaterial, wheelSideMaterial);
        return wheel;
    }

    public static Group tetrixBox(float length, float height, float depth, float patternWidth){
        PhongMaterial tetrixMaterial = Util3D.imageMaterial("/virtual_robot/assets/tetrix.jpg");
        return Util3D.patternBox(length, height, depth, patternWidth, patternWidth, patternWidth, tetrixMaterial);
    }


}
