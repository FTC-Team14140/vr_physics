package virtual_robot.controller;

import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

/**
 * For internal use only. Main class for the JavaFX application.
 */
public class VirtualRobotApplication extends Application {

    private static VirtualRobotController controllerHandle;

    @Override
    public void start(Stage primaryStage) throws Exception{
        FXMLLoader loader = new FXMLLoader(getClass().getResource("virtual_robot.fxml"));
        Parent root = (BorderPane)loader.load();
        controllerHandle = loader.getController();
        primaryStage.setTitle("Virtual Robot");
        primaryStage.setScene(new Scene(root));
        primaryStage.setResizable(false);
        primaryStage.setOnShowing(new EventHandler<WindowEvent>() {
            @Override
            public void handle(WindowEvent event) {
                controllerHandle.setConfig(null);
            }
        });
        primaryStage.show();
    }

    @Override
    public void stop() {
        if (controllerHandle.displayExecutorService != null && !controllerHandle.displayExecutorService.isShutdown()) {
            controllerHandle.displayExecutorService.shutdownNow();
        }
        if (controllerHandle.physicsExecutorService != null && !controllerHandle.physicsExecutorService.isShutdown()) {
            controllerHandle.physicsExecutorService.shutdownNow();
        }
        if (controllerHandle.gamePadExecutorService != null && !controllerHandle.gamePadExecutorService.isShutdown()) {
            controllerHandle.gamePadExecutorService.shutdownNow();
        }
        controllerHandle.gamePadHelper.quit();
        controllerHandle.shutDownODE();
    }

    public static VirtualRobotController getControllerHandle(){return controllerHandle;}


    public static void main(String[] args) {
        launch(args);
    }
}
