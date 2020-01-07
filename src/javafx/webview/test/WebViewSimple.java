package javafx.webview.test;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

public class WebViewSimple  extends Application {

	public static void main(String[] args) {
		launch(args);

	}

	@Override
	public void start(Stage primaryStage) throws Exception {
		Scene primary  = new Scene(new Browser(), 750,500, Color.web("#666970"));
		primaryStage.setTitle("Web view sample");
		primaryStage.setScene(primary);
		primaryStage.show();
		
	}

}
