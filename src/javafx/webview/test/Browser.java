package javafx.webview.test;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.HPos;
import javafx.geometry.VPos;
import javafx.scene.control.Hyperlink;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Region;
import javafx.scene.web.WebEngine;
import javafx.scene.web.WebView;

public class Browser  extends Region {
	
	private WebView web = new WebView();
	private WebEngine engine = web.getEngine();
	
	private String[] imageFiles = new String[] {
			"oracle.png", "google.png", "microsoft.png", "amdocs.png"
	};
	private String[] captions = new String[]{"oracle.com", "google.com", "microsoft.com", "amdocs.com"};
	private String[] urls = {"http://oracle.com", "http://google.com", "http://microsoft.com", "http://amdocs.com"	
	};
	private Hyperlink[] hpls = new Hyperlink[4];
	
	private HBox toolBar;
	
	public Browser() {
		getStyleClass().add("browser");
		for(int i=0; i<4; i++) {
			hpls[i] = new Hyperlink(captions[i]);
			String url = urls[i];
			hpls[i].setOnAction(new EventHandler<ActionEvent>() {
				@Override
				public void handle(ActionEvent event) {
					engine.load(url);
					
				}
			});
		}
		
		toolBar = new HBox();
		toolBar.getChildren().addAll(hpls);
		toolBar.getStyleClass().add("browser-toolbar");
		engine.load("http://google.com");
		getChildren().add(toolBar);
		getChildren().add(web);
	}
	
	@Override protected void layoutChildren() {
        double w = getWidth();
        double h = getHeight();
        double tbHeight = toolBar.prefHeight(w);
        layoutInArea(web,0,0,w,h-tbHeight,0, HPos.CENTER, VPos.CENTER);
        layoutInArea(toolBar,0,h-tbHeight,w,tbHeight,0,HPos.CENTER,VPos.CENTER);
    }

}
