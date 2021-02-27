package se.oru.coordination.coordination_oru.util;

import java.awt.Desktop;
import java.awt.Dimension;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import javax.imageio.ImageIO;

import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.servlet.ServletContextHandler;
import org.eclipse.jetty.servlet.ServletHolder;
import org.eclipse.jetty.websocket.api.RemoteEndpoint;
import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.geom.util.AffineTransformation;
import com.vividsolutions.jts.util.GeometricShapeFactory;

import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle;

public class BrowserVisualizationDist implements FleetVisualization {
	
	private ArrayList<String> msgQueue = new ArrayList<String>();
	private static int UPDATE_PERIOD = 30;
	private double robotFootprintArea = -1;
	private double robotFootprintXDim = -1;
	private String overlayText = null;

	public BrowserVisualizationDist() {
		this("localhost", 30);
	}

	public BrowserVisualizationDist(String serverHostNameOrIP) {
		this(serverHostNameOrIP, 30);
	}

	public BrowserVisualizationDist(int updatePeriodInMillis) {
		this("localhost", updatePeriodInMillis);
	}
	
	public BrowserVisualizationDist(String serverHostNameOrIP, int updatePeriodInMillis) {
		UPDATE_PERIOD = updatePeriodInMillis;
		BrowserVisualizationDist.setupVizMessageServer();
        Thread updateThread = new Thread("Visualization update thread") {
        	public void run() {
        		while (true) {
        			sendMessages();
        			try { Thread.sleep(UPDATE_PERIOD); }
        			catch (InterruptedException e) { e.printStackTrace(); }
        		}
        	}
        };
        updateThread.start();
        BrowserVisualizationDist.setupVizServer(serverHostNameOrIP);
        startOpenInBrowser(serverHostNameOrIP);
	}
	
	private void startOpenInBrowser(String serverHostNameOrIP) {
		if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
			try { Desktop.getDesktop().browse(new URI("http://" + serverHostNameOrIP + ":8080")); }
			catch (IOException e) { e.printStackTrace(); }
			catch (URISyntaxException e) { e.printStackTrace(); }
		}
	}
	
	private void updateOverlayText() {
		if (this.overlayText != null) {
			String jsonString = "{ \"operation\" : \"setOverlayText\","
					+ "\"data\" : "
					+ "{ \"text\" : \""+ this.overlayText + "\" }}";
			sendMessage(jsonString);
		}
	}
	
	public void setOverlayText(String text) {
		this.overlayText = text;
	}

	private void updateRobotFootprintArea(Geometry geom) {
		if (robotFootprintArea == -1) {
			robotFootprintArea = geom.getArea();
			double minX = Double.MAX_VALUE;
			double maxX = Double.MIN_VALUE;
			for (Coordinate coord : geom.getCoordinates()) {
				if (coord.x < minX) minX = coord.x;
				if (coord.x > maxX) maxX = coord.x;
			}
			this.robotFootprintXDim = maxX-minX;
		}
	}
	
	public void setInitialTransform(double scale, double xTrans, double yTrans) {
		BrowserVisualizationSocket.initialScale = scale;
		BrowserVisualizationSocket.initialTranslation = new Coordinate(xTrans,yTrans);		
	}
	
	private static int getScreenDPI() {
		//Dimension screen = java.awt.Toolkit.getDefaultToolkit().getScreenSize();
		//System.out.println("Screen width: "+screen.getWidth()); 
		//System.out.println("Screen height: "+screen.getHeight()); 
		int pixelPerInch = java.awt.Toolkit.getDefaultToolkit().getScreenResolution(); 
		//System.out.println("DPI: " + pixelPerInch); 
		return pixelPerInch; 
	}
	
	private static double getScreenHeight() {
		Dimension screen = java.awt.Toolkit.getDefaultToolkit().getScreenSize();
		return screen.getHeight(); 
	}
	
	public void guessInitialTransform(double robotDimension, Pose ... robotPoses) {
		BrowserVisualizationSocket.initialScale = getScreenDPI()/robotDimension;
		double avgX = 0;
		double avgY = 0;
		for (int i = 0; i < robotPoses.length; i++) {
			avgX += robotPoses[i].getX();
			avgY += robotPoses[i].getY();
		}
		avgX /= robotPoses.length;
		avgY /= robotPoses.length;
		avgY -= 0.45*(getScreenHeight()/getScreenDPI());
		BrowserVisualizationSocket.initialTranslation = new Coordinate(avgX,avgY);		
	}

	private static void setupVizServer(String serverHostNameOrIP) {
		Server server = new Server(8080);
		server.setHandler(new BrowserVisualizationServer(serverHostNameOrIP));
		try {
			server.start();
			//server.join();
		}
        catch (Throwable t) { t.printStackTrace(System.err); }
	}
	
	private static void setupVizMessageServer() {
        Server server = new Server();
        ServerConnector connector = new ServerConnector(server);
        connector.setPort(8081);
        server.addConnector(connector);

        // Setup the basic application "context" for this application at "/"
        // This is also known as the handler tree (in jetty speak)
        ServletContextHandler context = new ServletContextHandler(ServletContextHandler.SESSIONS);
        context.setContextPath("/");
        server.setHandler(context);
        
        // Add a websocket to a specific path spec
        ServletHolder holderEvents = new ServletHolder("ws-events", BrowserVisualizationServlet.class);
        context.addServlet(holderEvents, "/fleet-events/*");
        
        try {
            server.start();
            server.dump(System.err);
            //server.join();
        }
        catch (Throwable t) { t.printStackTrace(System.err); }		
	}
	
	private void enqueueMessage(String message) {
		if (BrowserVisualizationSocket.ENDPOINTS != null && BrowserVisualizationSocket.ENDPOINTS.size() > 0) {
			synchronized (BrowserVisualizationSocket.ENDPOINTS) {
				this.msgQueue.add(message);
			}
		}
	}
	
	private void sendMessages() {
		if (BrowserVisualizationSocket.ENDPOINTS != null && BrowserVisualizationSocket.ENDPOINTS.size() > 0) {
			synchronized (BrowserVisualizationSocket.ENDPOINTS) {
				for (String message : this.msgQueue) {
					sendMessage(message);
				}
				msgQueue.clear();
				updateOverlayText();
				sendUpdate();
			}
		}
	}
	
	private void sendMessage(String text) {
		if (BrowserVisualizationSocket.ENDPOINTS != null) {
			for (RemoteEndpoint rep : BrowserVisualizationSocket.ENDPOINTS) {
				try {
					rep.sendString(text);
				}
				catch(IOException e) { e.printStackTrace(); }
			}
		}
	}

	@Override
	public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String... extraStatusInfo) {
		double x = rr.getPathIndex() != -1 ? rr.getPose().getX() : te.getTrajectory().getPose()[0].getX();
		double y = rr.getPathIndex() != -1 ? rr.getPose().getY() : te.getTrajectory().getPose()[0].getY();
		double theta = rr.getPathIndex() != -1 ? rr.getPose().getTheta() : te.getTrajectory().getPose()[0].getTheta();
		
		String name = "R"+te.getRobotID();
		String extraData = " : " + rr.getPathIndex();
		if (extraStatusInfo != null) {
			for (String st : extraStatusInfo) {
				extraData += (" | " + st);
			}
		}
		
		Geometry geom = TrajectoryEnvelope.getFootprint(te.getFootprint(), x, y, theta);
		this.updateRobotFootprintArea(geom);
		double scale = Math.sqrt(robotFootprintArea)*0.2;
		Geometry arrowGeom = createArrow(rr.getPose(), robotFootprintXDim/scale, scale);
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(name, geom, "#ff0000", -1, true, extraData) + "}";
		String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+name, arrowGeom, "#ffffff", -1, true, null) + "}";
		enqueueMessage(jsonString);
		enqueueMessage(jsonStringArrow);
	}
	
	@Override
	public void displayRobotState(Polygon fp, RobotReport rr, String... extraStatusInfo) {
		double x = rr.getPose().getX();
		double y = rr.getPose().getY();
		double theta = rr.getPose().getTheta();
		
		String name = "R"+rr.getRobotID();
		String extraData = " : " + rr.getPathIndex();
		if (extraStatusInfo != null) {
			for (String st : extraStatusInfo) {
				extraData += (" | " + st);
			}
		}
		
		Geometry geom = TrajectoryEnvelope.getFootprint(fp, x, y, theta);
		this.updateRobotFootprintArea(geom);
		double scale = Math.sqrt(robotFootprintArea)*0.2;
		Geometry arrowGeom = createArrow(rr.getPose(), robotFootprintXDim/scale, scale);
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(name, geom, "#ff0000", -1, true, extraData) + "}";
		String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+name, arrowGeom, "#ffffff", -1, true, null) + "}";
		enqueueMessage(jsonString);
		enqueueMessage(jsonStringArrow);
	}



	@Override
	public void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor) {
		Geometry arrow = createArrow(rrWaiting.getPose(), rrDriving.getPose());
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(dependencyDescriptor, arrow, "#adccff", 1000, true, null) + "}";
		enqueueMessage(jsonString);
	}
	
	private String geometryToJSONString(String name, Geometry geom, String color, long age, boolean filled, String extraData) {
		String ret = "{ \"name\" : \"" + name + "\", \"color\" : \"" + color + "\", ";
		if (age > 0) ret += " \"age\" : " + age + ", ";
		ret += " \"filled\" : " + filled + ", ";
		if (extraData != null && !extraData.trim().equals("")) ret += " \"extraData\" : \"" + extraData + "\", ";		
		ret += "\"coordinates\" : [";
		Coordinate[] coords = geom.getCoordinates();
		for (int i = 0; i < coords.length; i++) {
			ret += "{\"x\" : " + coords[i].x + ", \"y\" : " + coords[i].y + "}";
			if (i < coords.length-1) ret += ", ";
		}
		ret += "]}";
		return ret;
	}

	@Override
	public void addEnvelope(TrajectoryEnvelope te) {
		GeometricShapeDomain dom = (GeometricShapeDomain)te.getEnvelopeVariable().getDomain();
		Geometry geom = dom.getGeometry();
		String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+te.getID(), geom, "#efe007", -1, false, null) + "}";
		enqueueMessage(jsonString);
	}



	@Override
	public void removeEnvelope(TrajectoryEnvelope te) {
		String jsonString = "{ \"operation\" : \"removeGeometry\","
				+ "\"data\" : "
				+ "{ \"name\" : \""+ "_"+te.getID() +"\" }}";
		enqueueMessage(jsonString);
	}

	@Override
	public void updateVisualization() {
		// This method does nothing - reason:
		// Viz change events are buffered and sent by an internal thread
		// in bursts every UPDATE_PERIOD ms to avoid blocking of RemoteEndpopints
	}
	
	public void sendUpdate() {
		String callRefresh = "{ \"operation\" : \"refresh\" }";
		sendMessage(callRefresh);
	}
	
	
	private Geometry createArrow(Pose pose) {
		return createArrow(pose, Math.sqrt(robotFootprintArea)*0.5, Math.sqrt(robotFootprintArea)*0.5);
		
	}
	private Geometry createArrow(Pose pose, double length, double size) {		
		GeometryFactory gf = new GeometryFactory();
		double aux = 1.8;
		double aux1 = 0.8;
		double aux2 = 0.3;
		double theta = pose.getTheta();
		Coordinate[] coords = new Coordinate[8];
		coords[0] = new Coordinate(0.0,-aux2);
		coords[1] = new Coordinate(length-aux,-aux2);
		coords[2] = new Coordinate(length-aux,-aux1);
		coords[3] = new Coordinate(length,0.0);
		coords[4] = new Coordinate(length-aux,aux1);
		coords[5] = new Coordinate(length-aux,aux2);
		coords[6] = new Coordinate(0.0,aux2);
		coords[7] = new Coordinate(0.0,-aux2);
		Polygon arrow = gf.createPolygon(coords);
		AffineTransformation at = new AffineTransformation();
		at.scale(size, size);
		at.rotate(theta);
		at.translate(pose.getX(), pose.getY());
		Geometry ret = at.transform(arrow);
		return ret;
	}
	
	private Geometry createArrow(Pose pose1, Pose pose2) {		
		GeometryFactory gf = new GeometryFactory();
		double aux = 1.8;
		double aux1 = 0.8;
		double aux2 = 0.3;
		double factor = Math.sqrt(robotFootprintArea)*0.3;
		double distance = Math.sqrt(Math.pow((pose2.getX()-pose1.getX()),2)+Math.pow((pose2.getY()-pose1.getY()),2))/factor;
		double theta = Math.atan2(pose2.getY() - pose1.getY(), pose2.getX() - pose1.getX());
		Coordinate[] coords = new Coordinate[8];
		coords[0] = new Coordinate(0.0,-aux2);
		coords[1] = new Coordinate(distance-aux,-aux2);
		coords[2] = new Coordinate(distance-aux,-aux1);
		coords[3] = new Coordinate(distance,0.0);
		coords[4] = new Coordinate(distance-aux,aux1);
		coords[5] = new Coordinate(distance-aux,aux2);
		coords[6] = new Coordinate(0.0,aux2);
		coords[7] = new Coordinate(0.0,-aux2);
		Polygon arrow = gf.createPolygon(coords);
		AffineTransformation at = new AffineTransformation();
		at.scale(factor, factor);
		at.rotate(theta);
		at.translate(pose1.getX(), pose1.getY());
		Geometry ret = at.transform(arrow);
		return ret;
	}


	public void setMap(String mapYAMLFile) {
	try {
		File file = new File(mapYAMLFile);
		BufferedReader br = new BufferedReader(new FileReader(file));
		String imageFileName = null;
		String st;
		
		//Coordinate bottomLeftOrigin = null;
		while((st=br.readLine()) != null){ 
			if (!st.trim().startsWith("#") && !st.trim().isEmpty()) {
				String key = st.substring(0, st.indexOf(":")).trim();
				String value = st.substring(st.indexOf(":")+1).trim();
				if (key.equals("image")) imageFileName = file.getParentFile()+File.separator+value;
				else if (key.equals("resolution")) BrowserVisualizationSocket.resolution = Double.parseDouble(value);
				else if (key.equals("origin")) {
					String x = value.substring(1, value.indexOf(",")).trim();
					String y = value.substring(value.indexOf(",")+1, value.indexOf(",", value.indexOf(",")+1)).trim();
					BrowserVisualizationSocket.origin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
					//bottomLeftOrigin = new Coordinate(Double.parseDouble(x),Double.parseDouble(y));
				}
			}
		}
		br.close();
		BrowserVisualizationSocket.map = ImageIO.read(new File(imageFileName));
		//BrowserVisualizationSocket.origin = new Coordinate(bottomLeftOrigin.x, BrowserVisualizationSocket.map.getHeight()*BrowserVisualizationSocket.resolution-bottomLeftOrigin.y);
	}
	catch (IOException e) { e.printStackTrace(); }
}

@Override
public int periodicEnvelopeRefreshInMillis() {
	return 1000;
}

/************************************ 
*** Methods for distributed fleet ***
*************************************/
public void displayRealRobotState(Polygon fp, Vehicle v,int pathIndex, PoseSteering[] path,String... extraStatusInfo) {
	double x = path[pathIndex].getPose().getX();
	double y = path[pathIndex].getPose().getY();
	double theta = path[pathIndex].getPose().getTheta();
	
	String name = "R"+v.getRobotID();
	String extraData = " " ; 
	if (extraStatusInfo != null) {
		for (String st : extraStatusInfo) {
			extraData += (" | " + st);
		}
	}
	Geometry geom = TrajectoryEnvelope.getFootprint(fp, x, y, theta);
	this.updateRobotFootprintArea(geom);
	String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(name, geom, "#ff0000d2", -1, true, extraData) + "}";
	enqueueMessage(jsonString);
}


public void displayRobotState(Polygon fp, Vehicle v,String... extraStatusInfo) {
	double x = v.getSpatialEnvelope().getPath()[0].getPose().getX();
	double y = v.getSpatialEnvelope().getPath()[0].getPose().getY();
	double theta = v.getSpatialEnvelope().getPath()[0].getPose().getTheta();
	
	String name = "R"+v.getRobotID();
	String extraData = " " ; //"" : " + v.getPathIndex();
	if (extraStatusInfo != null) {
		for (String st : extraStatusInfo) {
			extraData += (" | " + st);
		}
	}
	
	Geometry geom = TrajectoryEnvelope.getFootprint(fp, x, y, theta);
	this.updateRobotFootprintArea(geom);
	double scale = Math.sqrt(robotFootprintArea)*0.2;
	//Geometry arrowGeom = createArrow(v.getSpatialEnvelope().getPath()[0].getPose(), robotFootprintXDim/scale, scale);
	String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(name, geom, "#e38922", -1, true, extraData) + "}";
	//String jsonStringArrow = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+name, arrowGeom, "#ffffff", -1, true, null) + "}";
	enqueueMessage(jsonString);
	//enqueueMessage(jsonStringArrow);
	String color;
	if(v.getNears().size()==0) color = "#047d00";
	else color = "#9b0ec9";

	// Geometry circleGeom = createCircle(x, y, 2*v.getRadius());
	// String jsonStringCircle = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+name, circleGeom, color, -1, false, extraData) + "}";
	// enqueueMessage(jsonStringCircle);
}

public void displayDependency(Vehicle vWaiting, se.oru.coordination.coordination_oru.distributed.models.RobotReport vDriving, String dependencyDescriptor) {
	Geometry arrow = createArrow(vWaiting.getSpatialEnvelope().getPath()[0].getPose(), vDriving.getSpatialEnvelope().getPath()[0].getPose());
	String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString(dependencyDescriptor, arrow, "#adccff", 1000, true, null) + "}";
	enqueueMessage(jsonString);
}

public void addEnvelope(Geometry geom, Vehicle v, String color) {
	// "#efe007"
	String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+v.getID()+color, geom, color, -1, false, null) + "}";
	enqueueMessage(jsonString);
}

private Geometry createPoint(Vehicle v, int pathIndex) {		
	GeometryFactory gf = new GeometryFactory();
	double theta = v.getWholePath()[pathIndex].getTheta();
	double x =  v.getWholePath()[pathIndex].getX();
	double y =  v.getWholePath()[pathIndex].getY();
	
	

	double size = Math.sqrt(v.getWholeSpatialEnvelope().getFootprint().getArea());

	Coordinate[] coords = new Coordinate[5];
	coords[0] = new Coordinate(-size/2,-0.5);
	coords[1] = new Coordinate(-size/2+0.1,-0.5);
	coords[2] = new Coordinate(-size/2+0.1,0.5);
	coords[3] = new Coordinate(-size/2,0.5);
	coords[4] = new Coordinate(-size/2,-0.5);

	Polygon point = gf.createPolygon(coords);
	AffineTransformation at = new AffineTransformation();
	at.scale(size, size);
	at.rotate(theta);
	at.translate(x, y);
	Geometry ret = at.transform(point);
	return ret;
}

public void displayPoint(Vehicle v, int pathIndex, String color) {
	Geometry point = createPoint(v,pathIndex);
	String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_"+v.getID()+"point"+color , point,color , -1, true, null) + "}";
	enqueueMessage(jsonString);
}

public void addCorridor(Geometry geom, int ID) {
	String color = "#0008f6"; //"#ffffff";
	String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_S"+ID, geom, color, -1, false, null) + "}";
	enqueueMessage(jsonString);
}

private Geometry createLight(Pose semaphore,int ID, int semaphoreNumber) {		
	GeometryFactory gf = new GeometryFactory();
	double theta = semaphore.getTheta();
	double x =  semaphore.getX();
	double y =  semaphore.getY();
	
	Coordinate[] coords = new Coordinate[5];
	coords[0] = new Coordinate(0.0,0.0);
	coords[1] = new Coordinate(0.4,0.0);
	coords[2] = new Coordinate(0.4,0.4);
	coords[3] = new Coordinate(0.0,0.4);
	coords[4] = new Coordinate(0.0,0.0);

	Polygon point = gf.createPolygon(coords);
	AffineTransformation at = new AffineTransformation();
	if (semaphoreNumber == 1) x=x-0.5;
	else if (semaphoreNumber == 2) x=x+0.5;

	at.translate(x, y-0.6);
	//at.rotate(theta);
	Geometry ret = at.transform(point);
	return ret;
}

public void displayLight(Pose semaphore,int ID, int semaphoreNumber,Boolean sAccess) {
	String color;
	if(sAccess) color = "#66ff00";
	else color = "#ff0000";
	Geometry point = createLight(semaphore,ID,semaphoreNumber);
	String jsonString = "{ \"operation\" : \"addGeometry\", \"data\" : " + this.geometryToJSONString("_Sl"+ID+semaphoreNumber, point,color , -1, true, null) + "}";
	enqueueMessage(jsonString);
}

public static Geometry createCircle(double x, double y, double RADIUS) {
	GeometricShapeFactory shapeFactory = new GeometricShapeFactory();
	shapeFactory.setNumPoints(64);
	shapeFactory.setCentre(new Coordinate(x, y));
	shapeFactory.setSize(RADIUS * 2);
	return shapeFactory.createCircle();
	}
	

}
