package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TestGrid {

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	private static String yamlFile = null;

	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal, 50, 3, false,yamlFile);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) throws InterruptedException {

		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;

		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(35, 12, 5);

		Pose start1 = new Pose(1, 11.2, Math.PI); 	Pose[] goal1 = { new Pose(16, 11.2, Math.PI) };	
		Pose start2 = new Pose(1, 1.2, Math.PI); 	Pose[] goal2 = { new Pose(16, 1.2, Math.PI) };
		Pose start3 = new Pose(1, 3.7, Math.PI); 	Pose[] goal3 = { new Pose(16, 3.7, Math.PI) };
		Pose start4 = new Pose(1, 6.2, Math.PI); 	Pose[] goal4 = { new Pose(16, 6.2, Math.PI) };
		Pose start5 = new Pose(1, 8.7, Math.PI); 	Pose[] goal5 = { new Pose(16, 8.7, Math.PI) };	

		Pose start6 = new Pose(15, 12.5, 0); 		Pose[] goal6 = { new Pose(0, 12.5, 0) };
		Pose start7 = new Pose(15, 2.5,0) ; 		Pose[] goal7 = { new Pose(0, 2.5, 0) };
		Pose start8 = new Pose(15, 5, 0); 		Pose[] goal8 = { new Pose(0, 5, 0) };
		Pose start9 = new Pose(15, 7.5, 0); 		Pose[] goal9 = { new Pose(0, 7.5, 0) };
		Pose start10 = new Pose(15, 10, 0); 		Pose[] goal10 = { new Pose(0, 10, 0) };

		Pose start11 = new Pose(11.2,0, -Math.PI/2); 	Pose[] goal11 = { new Pose(11.2,15, -Math.PI/2) };
		Pose start12 = new Pose(13.7,0,-Math.PI/2) ; 	Pose[] goal12 = { new Pose(13.7,15, -Math.PI/2) };
		Pose start13 = new Pose(3.7, 0, -Math.PI/2); 	Pose[] goal13 = { new Pose(3.7,15, -Math.PI/2) };
		Pose start14 = new Pose(6.2, 0, -Math.PI/2); 	Pose[] goal14 = { new Pose(6.2,15, -Math.PI/2) };
		Pose start15 = new Pose(8.7,0, -Math.PI/2); 	Pose[] goal15 = { new Pose(8.7,15, -Math.PI/2) };
		
		Pose start16 = new Pose(12.5,14, Math.PI/2); 	Pose[] goal16 = { new Pose(12.5,-1, Math.PI/2) };
		Pose start17 = new Pose(2.5, 14, Math.PI/2) ; 	Pose[] goal17 = { new Pose(2.5,-1,  Math.PI/2) };
		Pose start18 = new Pose(5, 14, Math.PI/2); 	Pose[] goal18 = { new Pose(5,-1,  Math.PI/2) };
		Pose start19 = new Pose(7.5, 14, Math.PI/2); 	Pose[] goal19 = { new Pose(7.5,-1,   Math.PI/2) };
		Pose start20 = new Pose(10,14, Math.PI/2); 	Pose[] goal20 = { new Pose(10,-1, Math.PI/2) };

		Thread thread1 = initThread(1, c, start1, goal1);
		Thread thread2 = initThread(2, c, start2, goal2);
		Thread thread3 = initThread(3, c, start3, goal3);
		Thread thread4 = initThread(4, c, start4, goal4);
		Thread thread5 = initThread(5, c, start5, goal5);
		Thread thread6 = initThread(6, c, start6, goal6);
		Thread thread7 = initThread(7, c, start7, goal7);
		Thread thread8 = initThread(8, c, start8, goal8);
		Thread thread9 = initThread(9, c, start9, goal9);
		Thread thread10 = initThread(10, c, start10, goal10);
		Thread thread11 = initThread(11, c, start11, goal11);
		Thread thread12 = initThread(12, c, start12, goal12);
		Thread thread13 = initThread(13, c, start13, goal13);
		Thread thread14 = initThread(14, c, start14, goal14);
		Thread thread15 = initThread(15, c, start15, goal15);
		Thread thread16 = initThread(16, c, start16, goal16);
		Thread thread17 = initThread(17, c, start17, goal17);
		Thread thread18 = initThread(18, c, start18, goal18);
		Thread thread19 = initThread(19, c, start19, goal19);
		Thread thread20 = initThread(20, c, start20, goal20);
	

		
		//viz.setInitialTransform(15, -10, 5);
		try {
			TimeUnit.SECONDS.sleep(5);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	
	double rMax = -1; double tMax = -1;
	for (Vehicle vh : vehicleList){
		double r = vh.getRadius();
		if (r > rMax) {
			rMax = r;
			tMax = r/vh.getVelMax();
		}
	}
	for (Vehicle vh : vehicleList){
		vh.Init(rMax, tMax, vehicleList, mainTable, viz);
		vh.setReplan(false);
		
	}
	//System.out.println("\n" + "Radius "  + rMax );
	Thread.sleep(2000);

	System.out.println("\n" + "Radius "  + rMax );
	int all = 0;
	for (Vehicle vh : vehicleList){
		all = all + vh.countAllcs();
	}
	System.out.println("\n" + "All CS "  + all/2 );
	thread1.start();
	thread2.start();
	thread3.start();
	thread4.start();
	thread5.start();
	thread6.start();
	thread7.start();
	thread8.start();	
	thread9.start();	
	thread10.start();
	thread11.start();
	thread12.start();	
	thread13.start();	
	thread14.start();
	thread15.start();		
	thread16.start();
	thread17.start();	
	thread18.start();	
	thread19.start();
	thread20.start();			
	}
}
