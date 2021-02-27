package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class Test4Grid {

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	private static String yamlFile = null;

	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal, 200, 2, false,yamlFile);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) throws InterruptedException {

		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;

        BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(20, 10, 2);


		Pose start1 = new Pose(1, 11.2, Math.PI); 	Pose[] goal1 = { new Pose(16, 11.2, Math.PI) };	
		Pose start2 = new Pose(1, 1.2, Math.PI); 	Pose[] goal2 = { new Pose(16, 1.2, Math.PI) };
		Pose start3 = new Pose(1, 3.7, Math.PI); 		Pose[] goal3 = { new Pose(16, 3.7, Math.PI) };
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
		Pose start18 = new Pose(5, 14, Math.PI/2); 		Pose[] goal18 = { new Pose(5,-1,  Math.PI/2) };
		Pose start19 = new Pose(7.5, 14, Math.PI/2); 	Pose[] goal19 = { new Pose(7.5,-1,   Math.PI/2) };
		Pose start20 = new Pose(10,14, Math.PI/2); 		Pose[] goal20 = { new Pose(10,-1, Math.PI/2) };

		Pose start21 = new Pose(21, 11.2, Math.PI); 	Pose[] goal21 = { new Pose(36, 11.2, Math.PI) };	
		Pose start22 = new Pose(21, 1.2, Math.PI); 	Pose[] goal22 = { new Pose(36, 1.2, Math.PI) };
		Pose start23 = new Pose(21, 3.7, Math.PI); 		Pose[] goal23 = { new Pose(36, 3.7, Math.PI) };
		Pose start24 = new Pose(21, 6.2, Math.PI); 	Pose[] goal24 = { new Pose(36, 6.2, Math.PI) };
		Pose start25 = new Pose(21, 8.7, Math.PI); 	Pose[] goal25 = { new Pose(36, 8.7, Math.PI) };	

		Pose start26 = new Pose(35, 12.5, 0); 		Pose[] goal26 = { new Pose(20, 12.5, 0) };
		Pose start27 = new Pose(35, 2.5,0) ; 		Pose[] goal27 = { new Pose(20, 2.5, 0) };
		Pose start28 = new Pose(35, 5, 0); 		Pose[] goal28 = { new Pose(20, 5, 0) };
		Pose start29 = new Pose(35, 7.5, 0); 		Pose[] goal29 = { new Pose(20, 7.5, 0) };
		Pose start30 = new Pose(35, 10, 0); 		Pose[] goal30 = { new Pose(20, 10, 0) };

		Pose start31 = new Pose(31.2,0, -Math.PI/2); 	Pose[] goal31 = { new Pose(31.2,15, -Math.PI/2) };
		Pose start32 = new Pose(33.7,0,-Math.PI/2) ; 	Pose[] goal32 = { new Pose(33.7,15, -Math.PI/2) };
		Pose start33 = new Pose(23.7, 0, -Math.PI/2); 	Pose[] goal33 = { new Pose(23.7,15, -Math.PI/2) };
		Pose start34 = new Pose(26.2, 0, -Math.PI/2); 	Pose[] goal34 = { new Pose(26.2,15, -Math.PI/2) };
		Pose start35 = new Pose(28.7,0, -Math.PI/2); 	Pose[] goal35 = { new Pose(28.7,15, -Math.PI/2) };
		
		Pose start36 = new Pose(32.5,14, Math.PI/2); 	Pose[] goal36 = { new Pose(32.5,-1, Math.PI/2) };
		Pose start37 = new Pose(22.5, 14, Math.PI/2) ; 	Pose[] goal37 = { new Pose(22.5,-1,  Math.PI/2) };
		Pose start38 = new Pose(25, 14, Math.PI/2); 		Pose[] goal38 = { new Pose(25,-1,  Math.PI/2) };
		Pose start39 = new Pose(27.5, 14, Math.PI/2); 	Pose[] goal39 = { new Pose(27.5,-1,   Math.PI/2) };
		Pose start40 = new Pose(30,14, Math.PI/2); 		Pose[] goal40 = { new Pose(30,-1, Math.PI/2) };

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
		
		Thread thread21 = initThread(21, c, start21, goal21);
		Thread thread22 = initThread(22, c, start22, goal22);
		Thread thread23 = initThread(23, c, start23, goal23);
		Thread thread24 = initThread(24, c, start24, goal24);
		Thread thread25 = initThread(25, c, start25, goal25);
		Thread thread26 = initThread(26, c, start26, goal26);
		Thread thread27 = initThread(27, c, start27, goal27);
		Thread thread28 = initThread(28, c, start28, goal28);
		Thread thread29 = initThread(29, c, start29, goal29);
		Thread thread30 = initThread(30, c, start30, goal30);
		Thread thread31 = initThread(31, c, start31, goal31);
		Thread thread32 = initThread(32, c, start32, goal32);
		Thread thread33 = initThread(33, c, start33, goal33);
		Thread thread34 = initThread(34, c, start34, goal34);
		Thread thread35 = initThread(35, c, start35, goal35);
		Thread thread36 = initThread(36, c, start36, goal36);
		Thread thread37 = initThread(37, c, start37, goal37);
		Thread thread38 = initThread(38, c, start38, goal38);
		Thread thread39 = initThread(39, c, start39, goal39);
		Thread thread40 = initThread(40, c, start40, goal40);


		Pose start41 = new Pose(1, 31.2, Math.PI); 	Pose[] goal41 = { new Pose(16, 31.2, Math.PI) };	
		Pose start42 = new Pose(1, 21.2, Math.PI); 	Pose[] goal42 = { new Pose(16, 21.2, Math.PI) };
		Pose start43 = new Pose(1, 23.7, Math.PI); 	Pose[] goal43 = { new Pose(16, 23.7, Math.PI) };
		Pose start44 = new Pose(1, 26.2, Math.PI); 	Pose[] goal44 = { new Pose(16, 26.2, Math.PI) };
		Pose start45 = new Pose(1, 28.7, Math.PI); 	Pose[] goal45 = { new Pose(16, 28.7, Math.PI) };	

		Pose start46 = new Pose(15, 32.5, 0); 		Pose[] goal46 = { new Pose(0, 32.5, 0) };
		Pose start47 = new Pose(15, 22.5,0) ; 		Pose[] goal47 = { new Pose(0, 22.5, 0) };
		Pose start48 = new Pose(15, 25, 0); 		Pose[] goal48 = { new Pose(0, 25, 0) };
		Pose start49 = new Pose(15, 27.5, 0); 		Pose[] goal49 = { new Pose(0, 27.5, 0) };
		Pose start50 = new Pose(15, 30, 0); 		Pose[] goal50 = { new Pose(0, 30, 0) };

		Pose start51 = new Pose(11.2,20, -Math.PI/2); 	Pose[] goal51 = { new Pose(11.2,35, -Math.PI/2) };
		Pose start52 = new Pose(13.7,20,-Math.PI/2) ; 	Pose[] goal52 = { new Pose(13.7,35, -Math.PI/2) };
		Pose start53 = new Pose(3.7, 20, -Math.PI/2); 	Pose[] goal53 = { new Pose(3.7,35, -Math.PI/2) };
		Pose start54 = new Pose(6.2, 20, -Math.PI/2); 	Pose[] goal54 = { new Pose(6.2,35, -Math.PI/2) };
		Pose start55 = new Pose(8.7,20, -Math.PI/2); 	Pose[] goal55 = { new Pose(8.7,35, -Math.PI/2) };
		
		Pose start56 = new Pose(12.5,34, Math.PI/2); 	Pose[] goal56 = { new Pose(12.5,19, Math.PI/2) };
		Pose start57 = new Pose(2.5, 34, Math.PI/2) ; 	Pose[] goal57 = { new Pose(2.5,19,  Math.PI/2) };
		Pose start58 = new Pose(5, 34, Math.PI/2); 		Pose[] goal58 = { new Pose(5,19,  Math.PI/2) };
		Pose start59 = new Pose(7.5, 34, Math.PI/2); 	Pose[] goal59 = { new Pose(7.5,19,   Math.PI/2) };
		Pose start60 = new Pose(10,34, Math.PI/2); 		Pose[] goal60 = { new Pose(10,19, Math.PI/2) };

		Pose start61 = new Pose(21, 31.2, Math.PI); 	Pose[] goal61 = { new Pose(36, 31.2, Math.PI) };	
		Pose start62 = new Pose(21, 21.2, Math.PI); 	Pose[] goal62 = { new Pose(36, 21.2, Math.PI) };
		Pose start63 = new Pose(21, 23.7, Math.PI); 	Pose[] goal63 = { new Pose(36, 23.7, Math.PI) };
		Pose start64 = new Pose(21, 26.2, Math.PI); 	Pose[] goal64 = { new Pose(36, 26.2, Math.PI) };
		Pose start65 = new Pose(21, 28.7, Math.PI); 	Pose[] goal65 = { new Pose(36, 28.7, Math.PI) };	

		Pose start66 = new Pose(35, 32.5, 0); 		Pose[] goal66 = { new Pose(20, 32.5, 0) };
		Pose start67 = new Pose(35, 22.5,0) ; 		Pose[] goal67 = { new Pose(20, 22.5, 0) };
		Pose start68 = new Pose(35, 25, 0); 		Pose[] goal68 = { new Pose(20, 25, 0) };
		Pose start69 = new Pose(35, 27.5, 0); 		Pose[] goal69 = { new Pose(20, 27.5, 0) };
		Pose start70 = new Pose(35, 30, 0); 		Pose[] goal70 = { new Pose(20, 30, 0) };

		Pose start71 = new Pose(31.2,20, -Math.PI/2); 	Pose[] goal71 = { new Pose(31.2,35, -Math.PI/2) };
		Pose start72 = new Pose(33.7,20,-Math.PI/2) ; 	Pose[] goal72 = { new Pose(33.7,35, -Math.PI/2) };
		Pose start73 = new Pose(23.7, 20, -Math.PI/2); 	Pose[] goal73 = { new Pose(23.7,35, -Math.PI/2) };
		Pose start74 = new Pose(26.2, 20, -Math.PI/2); 	Pose[] goal74 = { new Pose(26.2,35, -Math.PI/2) };
		Pose start75 = new Pose(28.7,20, -Math.PI/2); 	Pose[] goal75 = { new Pose(28.7,35, -Math.PI/2) };
		
		Pose start76 = new Pose(32.5,34, Math.PI/2); 	Pose[] goal76 = { new Pose(32.5,19, Math.PI/2) };
		Pose start77 = new Pose(22.5, 34, Math.PI/2) ; 	Pose[] goal77 = { new Pose(22.5,19,  Math.PI/2) };
		Pose start78 = new Pose(25, 34, Math.PI/2); 	Pose[] goal78 = { new Pose(25,19,  Math.PI/2) };
		Pose start79 = new Pose(27.5, 34, Math.PI/2); 	Pose[] goal79 = { new Pose(27.5,19,   Math.PI/2) };
		Pose start80 = new Pose(30,34, Math.PI/2); 		Pose[] goal80 = { new Pose(30,19, Math.PI/2) };


		Thread thread41 = initThread(41, c, start41, goal41);
		Thread thread42 = initThread(42, c, start42, goal42);
		Thread thread43 = initThread(43, c, start43, goal43);
		Thread thread44 = initThread(44, c, start44, goal44);
		Thread thread45 = initThread(45, c, start45, goal45);
		Thread thread46 = initThread(46, c, start46, goal46);
		Thread thread47 = initThread(47, c, start47, goal47);
		Thread thread48 = initThread(48, c, start48, goal48);
		Thread thread49 = initThread(49, c, start49, goal49);
		Thread thread50 = initThread(50, c, start50, goal50);
		
		Thread thread51 = initThread(51, c, start51, goal51);
		Thread thread52 = initThread(52, c, start52, goal52);
		Thread thread53 = initThread(53, c, start53, goal53);
		Thread thread54 = initThread(54, c, start54, goal54);
		Thread thread55 = initThread(55, c, start55, goal55);
		Thread thread56 = initThread(56, c, start56, goal56);
		Thread thread57 = initThread(57, c, start57, goal57);
		Thread thread58 = initThread(58, c, start58, goal58);
		Thread thread59 = initThread(59, c, start59, goal59);
		Thread thread60 = initThread(60, c, start60, goal60);
		
		Thread thread61 = initThread(61, c, start61, goal61);
		Thread thread62 = initThread(62, c, start62, goal62);
		Thread thread63 = initThread(63, c, start63, goal63);
		Thread thread64 = initThread(64, c, start64, goal64);
		Thread thread65 = initThread(65, c, start65, goal65);
		Thread thread66 = initThread(66, c, start66, goal66);
		Thread thread67 = initThread(67, c, start67, goal67);
		Thread thread68 = initThread(68, c, start68, goal68);
		Thread thread69 = initThread(69, c, start69, goal69);
		Thread thread70 = initThread(70, c, start70, goal70);
		Thread thread71 = initThread(71, c, start71, goal71);
		Thread thread72 = initThread(72, c, start72, goal72);
		Thread thread73 = initThread(73, c, start73, goal73);
		Thread thread74 = initThread(74, c, start74, goal74);
		Thread thread75 = initThread(75, c, start75, goal75);
		Thread thread76 = initThread(76, c, start76, goal76);
		Thread thread77 = initThread(77, c, start77, goal77);
		Thread thread78 = initThread(78, c, start78, goal78);
		Thread thread79 = initThread(79, c, start79, goal79);
		Thread thread80 = initThread(80, c, start80, goal80);




        
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
	Thread.sleep(1500);
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
	
	thread21.start();
	thread22.start();
	thread23.start();
	thread24.start();
	thread25.start();
	thread26.start();
	thread27.start();
	thread28.start();	
	thread29.start();	
	thread30.start();
	thread31.start();
	thread32.start();	
	thread33.start();	
	thread34.start();
	thread35.start();		
	thread36.start();
	thread37.start();	
	thread38.start();	
	thread39.start();
	thread40.start();


	thread41.start();
	thread42.start();
	thread43.start();
	thread44.start();
	thread45.start();
	thread46.start();
	thread47.start();
	thread48.start();	
	thread49.start();	
	thread50.start();
	thread51.start();
	thread52.start();	
	thread53.start();	
	thread54.start();
	thread55.start();		
	thread56.start();
	thread57.start();	
	thread58.start();	
	thread59.start();
	thread60.start();
	
	thread61.start();
	thread62.start();
	thread63.start();
	thread64.start();
	thread65.start();
	thread66.start();
	thread67.start();
	thread68.start();	
	thread69.start();	
	thread70.start();
	thread71.start();
	thread72.start();	
	thread73.start();	
	thread74.start();
	thread75.start();		
	thread76.start();
	thread77.start();	
	thread78.start();	
	thread79.start();
	thread80.start();

	}
}
