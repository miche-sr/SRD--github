package se.oru.coordination.coordination_oru.ourproject.models;

import static org.junit.Assert.fail;

//import java.lang.reflect.Array;
//import java.sql.Time;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeSet;

import javax.lang.model.util.ElementScanner6;

//import org.graalvm.compiler.phases.verify.VerifyDebugUsage;

import se.oru.coordination.coordination_oru.ourproject.algorithms.CriticalSectionsFounder;
import se.oru.coordination.coordination_oru.ourproject.algorithms.ConstantAccelerationForwardModel.Behavior;

public class VehicleThread implements Runnable {
	
	private CriticalSectionsFounder intersect = new CriticalSectionsFounder();

	private boolean run = true;
	private Vehicle v;
	private double elapsedTrackingTime = 0.0;
	private int slp = -1;
	private int oldCp = -1;
	private TreeSet<CriticalSection> analysedCs = new TreeSet<CriticalSection>(); 
	private ArrayList<Integer> analysedVehiclesI = new ArrayList<Integer>();
	private HashMap<Integer, RobotReport> rrNears = new HashMap<Integer, RobotReport>();
	private Boolean prec = true;
	private CriticalSection csOld;
	private boolean FreeAccess = true;
	private int smStopIndex = -1;
	private String List = " ";


	private static String colorEnv = "#adadad"; //"#f600f6"
	private static String colorTruEnv ="#000000"; //#efe007";
	private static String colorStp = "#047d00"; //"#0008f6"; //"#ffffff";
	private static String colorCrp = "#a30202"; //"#29f600";
	//private static String colorSlp =  "#0008f6";
	private ArrayList<Double> timesCs = new ArrayList<Double>();
	private ArrayList<Double> timesPrec = new ArrayList<Double>();


	public VehicleThread(Vehicle v){
		this.v = v;
	}
	
	/************************** 
	****  MAIN ALGORITHM   ****
	***************************/

	public void run() {
		double start = System.currentTimeMillis();

		try{
			while(v.getForwardModel().getRobotBehavior() != Behavior.reached && run){
				double startTc = System.currentTimeMillis();
				
				//// UNPACK MESSAGES ////
				rrNears.clear();
				List = " ";
				for (Integer vh : v.getNears()){
					rrNears.put(vh,v.getMainTable().get(vh));
					List = (List + vh + " " );
					if (v.checkCollision(vh) ) {
						System.out.println("\u001B[31m" + "ATTENZIONE COLLISIONE  R" + v.getID() +" E R" +vh + "\u001B[0m");
//						printLog(List, prec);
						run = false;
					}
				}

				v.sendNewRr();
//				if (v.getID()==1 || v.getID() == 7) printLog(List, prec);
				
			/****************************
			 * FILTER CRITICAL SECTIONS *
			 ****************************/
			// if a cs has already been found and no one is inside it then I don't recalculate it //
				double startCs = System.currentTimeMillis();
				analysedVehiclesI.clear();
				this.analysedCs.clear();
				for (CriticalSection analysedCs : v.getCs()){
					int v2Id = analysedCs.getVehicle2().getID();
					if (rrNears.containsKey(v2Id)){
						if (v.getPathIndex() < analysedCs.getTe1Start() && rrNears.get(v2Id).getPathIndex() < analysedCs.getTe2Start() // Cs non intrapresa
									&&	analysedCs.getVehicle2().getTruncateTimes().get(analysedCs.getTe2Start()) == rrNears.get(v2Id).getTruncateTimes().get(analysedCs.getTe2Start())
									//&& rrNears.get(v2Id).getFlagCs() != true && v.isCsTooClose() !=true // no rischio deadlock
									&& !analysedCs.isCsTruncated()) { // no cs troncata
							this.analysedCs.add(analysedCs);
							analysedVehiclesI.add( analysedCs.getVehicle2().getID());
						}
					}
				}
				
				//analysedVehiclesI.clear(); //skip filtering
				
				// re-add the cs already analysed and find the cs of other vehicles
				v.clearCs();
				for (CriticalSection analysedCs : this.analysedCs){
					v.getCs().add(analysedCs);
				}
				
				
				for (RobotReport vh : rrNears.values()){
					if (!analysedVehiclesI.contains(vh.getID())) {
						v.appendCs(vh);
					}
				}
				double finishCs = System.currentTimeMillis();
				double timeElapsedCs = (finishCs - startCs);
				if(v.getCs().size() != 0) timesCs.add(timeElapsedCs);

				/**********************************
				 		** SEMAPHORE **
				 ++++++++++++++++++++++++++++++++*/

				 for (TrafficLights TL : v.getTrafficLightsNears()){
					synchronized(TL){
						if (v.getWholeSpatialEnvelope().getPolygon().intersects(TL.getCorridorPath().getPolygon())){
	
							if (!TL.getRobotInsideCorridor().contains(v.getID())){
								if (TL.checkSemophore(v)){ // entro nel corridioi
										TL.addVehicleInside(v);
										FreeAccess = true;
								}
								else{ // semaforo rosso
									if (FreeAccess == true) smStopIndex = intersect.SmStopIndex(v, TL);
									FreeAccess = false;
								}
							}
						}
						if (!v.getSpatialEnvelope().getPolygon().intersects(TL.getCorridorPath().getPolygon())) {
						 	if (FreeAccess == true) {//lo stavo attraversando e sono uscito
								if (TL.getRobotInsideCorridor().contains(v.getID()))	TL.removeVehicleInside(v);
							}
					
						}
						
					}
				}



				
				/*******************************
				 ** CALCULATE THE PRECEDENCES ** 
				 *******************************/
				double startPrec = System.currentTimeMillis();
				prec = true;
				
				v.setCriticalPoint(v.getWholePath().length-1); // ex -1
				csOld = null;
				if (v.getCs().size() <= 1)  v.setCsTooClose(false);
				for (CriticalSection cs : this.v.getCs()){
					
					

					prec =v.ComputePrecedences(cs);
					if (csOld != null) v.setCsTooClose(intersect.csTooClose(v, csOld, cs));
					if (prec == false && v.isCsTooClose() && csOld != null){		//calculate precedence as long as I have precedence
						v.setCriticalPoint(csOld);
						break;
					}
					else if (prec == false){
						v.setCriticalPoint(cs);
						break;
					}
					if (v.isCsTooClose() == false || csOld == null) 
						csOld = cs;
				}

				double endPrec = System.currentTimeMillis();
				double elapsePrec = endPrec  - startPrec;
				if(v.getCs().size() != 0) timesPrec.add(elapsePrec);


				if(FreeAccess== false && v.getStoppingPoint() != -1 && smStopIndex >= 6)
					v.setCriticalPoint( Math.min(v.getCriticalPoint(),smStopIndex-6) );
				
				if (oldCp != v.getCriticalPoint()) {
					v.setSlowingPointNew();
					slp = v.getForwardModel().getPathIndex(v.getWholePath(), v.getSlowingPoint());
					oldCp = v.getCriticalPoint();
				}
				if (v.getForwardModel().getRobotBehavior() == Behavior.stop)
					v.setSlowingPointNew();

				//// UPDATE VALUES ///
				v.setPathIndex(elapsedTrackingTime,FreeAccess);
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());
				v.setStoppingPoint();
				v.setTimes();
				v.setSpatialEnvelope2(FreeAccess,smStopIndex);
//				v.sendNewRr();
				
				/***********************************
				 ****** VISUALIZATION AND PRINT ****
				 ***********************************/
				v.getVisualization().addEnvelope(v.getWholeSpatialEnvelope().getPolygon(),v,colorEnv); 
				v.getVisualization().addEnvelope(v.getSpatialEnvelope().getPolygon(),v,colorTruEnv);
				v.getVisualization().displayPoint(v, v.getCriticalPoint(), colorCrp); //-1 perche array parte da zero
				v.getVisualization().displayPoint(v, v.getStoppingPoint(), colorStp);
				String infoCs = v.getForwardModel().getRobotBehavior().toString();
				v.getVisualization().displayRobotState(v.getSpatialEnvelope().getFootprint(), v,infoCs);

				/// SLEEPING TIME ////
				double finishTc = System.currentTimeMillis();
				double timeElapsedTc = (finishTc - startTc);
				if (timeElapsedTc > v.getTc())
				System.out.println("\u001B[31m"+"R" + this.v.getID() + " - ATTENZIONE Time Elapsed Tc " + "\t" + timeElapsedTc + " > Tc " +v.getTc() + "\u001B[0m");
				
				Thread.sleep(v.getTc());
				this.elapsedTrackingTime += v.getTc()*Vehicle.mill2sec;
			}

			// FINE THREAD
			double finish = System.currentTimeMillis();
			double timeElapsed = (finish - start)/1000;
		
			double sum = 0;
			double tmax = 0;
			int count = 0;
			for(double t : timesCs){
				if ( t != 0.0 ){
				sum = sum + t;
				count = count +1;}
				
				if(t > tmax) tmax = t;
			}
			double media = sum/ count ;

			double sumPrec = 0;
			double tmaxPrec = 0;
			int countPrec = 0;
			for(double p : timesPrec){
				if ( p != 0.0 ){
				sumPrec = sumPrec + p;
				countPrec = countPrec +1;}
				
				if(p > tmaxPrec) tmaxPrec = p;
			}
			double mediaPrec = sumPrec/ countPrec ;


			System.out.println("\u001B[34m"+"R" + this.v.getID() + " " +  String.format("%.3f", timeElapsed)+ 
			" " + String.format("%.2f", media)+ " " + String.format("%.2f", tmax) + " " + String.format("%.2f", mediaPrec)+ " " + String.format("%.2f", tmaxPrec) + "\u001B[0m");
		

		}	
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");
		}	
	}


	/// FUNCTION FOR PRINTING INFORMATIONS ////	
	public void printLog(String List, Boolean prec) {
		String CsString = "";
		String Dist = String.format("%.2f", v.getDistanceTraveled());
		String Vel = String.format("%.2f", v.getVelocity());
		String Slow = String.format("%.2f", v.getSlowingPoint());
		//String infoCs = this.infoCs();
		String infoCs = v.getForwardModel().getRobotBehavior().toString();
		if (v.getCs().size() != 0){
			int i = 0;
			for (CriticalSection cs : v.getCs()) {
				CsString = CsString + (i+1 +"° Sezione Critica" +
				"\t Mia: " + cs.getTe1Start()+"-"+cs.getTe1End() +
				"\t R" + cs.getVehicle2().getID() + ":" +
				"\t" + cs.getTe2Start()+"-"+cs.getTe2End()) + 
				"\t precedenza:" + cs.isPrecedenza() + "\n";
				i += 1;
			}
		}
		else	CsString = ("0 Sezioni Critiche\n");
				
		System.out.println(
			"============================================================\n"+
			"Info R" + this.v.getID() + " : \n" + 
			"Last Index: "+ (v.getWholePath().length-1) + "\t \t " + infoCs + "\n" + 
			"Vicini: "  + List + "\t \t Precedence: " + prec +  "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\t \t Stopping Point: " + v.getStoppingPoint() + "\n" +
			"Distance: "+ Dist  + "\t \t Velocity: " + Vel + "\n" +
			"Slowing Point: " + Slow +"\t Critical Point: " + v.getCriticalPoint()+ "\n" + 
			CsString + "\n" +
			"Traiettoria Attuale \n" +v.getTruncateTimes() + "\n"+
			"============================================================"
			);
	}
	



}