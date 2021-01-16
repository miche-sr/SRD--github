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
	private int sp = -1;
	private int oldCp = -1;
	private TreeSet<CriticalSection> analysedCs = new TreeSet<CriticalSection>(); 
	private ArrayList<RobotReport> analysedVehicles = new ArrayList<RobotReport>();
	private HashMap<Integer, RobotReport> rrNears = new HashMap<Integer, RobotReport>();
//	private ArrayList<RobotReport> rrNears = new ArrayList<RobotReport>();
	private Boolean prec = true;
	private CriticalSection csOld;
	private boolean FreeAccess = true;
	private int smStopIndex = -1;
	private String List = " ";

	public VehicleThread(Vehicle v){
		this.v = v;
	}
	
	/************************** 
	****  MAIN ALGORITHM   ****
	***************************/

	public void run() {
		

		try{
			while(v.getForwardModel().getRobotBehavior() != Behavior.reached && run){
				
				//// UNPACK MESSAGES ////
				rrNears.clear();
				List = " ";
				for (Vehicle vh : v.getNears()){
					rrNears.put(vh.getID(),v.getMainTable().get(vh.getID()));
					List = (List + vh.getID() + " " );
					if (v.checkCollision(vh) ) {
						System.out.println("\u001B[31m" + "ATTENZIONE COLLISIONE  R" + v.getID() +" E R" +vh.getID()+ "\u001B[0m");
						printLog(List, prec);
						run = false;
					}
				}
				
			/****************************
			 * FILTER CRITICAL SECTIONS *
			 ****************************/
			// if a cs has already been found and no one is inside it then I don't recalculate it //
				analysedVehicles.clear();
				this.analysedCs.clear();
				for (CriticalSection analysedCs : v.getCs()){
					int v2Id = analysedCs.getVehicle2().getID();
					if (rrNears.containsKey(v2Id)){
						if (v.getPathIndex() < analysedCs.getTe1Start() 
								&& rrNears.get(v2Id).getPathIndex() < analysedCs.getTe2Start()
								&& rrNears.get(v2Id).getFlagCs() != true && v.isCsTooClose() !=true
									&& !analysedCs.isCsTruncated()) {
							this.analysedCs.add(analysedCs);
							analysedVehicles.add(analysedCs.getVehicle2());
						}
					}
				}
				
				
				// re-add the cs already analysed and find the cs of other vehicles
				v.clearCs();
				for (CriticalSection analysedCs : this.analysedCs)
					v.getCs().add(analysedCs);
				
				
				for (RobotReport vh : rrNears.values()){
					if (!analysedVehicles.contains(vh)) {
						v.appendCs(vh);
					}
				}
				if(run == false)
				System.out.println("\u001B[31m" + "R" + v.getID() +"Cs" +v.getCs().toString()+ "\u001B[0m");

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
					csOld = cs;
				}

				if(FreeAccess== false && v.getStoppingPoint() != -1 && smStopIndex >= 6)
					v.setCriticalPoint( Math.min(v.getCriticalPoint(),smStopIndex-6) );
				
				if (oldCp != v.getCriticalPoint()) {
					v.setSlowingPointNew();
					sp = v.getForwardModel().getPathIndex(v.getWholePath(), v.getSlowingPoint());
					oldCp = v.getCriticalPoint();
				}
				

				//// UPDATE VALUES ///
				v.setPathIndex(elapsedTrackingTime);
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());
				v.setStoppingPoint();
				
				v.setTimes();
				v.setSpatialEnvelope();

				//// SEND NEW ROBOT REPORT ////
				
				
				//printLog(List, prec);
				v.sendNewRr();

				
				/***********************************
				 ****** VISUALIZATION AND PRINT ****
				 ***********************************/

				
				v.getVisualization().addEnvelope(v.getWholeSpatialEnvelope().getPolygon(),v,"#f600f6");
				v.getVisualization().addEnvelope(v.getSpatialEnvelope().getPolygon(),v,"#efe007");
				v.getVisualization().displayPoint(v, v.getCriticalPoint(), "#29f600"); //-1 perche array parte da zero
				v.getVisualization().displayPoint(v, sp, "#0008f6");
				v.getVisualization().displayPoint(v, v.getStoppingPoint(), "#ffffff");

				String infoCs = v.getForwardModel().getRobotBehavior().toString();
				v.getVisualization().displayRobotState(v.getSpatialEnvelope().getFootprint(), v,infoCs);

				/// SLEEPING TIME ////
				Thread.sleep(v.getTc());
				this.elapsedTrackingTime += v.getTc()*Vehicle.mill2sec;
			}
			System.out.println("\u001B[34m"+"\n R" + this.v.getID() + " : GOAL RAGGIUNTO" +"\u001B[0m");
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