package se.oru.coordination.coordination_oru.ourproject.models;


public class VehicleThread implements Runnable {

	
	private Vehicle v;
	
	
	public VehicleThread( Vehicle v){
		
		this.v = v;
		}
		

	
	
	public void run() {  //algorithm 1
		int i = 0;
		String List;
			//System.out.println("\n" + "Robot "  + this.v.getID() );
			//System.out.println("\n" + "Category "  + this.v.getCategory() );
			
			
			
		try{
			
			while(i<5){

				/*
				System.out.println("\n" + "R"  + this.v.getID() + " giro "+ i + " posizione : " + this.v.getPose().getX());
				this.v.setPose(this.v.getPose().getX() + this.v.getVelMax() ,this.v.getPose().getY() , 0);
				
				
				List = " ";
				for (Vehicle vh : this.v.getNears()){
					List = (List + vh.getID()+" " );
					}
				System.out.println("\n" + "List R" + this.v.getID() + " : " + List);
				//System.out.println("\n R" + this.v.getID() + " " + this.v.getPose().getX());
				*/	
					
				i++;
				Thread.sleep(v.getTc());
				
			}
		}
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");
		}
		
	}

	
}

