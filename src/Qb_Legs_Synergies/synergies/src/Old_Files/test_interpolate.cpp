


 interp_Instant=std::round(Instant*Ts_old/pub_Ts); 
     if(flag_pub_traj){ 
       
		  if(speed==Current_Speed){
		    Output_Traj=Old_Traj_pub;
		    pub_num=Old_Traj_pub.rows();
		  }else{ // else if speed!= Current_Speed
		    //When data are ready
			one_time=false; //new speed variations are disabled
	    
		      if(flag_concatenate_started){ 
		  
			    if(index>interp_Instant){
	      // 		      std::cout<<"too_soon"<<std::endl;
				Output_Traj=Old_Traj_pub;
				pub_num=Old_Traj_pub.rows();
			    //need to wait the next step, i.e. continue with current Output_Traj
			    }else{ 	   
				  if(index==interp_Instant){
// 				      std::cout<<"Index==interp_Instant"<<std::endl;
				      pub_num=Middle_Traj_pub.rows();
				      Output_Traj=Middle_Traj_pub;
				      flag_concatenate_started=false;
				      flag_concatenate_completed=false;
// 				      std::cout<<"pub num "<<pub_num<<std::endl;
				  }else{
				      Output_Traj=Old_Traj_pub;
				      pub_num=Old_Traj_pub.rows();
			      }
			    }
		      }
		  }
	    
	    if(flag_concatenate_started==false &&  flag_concatenate_completed==false){
		if(index==pub_num){
		    index=0;
		    Output_Traj=New_Traj_pub;
		    Old_Traj_pub=Output_Traj;
		  
		    flag_concatenate_completed=true;   
		    pub_num=New_Traj_pub.rows();
		    std::cout<<"Speed modified from "<< speed <<" to "<<Current_Speed<<std::endl;
		    speed=Current_Speed;
		    one_time=true;
		    
	      }
	    }     
	  
	    if(index==pub_num) index=0;
   