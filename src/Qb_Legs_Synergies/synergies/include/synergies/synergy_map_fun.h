/* ------------------------------------------------------------------------- 
	HEADER FUNCTION FOR MAPs OF THE SOFTLegs PROJECT 
	Map function obtained with MultiPolyRegress function
	Generated automatically the 21-Feb-2019 15:50:54
------------------------------------------------------------------------- */
#include<iterator> 
#include <synergies/spline.h> 
Eigen::MatrixXd From_Syn_To_Traj(double speed,double foot_h,double foot_l,Eigen::MatrixXd PCs_Synergy, Eigen::MatrixXd Mus_Synergy){ 

// Size variables
int r_PCs = 360;
int c_PCs = 360;
int r_Mus = 6;
int c_Mus = 6;

// Get principal components and mus from matrices 
Eigen::MatrixXd First_PC(r_PCs,1); 
Eigen::MatrixXd First_mu(r_Mus,1); 
Eigen::MatrixXd Second_PC(r_PCs,1); 
Eigen::MatrixXd Second_mu(r_Mus,1); 
Eigen::MatrixXd Third_PC(r_PCs,1); 
Eigen::MatrixXd Third_mu(r_Mus,1); 
Eigen::MatrixXd Fourth_PC(r_PCs,1); 
Eigen::MatrixXd Fourth_mu(r_Mus,1); 
// PCs
for(int i=0;i<r_PCs;i++){ 
	First_PC(i)  = PCs_Synergy(i,0); 
	Second_PC(i) = PCs_Synergy(i,1); 
	Third_PC(i)  = PCs_Synergy(i,2); 
	Fourth_PC(i) = PCs_Synergy(i,3); 
}
// Mus
for(int i=0;i<r_Mus;i++){ 
	First_mu(i)  = Mus_Synergy(i,0); 
	Second_mu(i) = Mus_Synergy(i,1); 
	Third_mu(i)  = Mus_Synergy(i,2); 
	Fourth_mu(i) = Mus_Synergy(i,3); 
}
// Order of the polynomial used is:3
// Principal components n�1
double Traj_1_gain= +-25.0913*1*1*pow(foot_l,1)+248.1094*1*1*pow(foot_l,2)+-60.8742*1*pow(foot_h,1)*1+-55.0791*1*pow(foot_h,1)*pow(foot_l,1)+-68.1956*1*pow(foot_h,1)*pow(foot_l,2)+-133.1719*1*pow(foot_h,2)*1+429.7568*1*pow(foot_h,2)*pow(foot_l,1)+11.405*pow(speed,1)*1*1+-254.6343*pow(speed,1)*1*pow(foot_l,1)+682.6788*pow(speed,1)*1*pow(foot_l,2)+58.8866*pow(speed,1)*pow(foot_h,1)*1+-34.981*pow(speed,1)*pow(foot_h,1)*pow(foot_l,1)+49.5213*pow(speed,1)*pow(foot_h,2)*1+63.3025*pow(speed,2)*1*1+-72.2718*pow(speed,2)*1*pow(foot_l,1)+-35.4774*pow(speed,2)*pow(foot_h,1)*1+-1.103*1*1*1+-54.8811*pow(speed,3)*1*1+2767.5694*1*pow(foot_h,3)*1+-691.3454*1*1*pow(foot_l,3);
double Pose_1_gain= +-1.1874*1*1*pow(foot_l,1)+-37.6835*1*1*pow(foot_l,2)+3.5078*1*pow(foot_h,1)*1+198.2076*1*pow(foot_h,1)*pow(foot_l,1)+-184.1081*1*pow(foot_h,1)*pow(foot_l,2)+-565.3993*1*pow(foot_h,2)*1+-1048.9327*1*pow(foot_h,2)*pow(foot_l,1)+2.5693*pow(speed,1)*1*1+24.7079*pow(speed,1)*1*pow(foot_l,1)+-69.2271*pow(speed,1)*1*pow(foot_l,2)+-217.4186*pow(speed,1)*pow(foot_h,1)*1+176.3382*pow(speed,1)*pow(foot_h,1)*pow(foot_l,1)+1264.7445*pow(speed,1)*pow(foot_h,2)*1+-21.5515*pow(speed,2)*1*1+24.7358*pow(speed,2)*1*pow(foot_l,1)+139.6262*pow(speed,2)*pow(foot_h,1)*1+-0.34463*1*1*1+13.5182*pow(speed,3)*1*1+4654.2497*1*pow(foot_h,3)*1+100.3403*1*1*pow(foot_l,3);

// Order of the polynomial used is:3
// Principal components n�2
double Traj_2_gain=+-24.1818*1*1*pow(foot_l,1)+201.0469*1*1*pow(foot_l,2)+-111.677*1*pow(foot_h,1)*1+128.8948*1*pow(foot_h,1)*pow(foot_l,1)+44.1957*1*pow(foot_h,1)*pow(foot_l,2)+1455.5577*1*pow(foot_h,2)*1+-1350.0738*1*pow(foot_h,2)*pow(foot_l,1)+18.2547*pow(speed,1)*1*1+24.932*pow(speed,1)*1*pow(foot_l,1)+688.3527*pow(speed,1)*1*pow(foot_l,2)+132.5806*pow(speed,1)*pow(foot_h,1)*1+-424.4439*pow(speed,1)*pow(foot_h,1)*pow(foot_l,1)+-599.4049*pow(speed,1)*pow(foot_h,2)*1+-62.9367*pow(speed,2)*1*1+-437.8734*pow(speed,2)*1*pow(foot_l,1)+-12.0392*pow(speed,2)*pow(foot_h,1)*1+0.32022*1*1*1+140.4432*pow(speed,3)*1*1+-9501.1467*1*pow(foot_h,3)*1+-720.6454*1*1*pow(foot_l,3);
double Pose_2_gain=+1.1805*1*1*pow(foot_l,1)+-28.5906*1*1*pow(foot_l,2)+9.0269*1*pow(foot_h,1)*1+-44.85*1*pow(foot_h,1)*pow(foot_l,1)+-42.8561*1*pow(foot_h,1)*pow(foot_l,2)+21.3706*1*pow(foot_h,2)*1+519.4341*1*pow(foot_h,2)*pow(foot_l,1)+0.82469*pow(speed,1)*1*1+22.1647*pow(speed,1)*1*pow(foot_l,1)+-115.1408*pow(speed,1)*1*pow(foot_l,2)+-21.4721*pow(speed,1)*pow(foot_h,1)*1+164.5975*pow(speed,1)*pow(foot_h,1)*pow(foot_l,1)+-162.2885*pow(speed,1)*pow(foot_h,2)*1+-8.6265*pow(speed,2)*1*1+26.3035*pow(speed,2)*1*pow(foot_l,1)+-27.0683*pow(speed,2)*pow(foot_h,1)*1+-0.076594*1*1*1+6.4317*pow(speed,3)*1*1+-1074.5054*1*pow(foot_h,3)*1+112.5269*1*1*pow(foot_l,3);

// Order of the polynomial used is:3
// Principal components n�3
double Traj_3_gain=+-13.2121*1*1*pow(foot_l,1)+-12.1854*1*1*pow(foot_l,2)+-114.5995*1*pow(foot_h,1)*1+243.3701*1*pow(foot_h,1)*pow(foot_l,1)+1324.0529*1*pow(foot_h,1)*pow(foot_l,2)+3504.3094*1*pow(foot_h,2)*1+-4152.3083*1*pow(foot_h,2)*pow(foot_l,1)+0.34707*pow(speed,1)*1*1+124.506*pow(speed,1)*1*pow(foot_l,1)+-481.9094*pow(speed,1)*1*pow(foot_l,2)+15.455*pow(speed,1)*pow(foot_h,1)*1+-1783.9773*pow(speed,1)*pow(foot_h,1)*pow(foot_l,1)+-871.595*pow(speed,1)*pow(foot_h,2)*1+-65.4897*pow(speed,2)*1*1+121.0472*pow(speed,2)*1*pow(foot_l,1)+607.9425*pow(speed,2)*pow(foot_h,1)*1+2.2355*1*1*1+47.6981*pow(speed,3)*1*1+-25406.4845*1*pow(foot_h,3)*1+235.61*1*1*pow(foot_l,3);
double Pose_3_gain=+0.090972*1*1*pow(foot_l,1)+2.7762*1*1*pow(foot_l,2)+-9.3645*1*pow(foot_h,1)*1+-5.3874*1*pow(foot_h,1)*pow(foot_l,1)+171.9167*1*pow(foot_h,1)*pow(foot_l,2)+210.9116*1*pow(foot_h,2)*1+-440.9223*1*pow(foot_h,2)*pow(foot_l,1)+-0.95109*pow(speed,1)*1*1+8.6603*pow(speed,1)*1*pow(foot_l,1)+21.0327*pow(speed,1)*1*pow(foot_l,2)+23.3142*pow(speed,1)*pow(foot_h,1)*1+-149.7205*pow(speed,1)*pow(foot_h,1)*pow(foot_l,1)+71.0464*pow(speed,1)*pow(foot_h,2)*1+-1.6375*pow(speed,2)*1*1+-20.3445*pow(speed,2)*1*pow(foot_l,1)+9.37*pow(speed,2)*pow(foot_h,1)*1+0.079311*1*1*1+5.0484*pow(speed,3)*1*1+-1380.0939*1*pow(foot_h,3)*1+-27.382*1*1*pow(foot_l,3);

// Order of the polynomial used is:3
// Principal components n�4
double Traj_4_gain=+-1.812*1*1*pow(foot_l,1)+-239.9242*1*1*pow(foot_l,2)+20.1335*1*pow(foot_h,1)*1+-43.8448*1*pow(foot_h,1)*pow(foot_l,1)+733.0162*1*pow(foot_h,1)*pow(foot_l,2)+304.5287*1*pow(foot_h,2)*1+-1046.2296*1*pow(foot_h,2)*pow(foot_l,1)+1.9085*pow(speed,1)*1*1+250.7879*pow(speed,1)*1*pow(foot_l,1)+-514.2354*pow(speed,1)*1*pow(foot_l,2)+-39.8555*pow(speed,1)*pow(foot_h,1)*1+-203.2881*pow(speed,1)*pow(foot_h,1)*pow(foot_l,1)+397.2772*pow(speed,1)*pow(foot_h,2)*1+-76.4581*pow(speed,2)*1*1+44.0823*pow(speed,2)*1*pow(foot_l,1)+-4.9359*pow(speed,2)*pow(foot_h,1)*1+0.35375*1*1*1+54.9494*pow(speed,3)*1*1+-4295.4347*1*pow(foot_h,3)*1+546.26*1*1*pow(foot_l,3);
double Pose_4_gain=+-2.5361e-05*1*1*pow(foot_l,1)+0.076632*1*1*pow(foot_l,2)+-0.0026495*1*pow(foot_h,1)*1+0.13385*1*pow(foot_h,1)*pow(foot_l,1)+-0.5657*1*pow(foot_h,1)*pow(foot_l,2)+-0.011343*1*pow(foot_h,2)*1+0.037041*1*pow(foot_h,2)*pow(foot_l,1)+0.0019992*pow(speed,1)*1*1+-0.11694*pow(speed,1)*1*pow(foot_l,1)+0.51005*pow(speed,1)*1*pow(foot_l,2)+-0.072641*pow(speed,1)*pow(foot_h,1)*1+0.26793*pow(speed,1)*pow(foot_h,1)*pow(foot_l,1)+0.018555*pow(speed,1)*pow(foot_h,2)*1+0.033971*pow(speed,2)*1*1+-0.17887*pow(speed,2)*1*pow(foot_l,1)+0.03348*pow(speed,2)*pow(foot_h,1)*1+-0.00013738*1*1*1+0.0050951*pow(speed,3)*1*1+-0.042198*1*pow(foot_h,3)*1+-0.32426*1*1*pow(foot_l,3);

// Result of the map
Eigen::MatrixXd out_Traj =  First_PC*Traj_1_gain+Second_PC*Traj_2_gain+Third_PC*Traj_3_gain+Fourth_PC*Traj_4_gain;
Eigen::MatrixXd New_out_Traj(60,6); 
for(int i=0;i<6;i++){ 
	 for(int j=0;j<60;j++) 
	 	 New_out_Traj(j,i) = out_Traj(i*60+j) + Pose_1_gain*First_mu(i)+Pose_2_gain*Second_mu(i)+Pose_3_gain*Third_mu(i)+Pose_4_gain*Fourth_mu(i);
}
return New_out_Traj;
}

// Autogenerated from the Main Scripts SOFTLegs 
Eigen::MatrixXd Interpolate_and_Resample(Eigen::MatrixXd Trajectory, double Traj_Ts, double Pub_Ts){ 

int Traj_num=Trajectory.rows(); // this is 60x6 
int n_joints=Trajectory.cols(); 
int pub_num=std::round(Traj_num*Traj_Ts/Pub_Ts); 

std::vector<double> X_traj(Traj_num),X_pub(pub_num),Traj_pub(pub_num), Traj(Traj_num); 

Eigen::MatrixXd out_Traj(pub_num,n_joints); 
//	The time vectors old and new 
for(int i=0;i<Traj_num;i++){ 
	X_traj[i]=(i+1)*Traj_Ts; // curr_Ts is the sample time of the trajectory that we want to implement 
} 

for(int i=0;i<pub_num;i++){ 
	X_pub[i]=(i+1)*Pub_Ts;   // curr_Ts is the sample time of the trajectory that we want to implement 
} 

tk::spline s; 
for(int i=0;i<n_joints;i++){ 
	for(int j=0;j<Traj_num;j++){ 
		Traj[j]=Trajectory(j,i); 
	}
	s.set_points(X_traj,Traj); 
	for(int j=0;j<pub_num;j++){ 
		out_Traj(j,i)=s(X_pub[j]); 
	}
}
return out_Traj; 
}


// Autogenerated from the Main Scripts SOFTLegs 
Eigen::MatrixXd Interpolate_and_Resample_Middle(Eigen::MatrixXd New_Traj, double Ts_new, Eigen::MatrixXd Old_Traj, double Ts_old, int Instant, double Pub_Ts){ 

int Traj_num=Old_Traj.rows();// this is 60x6 and it is the same of the new 
int n_joints=Old_Traj.cols(); 
int pub_num=std::round(((Instant-1)*Ts_old+(Traj_num-Instant+1)*Ts_new)/Pub_Ts); 
// -1 because we switch the signals at i==Instant, i.e. i==Instant New_signals(i) 

std::vector<double> X_traj(Traj_num),X_pub(pub_num),Traj_pub(pub_num), Traj(Traj_num); 

Eigen::MatrixXd Middle_Traj_pub(pub_num,n_joints); 
for(int i=0;i<Traj_num;i++){ 
	if(i<Instant){ 
		X_traj[i]=(i+1)*Ts_old; // curr_Ts is the sample time of the trajectory that we want to implement 
	}else{ 
		X_traj[i]=X_traj[Instant-1]+(i-Instant+1)*Ts_new;  
	} 
} 
for(int i=0;i<pub_num;i++){ 
	X_pub[i]=(i+1)*Pub_Ts; // curr_Ts is the sample time of the trajectory that we want to implement 
} 
tk::spline s; 
for(int i=0;i<n_joints;i++){ 
	for(int j=0;j<Traj_num;j++){ 
		if(j<Instant){ 
			Traj[j]=Old_Traj(j,i); 
		}else{ 
			Traj[j]=New_Traj(j,i); 
		} 
	} 
	s.set_points(X_traj,Traj); 
	for(int j=0;j<pub_num;j++){ 
		Middle_Traj_pub(j,i)=s(X_pub[j]); 
	} 
} 
return Middle_Traj_pub; 
} 
