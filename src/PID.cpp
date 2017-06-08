#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) 
{
	//todo: use twiddle to set these
	Kp_ = Kp;
	Kd_ = Kd;
	Ki_ = Ki;
	time_step = 0;
	sum_cte = 0.0;
	twiddleIterCount = 0;
	twiddleErr = 0.0;
	updateErr = 0.0;
	computeTwiddleErrFlag = true;
	twiddleFlag = false;
	//Create twiddle params
    for(int i=0; i < 3; i++)
    {
      ascendFlags.push_back(true);
    }
    dp.push_back(0.1);
    dp.push_back(1.0);
    dp.push_back(0.001);
    pIndex = -1;
    minSteps = 100;
    totalSteps = 300;
}

void PID::UpdateError(double cte) 
{
	p_error = -Kp_ * cte;
	if(time_step == 0)
	{
		prev_cte = cte;
	}
	double cte_d = cte - prev_cte;
	time_step += 1;
	prev_cte = cte;
	d_error = -Kd_ * cte_d;
	sum_cte += cte;
	i_error = -Ki_ * sum_cte;
}

double PID::TotalError() 
{
	return p_error + d_error + i_error;
}

void PID::Twiddle(double cte, double& best_err, std::vector<double>& dp)
{
	if(twiddleIterCount == totalSteps)
	{
		//Reset count
		twiddleIterCount = 0;
		//Choose appropriate p index
		pIndex += 1;
		pIndex = pIndex % 3;
		std::vector<double> p;
		p.push_back(Kp_);
		p.push_back(Kd_);
		p.push_back(Ki_);

		double err = updateErr / (totalSteps - minSteps);
		updateErr = 0.0;
		
		if (err < best_err)
		{
			best_err = err;
			dp[pIndex] *= 1.1;	
			ascendFlags[pIndex] = true;
		}
		else
		{
			if(ascendFlags[pIndex] == true)
			{
				ascendFlags[pIndex] = false;
			}
			else
			{
				p[pIndex] += dp[pIndex];
				dp[pIndex] *= 0.9;
				ascendFlags[pIndex] = true;
			}
		}
		if(ascendFlags[pIndex] == true)
		{
			p[pIndex] += dp[pIndex];
		}
		else
		{
			p[pIndex] -= 2.0 * dp[pIndex];
		}
		Kp_ = p[0];
		Kd_ = p[1];
		Ki_ = p[2];		
	}
	else
	{
		if(twiddleIterCount >= minSteps)
		{
			updateErr += cte * cte;
		}
	}
}



// void PID::Twiddle(double cte, double& best_err, std::vector<double>& dp)
// {

// 	std::vector<double> p;
// 	p.push_back(Kp_);
// 	p.push_back(Kd_);
// 	p.push_back(Ki_);

// 	for (int i = 0; i < p.size(); i++)
// 	{
// 		if(twiddleIterCount == 1)
// 		{
// 			p[i] += dp[i];
// 			Kp_ = p[0];
// 			Kd_ = p[1];
// 			Ki_ = p[2];
// 		}	
// 		if(twiddleIterCount == 200)
// 		{
// 			twiddleIterCount = 0;
// 			double err = updateErr / 100.0;
// 			updateErr = 0.0;
// 			if (err < best_err)
// 			{
// 				best_err = err;
// 				dp[i] *= 1.1;
// 				p[i] += dp[i];
// 				Kp_ = p[0];
// 				Kd_ = p[1];
// 				Ki_ = p[2];
// 			}
// 			else
// 			{
// 				p[i] -= 2 * dp[i];
// 				Kp_ = p[0];
// 				Kd_ = p[1];
// 				Ki_ = p[2];
// 				if(twiddleIterCount == 200)
// 				{
// 					twiddleIterCount = 0;
// 					double err = updateErr / 100.0;
// 					updateErr = 0.0;
// 					if (err < best_err)
// 					{
// 						best_err = err;
// 						dp[i] *= 1.1;
// 					}
// 					else
// 					{
// 						p[i] += dp[i];
// 						Kp_ = p[0];
// 						Kd_ = p[1];
// 						Ki_ = p[2];
// 						dp[i] *= 0.9;
// 					}
// 				}
// 				else
// 				{
// 					if(twiddleIterCount >= 100)
// 					{
// 						updateErr += cte * cte;
// 					}
// 					//return
// 				}
// 			}	
// 		}
// 		else
// 		{
// 			if(twiddleIterCount >= 100)
// 			{
// 				updateErr += cte * cte;
// 			}
// 			//return
// 		}
// 	}
// }




