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
	this->Kp = Kp;
	this->Kd = Kd;
	this->Ki = Ki;
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
	p_error = -Kp * cte;
	if(time_step == 0)
	{
		prev_cte = cte;
	}
	double cte_d = cte - prev_cte;
	time_step += 1;
	prev_cte = cte;
	d_error = -Kd * cte_d;
	sum_cte += cte;
	i_error = -Ki * sum_cte;
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
		p.push_back(Kp);
		p.push_back(Kd);
		p.push_back(Ki);

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
		Kp = p[0];
		Kd = p[1];
		Ki = p[2];		
	}
	else
	{
		if(twiddleIterCount >= minSteps)
		{
			updateErr += cte * cte;
		}
	}
}





