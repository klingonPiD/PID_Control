#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  long long time_step;
  double prev_cte;
  double sum_cte;

  //twiddle params
  bool computeTwiddleErrFlag;
  bool twiddleFlag;
  int twiddleIterCount;
  double twiddleErr;
  double updateErr;
  double finalErr;
  std::vector<double> dp;
  std::vector<bool> ascendFlags;
  int pIndex;
  int minSteps;
  int totalSteps;
  double bestKp;
  double bestKd;
  double bestKi;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Kd, double Ki);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  //double Twiddle(std::vector<double> v, double cte);
  void Twiddle(double cte, double& best_err, std::vector<double>& dp);
};

#endif /* PID_H */
