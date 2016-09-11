//Defines
#define R                     2.7          //THis is the measured noise estimated on an encoder sampled 0.02 seconds
#define Q                     5
#define LAMBDA_SQUARED        100
#define GAMMA                 1


//Structure to hold Kalman Filter Variables
typedef struct {
  float prevPredictedVelocity = 0;
  float prevVariance = 0;
  float FilteredVelocity = 0;
} KalmanVars;

KalmanVars EstimateVelocity(KalmanVars prevData, float tasktime, float motorRPM)
{

  float PredictedVelocity = 0;
  float Variance = 0;
  float FilterGain = 0;
  float FilteredVelocity = 0;
  float Q_calc = 0;
  float Gamma_Lambda = 0;

   //Calculate Dynamic noise variance...will help with changing speed ranges.
  Gamma_Lambda = motorRPM - prevData.prevPredictedVelocity;
  Q_calc = (LAMBDA_SQUARED * ( tasktime * tasktime ) * ( Gamma_Lambda * Gamma_Lambda ) ) / (1 + GAMMA * ( leftMotor_RPM * leftMotor_RPM ));

  //Calc Filter Values
  PredictedVelocity = prevData.prevPredictedVelocity;
  Variance = prevData.prevVariance + Q_calc;
  FilterGain = Variance / (Variance + R);
  FilteredVelocity = PredictedVelocity + FilterGain * (motorRPM - PredictedVelocity);

  //Update
  prevData.prevVariance = (1 - FilterGain)*(Variance);
  prevData.prevPredictedVelocity = FilteredVelocity;
  prevData.FilteredVelocity = FilteredVelocity;

  return prevData;

}
