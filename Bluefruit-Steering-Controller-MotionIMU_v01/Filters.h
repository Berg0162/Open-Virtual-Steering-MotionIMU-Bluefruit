#ifndef _Filters_h
#define _Filters_h

/* A simplified one dimensional Kalman filter implementation - actually a single variable low pass filter ;-)
   Based on: http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/

                  by Bruno Azevedo Chagas
      https://github.com/bachagas/Kalman/tree/master
*/
/* --------------  A simplified one dimensional Kalman filter implementation --------------
* The initial values for p is not very important since it is adjusted during the process. 
* It must be just high enough to narrow down. The initial value for the sensor readout is also not 
* very important, since it is updated during the process. But tweaking the values for the 
* process noise (q) and sensor noise (r) is essential to get clear sensor readouts.
* q = 0.125 //process noise covariance
* r = 32    //sensor noise covariance
* p = 1023  //estimated error -> large enough to narrow down
* x = 0.0   //initial value
*  
*  Kalman(double process_noise, double sensor_noise, double estimated_error, double intial_value)
 --------------------------------------------------------------------------------------------*/

class Kalman {
  private:
    /* Kalman filter variables */
    double q; //process noise covariance
    double r; //measurement noise covariance
    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain
    
  public:
    Kalman(double process_noise, double sensor_noise, double estimated_error, double intial_value) {
      /* The variables are x for the filtered value, q for the process noise, 
         r for the sensor noise, p for the estimated error and k for the Kalman Gain. 
         The state of the filter is defined by the values of these variables.
         
         The initial values for p is not very important since it is adjusted
         during the process. It must be just high enough to narrow down.
         The initial value for the readout is also not very important, since
         it is updated during the process.
         But tweaking the values for the process noise and sensor noise
         is essential to get clear readouts.
         
         For large noise reduction, you can try to start from: (see http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/ )
         q = 0.125
         r = 32
         p = 1023 //"large enough to narrow down"
         e.g.
         myVar = Kalman(0.125,32,1023,0);
      */
        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
        this->x = intial_value; //x will hold the iterated filtered value
    }
    
    double getFilteredValue(double measurement) {
      /* Updates and gets the current measurement value */
      //prediction update
      //omit x = x
      this->p = this->p + this->q;
    
      //measurement update
      this->k = this->p / (this->p + this->r);
      this->x = this->x + this->k * (measurement - this->x);
      this->p = (1 - this->k) * this->p;
      
      return this->x;
    }
    
    void setParameters(double process_noise, double sensor_noise, double estimated_error) {
        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
    }

    void setParameters(double process_noise, double sensor_noise) {
        this->q = process_noise;
        this->r = sensor_noise;
    }
    
    double getProcessNoise() {
      return this->q;
    }
    
    double getSensorNoise() {
      return this->r;
    }
    
    double getEstimatedError() {
      return this->p;
    }
};

class EMA_Filter {
private:
  float emaAlphaFactor;
  float exponential_average;
public:
  EMA_Filter(const float emaAlpha) {
    emaAlphaFactor = emaAlpha;
    }

  float getFilteredValue(float current_value) {
    exponential_average = (emaAlphaFactor*current_value + (100-emaAlphaFactor)*exponential_average) / 100 ;
    return exponential_average;
  }  
  
  float clearFilter(float current_value) {
    exponential_average = current_value; // Clear 
    return exponential_average;
  }
};

class StdDeviation {
private:
  uint8_t counter = 1;
  double sum = 0.0; 
public:
  StdDeviation() {}
  double GetStdDeviation(float M1, float M2) {
      counter++;
      M1 = fabs(M1);
      M2 = fabs(M2);
      sum += pow((M1-M2), 2);
      return sqrt(sum/(counter-1));
  }
  double ClearStdDeviation(void) {
      counter = 1; 
      sum = 0.0;
      return 0.0;    
  }
};

#endif
