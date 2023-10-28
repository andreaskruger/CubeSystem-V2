#ifndef PID_H
#define PID_h

/*Defines*/

/*Prototypes*/

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
void PID_setProportional(double P);

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
void PID_setIntegral(double I);

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
void PID_setDerivate(double D);

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
double PID_getProportional();

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
double PID_getIntegral();

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
double PID_getDerivate();

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
double PID_getError(int i);

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
double PID_getDerError();

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
double PID_getPropError();

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
double PID_getEtime(int i);

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
float PID_angle(float angle, int id );


#endif