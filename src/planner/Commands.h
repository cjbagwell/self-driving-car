/**
 * @file Commands.h
 * @author C. Jordan Bagwell (cjbagwell@crimson.ua.edu)
 * @brief TODO: some stuff here
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */


/**
 * @brief Controls that can be executed by a self-driving car.
 * 
 * @param app The accelerator pedal position (APP) for a vehicle.  This is represented as a
 * value between 0 and 1 where 0 represents no APP and 1 represents maximum APP.
 * 
 * @param bpp The brake pedal position (BPP) for a vehicle.  This is represented as a value
 * between 0 and 1 where 0 represents no BPP and 1 represents maximum BPP.
 * 
 * @param steerAngleRate The rate at which the steering agle should of the vehicle should change.
 * This is expressed in radians per second [rad/s]
 * 
 */
class Commands{
private:
    double app;
    double bpp;
    double steerAngleRate;
public:
    Commands()
                :
                app(0.0), 
                bpp(0.0), 
                steerAngleRate(0.0)
                {};

    Commands(double app, 
                double bpp, 
                double steerAngleRate)
                :
                app(app), 
                bpp(bpp), 
                steerAngleRate(steerAngleRate) 
                {};

    double getApp(){return this->app;}
    double getBpp(){return this->bpp;}
    double getSteeringAngleRate(){return this->steerAngleRate;}
    void setApp(double newApp){this->app = newApp;}
    void setBpp(double newBpp){this->bpp = newBpp;}
    void setSteeringAngleRate(double newSAR){this->steerAngleRate=newSAR;}
};
