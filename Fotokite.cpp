/* 
 * File:   Fotokite.cpp
 * Author: Jan Dufek
 * 
 * Created on April 27, 2017, 2:20 PM
 */

#include "Fotokite.hpp"

Fotokite::Fotokite(const char * ip_address, const short port) {
    
    // Initialize Fotokite state
    state = new FotokiteState();
    
    // Initialize communication with Fotokite OCU Server
    communication = new SocketCommunication(state, ip_address, port);

}

Fotokite::Fotokite(const char * serialPort) {
    
    // Initialize Fotokite state
    state = new FotokiteState();
    
    // Initialize communication with Fotokite ground station over serial port
    communication = new SerialCommunication(state, serialPort);

}

Fotokite::Fotokite(const Fotokite& orig) {
}

Fotokite::~Fotokite() {
    
    // Stop motion
    this->gimbal(0, 0);
    this->pos(0, 0, 0);
    this->yaw(0);
    
    // Delete communication
    delete communication;
    
}

/**
 * Ground Station mode.
 * 
 * @return 0: IDLE, 1: MENU, 2: LAIRD_SCREEN, 10: REMOTE_CONTROLLED
 */
int Fotokite::getGroundMode() {
    return state->groundMode;
}

/**
 * Ground Station on time in seconds.
 * 
 * @return 
 */
double Fotokite::getRuntimeS() {
    return state->runtimeS;
}

/**
 * Ground Station battery voltage in volts.
 * 
 * @return 
 */
double Fotokite::getGroundBattVoltage() {
    return state->groundBattVoltage;
}

/**
 * Tether length in encoder counts relative to the when the Ground Station was turned on.
 * 
 * @return 
 */
double Fotokite::getRelTetherLength() {
    return state->relTetherLength;
}

/**
 * QX, QY, QZ, QW are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQX() {
    return state->QX;
}

/**
 * QX, QY, QZ, QW are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQY() {
    return state->QY;
}

/**
 * QX, QY, QZ, QW are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQZ() {
    return state->QZ;
}

/**
 * QX, QY, QZ, QW are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQW() {
    return state->QW;
}

/**
 * Vertical angle of tether in radians where 0 is aligned with gravity.
 * 
 * @return 
 */
double Fotokite::getElevation() {
    return state->elevation;
}

/**
 * Relative horizontal angle of tether in radians w.r.t the vehicle initialization frame.
 * 
 * @return 
 */
double Fotokite::getRelAzimuth() {
    return state->relAzimuth;
}

/**
 * Barometric altitude.
 * 
 * @return 
 */
double Fotokite::getBaroAlt() {
    return state->baroAlt;
}

/**
 * Flight mode.
 * 
 * @return 1: IDLE, 2: ANGLE, 3: ANGLE_RATE, 5: MOTOR_LOW_LEVEL, 48: EMERGENCY_LAND, 49: MOTOR_STOP_RESTART, 50: TIMEOUT, 52: MOTOR_STOP_DISABLE, 53: BATTERY_TOO_LOW_TO_START, 54: NO_TETHER_POWER, 55: HARD_KILL
 */
int Fotokite::getFlightMode() {
    return state->flightMode;
}

/**
 * Flight Unit on time in seconds.
 * 
 * @return 
 */
double Fotokite::getOnTime() {
    return state->onTime;
}

/**
 * Backup battery voltage in volts.
 * @return 
 */
double Fotokite::getBackup() {
    return state->backup;
}

/**
 * Bit fields for the flight flags.
 * 
 * Bit 0: MISSED_TICK
 * Bit 1: I2C_ERROR
 * Bit 2: CRIT_LOW_BATT
 * Bit 3: NO_SYNC_CMDS
 * Bit 4: LOW_LAUNCH_ANGLE
 * Bit 5: MOTOR_ERROR
 * Bit 6: LOW_BATT
 * Bit 7: LOW_BACKUP_BATT
 * Bit 8: EMERGENCY_LAND
 * Bit 9: NO_TETHER_DETECTED
 * Bit 10: HARD_KILL
 * Bit 11: OPTOFORCE_TEMP_INVALID
 * Bit 12: FLYAWAY_CANDIDATE
 * 
 * @return 
 */
unsigned int Fotokite::getFlags() {
    return state->flags;
}

/**
 * Check if the value lies within [-limit, limit] interval.
 * 
 * @param value
 * @param limit
 */
double Fotokite::correctInputValues(double value, double limit) {

    // Check if the values are legal
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    } else {
        return value;
    }

}

/**
 * Sends string command to Fotokite
 * 
 * @param command
 */
void Fotokite::sendCommand(string command) {
    
    this->communication->send(command + '\n');
    
}

/**
 * Adjusts desired gimbal angle rate for both pitch and roll. Floating point values in rad/s.
 * 
 * @param pitchRate
 * @param rollRate
 */
void Fotokite::gimbal(double pitchRate, double rollRate) {

    // Check if the values are legal
    pitchRate = correctInputValues(pitchRate, 0.3);
    rollRate = correctInputValues(rollRate, 0.3);

    // Send command
    sendCommand("Gimbal " + to_string(pitchRate) + "," + to_string(rollRate));

}

/**
 * Adjusts desired gimbal angle rate for roll. Floating point values in rad/s.
 * 
 * @param pitchRate
 * @param rollRate
 */
void Fotokite::gimbalRoll(double rollRate) {

    // Check if the values are legal
    rollRate = correctInputValues(rollRate, 0.3);

    // Send command
    sendCommand("GimbalRoll " + to_string(rollRate));

}

/**
 * Adjusts desired gimbal angle rate for pitch. Floating point values in rad/s.
 * 
 * @param pitchRate
 * @param rollRate
 */
void Fotokite::gimbalPitch(double pitchRate) {

    // Check if the values are legal
    pitchRate = correctInputValues(pitchRate, 0.3);

    // Send command
    sendCommand("GimbalPitch " + to_string(pitchRate));

}

/**
 * Adjust the position of the Flight Unit by commanding a elevation or azimuth angle rate or a change in tether length. Elevation and azimuth are signed floating point values in rad/s. Length is in spool encoder counts.
 * 
 * @param elevRate
 * @param azimuthRate
 * @param lengthDelta
 */
void Fotokite::pos(double elevRate, double azimuthRate, double lengthDelta) {

    // Check if the values are legal
    elevRate = correctInputValues(elevRate, 0.25);
    azimuthRate = correctInputValues(azimuthRate, 0.2);
    
    // Send command
    sendCommand("Pos " + to_string(elevRate) + "," + to_string(azimuthRate) + "," + to_string(lengthDelta));

}

/**
 * Adjust the position of the Flight Unit by commanding a elevation. Elevation is signed floating point value in rad/s.
 * 
 * @param elevRate
 */
void Fotokite::posV(double elevRate) {
    
    // Check if the values are legal
    elevRate = correctInputValues(elevRate, 0.25);
    
    // Send command
    sendCommand("PosV " + to_string(elevRate));

}

/**
 * Adjust the position of the Flight Unit by commanding an azimuth angle rate. Azimuth is signed floating point values in rad/s.
 * 
 * @param azimuthRate
 */
void Fotokite::posH(double azimuthRate) {

    // Check if the values are legal
    azimuthRate = correctInputValues(azimuthRate, 0.2);
    
    // Send command
    sendCommand("PosH " + to_string(azimuthRate));
    
}

/**
 * Adjust the position of the Flight Unit by commanding a change in tether length. Length is in spool encoder counts.
 * 
 * @param lengthDelta
 */
void Fotokite::posL(double lengthDelta) {

    // Check if the values are legal
    lengthDelta = correctInputValues(lengthDelta, numeric_limits<float>::max());
    
    // Send command
    sendCommand("PosL " + to_string(lengthDelta));
    
}

/**
 * Adjust the yaw orientation of the Flight Unit by commanding a yaw angle rate, signed floating point, in rad/s.
 * 
 * @param yawRate
 */
void Fotokite::yaw(double yawRate) {
    
    // Check if the values are legal
    yawRate = correctInputValues(yawRate, 0.4);
    
    // Send command
    sendCommand("Yaw " + to_string(yawRate));
    
}

/**
 * Prints the current state of Fotokite to the console.
 * 
 */
void Fotokite::printState() {
    
    string GSStatus = "!GSStatus " + to_string(getGroundMode()) + "," + to_string(getRuntimeS()) + "," + to_string(getGroundBattVoltage()) + "," + to_string(getRelTetherLength());
    string Attitude = "!Attitude " + to_string(getQX()) + "," + to_string(getQY()) + "," + to_string(getQZ()) + "," + to_string(getQW());
    string Pos = "!Pos " + to_string(getElevation()) + "," + to_string(getRelAzimuth()) + "," + to_string(getBaroAlt());
    string FlightStatus = "!FlightStatus " + to_string(getFlightMode()) + "," + to_string(getOnTime()) + "," + to_string(getBackup()) + "," + to_string(getFlags());
    
    string currentStatus = GSStatus + "\n" + Attitude + "\n" + Pos + "\n" + FlightStatus;
    
    cout << currentStatus << endl;
    
}