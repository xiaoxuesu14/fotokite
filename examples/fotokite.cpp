#include "Fotokite.hpp" 

int main(int argc, char *argv[]) {

    // Initialize Fotokite interface
    //    Fotokite * fotokite = new Fotokite("127.0.0.1", 5050);
    Fotokite * fotokite = new Fotokite("/dev/cu.usbmodem1");

    // Print state of fotokite
    while (true) {
        fotokite->printState();
    }

    // Ground status message test
    //    fotokite->getGroundMode();
    //    fotokite->getRuntimeS();
    //    fotokite->getGroundBattVoltage();
    //    fotokite->getRelTetherLength();

    // Attitude message test
    //    fotokite->getQX();
    //    fotokite->getQY();
    //    fotokite->getQZ();
    //    fotokite->getQW();

    // Position message test
    //    fotokite->getElevation();
    //    fotokite->getRelAzimuth();
    //    fotokite->getBaroAlt();

    // Flight status message test
    //    fotokite->getFlightMode();
    //    fotokite->getOnTime();
    //    fotokite->getBackup();
    //    fotokite->getFlags();

    // Gimbal test
    //    fotokite->gimbal(0.30, 0.1);
    //    fotokite->gimbalRoll(0.29);
    //    fotokite->gimbalPitch(0.30);

    // Position test
    //    fotokite->pos(-0.25, -0.2, -77836.8758);
    //    fotokite->posV(0.25);
    //    fotokite->posH(0.200000);
    //    fotokite->posL(0);

    // Yaw test
    //    fotokite->yaw(-0.1);

    // Delete Fotokite object (important for clean exit)
    delete fotokite;

    return 0;
}