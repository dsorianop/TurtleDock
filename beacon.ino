#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEBeacon.h>
#include <BLEServer.h>

//identifier for scanning devices to search for
#define BEACON_UUID "6ba7b810-9dad-11d1-80b4-00c04fd430c8"

void setup() {
    //slow baud rates created noticeable delays, so highest baud rate was chosen for faster output
    Serial.begin(115200);
    Serial.println("Starting BLE Beacon (TurtleDock)...");
    
    //appears in scans
    BLEDevice::init("TurtleDock");
    
    //the new server handles the stack and the advertising
    BLEServer *pServer = BLEDevice::createServer();
    
    //in charge of intervals and how we broadcast our beacon data
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    
    //holds beacon information
    BLEAdvertisementData advertisementData;
    
    //creates beacon object
    BLEBeacon tDBeacon = BLEBeacon();
    
    //0x4C00 is Apple's assigned ID for iBeacon
    tDBeacon.setManufacturerId(0x4C00);
    
    //the UUID is set as the primary identifier for the beacon network
    tDBeacon.setProximityUUID(BLEUUID(BEACON_UUID));
    
    //tentative
    //makes beacon visible without requiring specific search criteria
    advertisementData.setFlags(0x04);
    
    //data (UUID, major, minor, txpower) is retrieved
    advertisementData.setManufacturerData(tDBeacon.getData());
    
    //data->object
    pAdvertising->setAdvertisementData(advertisementData);
    
    //included per online discussions in order to minimize power consumption
    pAdvertising->setScanResponse(false);
    
    //minimum advertising interval
    pAdvertising->setMinInterval(0x20);
    
    //maximum advertising interval
    pAdvertising->setMaxInterval(0x40);
    
    BLEDevice::startAdvertising();
    
    Serial.println("Beacon broadcasting has started.");
}

void loop() {
    //left empty intentionally
}