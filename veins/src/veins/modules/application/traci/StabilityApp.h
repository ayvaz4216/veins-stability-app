#pragma once
#include "veins/modules/application/traci/MyVeinsApp.h"
#include "veins/modules/application/traci/StabilityMessage_m.h"
#include <map>
#include <string>

namespace veins {

enum RoutingMode {
    GREEDY,      // Forward to closest neighbor to destination
    LET_BASED,   // Forward based on Link Expiration Time
    STABILITY    // Your original stability-based approach
};

// Structure to store neighbor information
struct NeighborInfo {
    Coord position;
    Coord velocity;
    simtime_t lastUpdate;
    double let; // Link Expiration Time
    bool isStable;
    
    NeighborInfo() : position(0,0,0), velocity(0,0,0), 
                     lastUpdate(0), let(0), isStable(false) {}
};

class StabilityApp : public MyVeinsApp {
  protected:
    cMessage* sendTimer;
    cMessage* dataPacketTimer;
    
    // VARIABLES FOR MANUAL PHYSICS
    Coord lastPos;
    Coord myCurrentVelocity;
    simtime_t lastCalcTime;

    // Routing variables
    RoutingMode currentMode;
    std::map<int, NeighborInfo> neighborTable;
    Coord destinationPos; // Target destination for routing
    int myId; // This node's ID
    int packetsSent;
    int packetsDelivered;
    int packetsDropped;
    
    // Statistics per routing mode
    std::map<RoutingMode, int> sentCount;
    std::map<RoutingMode, int> deliveredCount;
    std::map<RoutingMode, int> droppedCount;
    std::map<RoutingMode, double> totalDelay;
    
    double transmissionRange; // Max communication range

    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void handleSelfMsg(cMessage* msg) override;
    virtual void onWSM(BaseFrame1609_4* frame) override;
    virtual void onBeaconMessage(StabilityMessage* wsm);
    virtual void onDataMessage(StabilityMessage* wsm);
    virtual void populateStabilityMessage(StabilityMessage* wsm);
    
    // Routing functions
    virtual void sendDataPacket();
    virtual int selectNextHopGreedy();
    virtual int selectNextHopLET();
    virtual int selectNextHopStability();
    virtual double calculateLET(const NeighborInfo& neighbor);
    virtual void updateNeighborTable(int senderId, Coord pos, Coord vel);
    virtual void cleanupNeighborTable();

  public:
    StabilityApp() : sendTimer(nullptr), dataPacketTimer(nullptr),
                     packetsSent(0), packetsDelivered(0), packetsDropped(0),
                     transmissionRange(300.0) {}
    virtual ~StabilityApp();
};

} // namespace veins
