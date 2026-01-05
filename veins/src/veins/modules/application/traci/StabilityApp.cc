#include "veins/modules/application/traci/StabilityApp.h"
#include <algorithm>
#include <limits>

using namespace veins;

Define_Module(StabilityApp);

StabilityApp::~StabilityApp() {
    cancelAndDelete(sendTimer);
    cancelAndDelete(dataPacketTimer);
}

void StabilityApp::initialize(int stage) {
    MyVeinsApp::initialize(stage);
    if (stage == 0) {
        sendTimer = new cMessage("beacon-timer");
        dataPacketTimer = new cMessage("data-timer");
        
        // Initialize physics variables
        lastPos = mobility->getPositionAt(simTime());
        lastCalcTime = simTime();
        myCurrentVelocity = Coord(0,0,0);

        // Get my node ID
        myId = getParentModule()->getIndex();

        // Set routing mode from NED parameter (default: STABILITY)
        int mode = par("routingMode").intValue();
        currentMode = static_cast<RoutingMode>(mode);
        
        transmissionRange = par("transmissionRange").doubleValue();
        
        // Don't set destination yet - will be set dynamically when sending packets
        destinationPos = Coord(0, 0, 0);

        // Initialize counters
        packetsSent = 0;
        packetsDelivered = 0;
        packetsDropped = 0;

        // Start beacon transmission
        scheduleAt(simTime() + uniform(0.1, 0.5), sendTimer);
        
        // Start data packet transmission after initial setup
        scheduleAt(simTime() + 2.0, dataPacketTimer);
    }
}

void StabilityApp::handleSelfMsg(cMessage* msg) {
    if (msg == sendTimer) {
        // --- 1. MANUAL PHYSICS CALCULATION ---
        Coord currentPos = mobility->getPositionAt(simTime());
        double dt = (simTime() - lastCalcTime).dbl();

        if (dt > 0) {
            myCurrentVelocity = (currentPos - lastPos) / dt;
        }

        lastPos = currentPos;
        lastCalcTime = simTime();

        // --- 2. CREATE & SEND BEACON ---
        StabilityMessage* wsm = new StabilityMessage();
        populateWSM(wsm);
        populateStabilityMessage(wsm);
        sendDown(wsm);

        // Clean up old neighbors
        cleanupNeighborTable();

        // --- 3. LOOP ---
        scheduleAt(simTime() + 1.0, sendTimer);
    } 
    else if (msg == dataPacketTimer) {
        sendDataPacket();
        scheduleAt(simTime() + 5.0, dataPacketTimer); // Send every 5 seconds
    }
    else {
        MyVeinsApp::handleSelfMsg(msg);
    }
}

void StabilityApp::populateStabilityMessage(StabilityMessage* wsm) {
    wsm->setSenderVelocity(myCurrentVelocity);
    wsm->setSenderPos(mobility->getPositionAt(simTime()));
    wsm->setSenderAddress(myId);
    wsm->setMessageType(BEACON);  // Mark as beacon message
}

void StabilityApp::onWSM(BaseFrame1609_4* frame) {
    StabilityMessage* wsm = dynamic_cast<StabilityMessage*>(frame);
    if (!wsm) {
        MyVeinsApp::onWSM(frame);
        return;
    }

    // Handle different message types
    if (wsm->getMessageType() == BEACON) {
        onBeaconMessage(wsm);
    } else if (wsm->getMessageType() == DATA) {
        onDataMessage(wsm);
    }

    MyVeinsApp::onWSM(frame);
}

void StabilityApp::onBeaconMessage(StabilityMessage* wsm) {
    // Update neighbor table
    int senderId = wsm->getSenderAddress();
    Coord senderPos = wsm->getSenderPos();
    Coord senderVel = wsm->getSenderVelocity();
    
    updateNeighborTable(senderId, senderPos, senderVel);
    
    // Get neighbor info
    NeighborInfo& neighbor = neighborTable[senderId];
    
    // Calculate relative speed for stability assessment
    double relSpeed = (senderVel - myCurrentVelocity).length();
    
    // Stability threshold: 14 m/s (approx 50 km/h)
    neighbor.isStable = (relSpeed <= 14.0);
    
    // Visual feedback
    if (neighbor.isStable) {
        EV << "Stable Neighbor " << senderId << " (RelSpeed: " << relSpeed << "m/s)" << endl;
    } else {
        EV << "Unstable Neighbor " << senderId << " (RelSpeed: " << relSpeed << "m/s)" << endl;
    }
}

void StabilityApp::onDataMessage(StabilityMessage* wsm) {
    // Received a data packet to forward
    int senderId = wsm->getSenderAddress();
    int destId = wsm->getDestinationId();
    Coord destPos = wsm->getDestinationPos();
    
    EV << "Received DATA packet from " << senderId 
       << " (hops: " << wsm->getHopCount() << ")" << endl;
    
    // Check if we are the destination
    Coord myPos = mobility->getPositionAt(simTime());
    double distToDest = (destPos - myPos).length();
    
    if (distToDest < 50.0 || myId == destId) {
        // We are at destination!
        packetsDelivered++;
        deliveredCount[static_cast<RoutingMode>(wsm->getRoutingMode())]++;
        
        simtime_t delay = simTime() - wsm->getGenerationTime();
        EV << "DATA PACKET DELIVERED! Delay: " << delay 
           << "s, Hops: " << wsm->getHopCount() 
           << ", Distance: " << wsm->getTotalDistance() << "m" << endl;
        
        getParentModule()->bubble("PACKET DELIVERED!");
        return;
    }
    
    // Forward the packet
    int nextHop = -1;
    RoutingMode mode = static_cast<RoutingMode>(wsm->getRoutingMode());
    
    switch(mode) {
        case GREEDY:
            nextHop = selectNextHopGreedy();
            break;
        case LET_BASED:
            nextHop = selectNextHopLET();
            break;
        case STABILITY:
            nextHop = selectNextHopStability();
            break;
    }
    
    if (nextHop == -1) {
        // Cannot forward - drop packet
        EV << "Cannot forward DATA packet - DROPPED!" << endl;
        packetsDropped++;
        droppedCount[mode]++;
        getParentModule()->bubble("FORWARDING FAILED!");
        return;
    }
    
    // Create forwarded packet
    StabilityMessage* fwdMsg = wsm->dup();
    fwdMsg->setHopCount(wsm->getHopCount() + 1);
    
    // Update distance traveled
    Coord lastPos = wsm->getSenderPos();
    double segmentDist = (myPos - lastPos).length();
    fwdMsg->setTotalDistance(wsm->getTotalDistance() + segmentDist);
    
    // Update sender info for next hop
    fwdMsg->setSenderAddress(myId);
    fwdMsg->setSenderPos(myPos);
    fwdMsg->setSenderVelocity(myCurrentVelocity);
    
    sendDown(fwdMsg);
    
    EV << "Forwarding DATA packet to node " << nextHop << endl;
    
    char bubbleText[100];
    sprintf(bubbleText, "FWD->%d (hop %d)", nextHop, fwdMsg->getHopCount());
    getParentModule()->bubble(bubbleText);
}

void StabilityApp::updateNeighborTable(int senderId, Coord pos, Coord vel) {
    NeighborInfo& info = neighborTable[senderId];
    info.position = pos;
    info.velocity = vel;
    info.lastUpdate = simTime();
    info.let = calculateLET(info);
}

double StabilityApp::calculateLET(const NeighborInfo& neighbor) {
    // LET formula based on relative velocity and distance
    Coord myPos = mobility->getPositionAt(simTime());
    Coord relPos = neighbor.position - myPos;
    Coord relVel = neighbor.velocity - myCurrentVelocity;
    
    double distance = relPos.length();
    
    // If moving away from each other or parallel
    double relSpeed = relVel.length();
    if (relSpeed < 0.1) {
        return 999.0; // Very stable, nearly stationary relative to each other
    }
    
    // Calculate dot product to determine if approaching or separating
    double dotProduct = relPos.x * relVel.x + relPos.y * relVel.y;
    
    if (dotProduct >= 0) {
        // Moving apart - link will break soon
        double timeToBreak = (transmissionRange - distance) / relSpeed;
        return std::max(0.0, timeToBreak);
    } else {
        // Moving closer - calculate time until they pass and start separating
        double timeToClosest = -dotProduct / (relSpeed * relSpeed);
        double closestDist = (relPos + relVel * timeToClosest).length();
        
        if (closestDist > transmissionRange) {
            return 0.0; // Will never be in range
        }
        
        // Time in range
        double timeInRange = 2 * sqrt(transmissionRange * transmissionRange - closestDist * closestDist) / relSpeed;
        return timeToClosest + timeInRange;
    }
}

void StabilityApp::cleanupNeighborTable() {
    // Remove neighbors not heard from in last 5 seconds
    auto it = neighborTable.begin();
    while (it != neighborTable.end()) {
        if (simTime() - it->second.lastUpdate > 5.0) {
            it = neighborTable.erase(it);
        } else {
            ++it;
        }
    }
}

void StabilityApp::sendDataPacket() {
    // For demonstration: pick a random neighbor as destination
    // In a real scenario, this would be a fixed destination or RSU
    if (neighborTable.empty()) {
        EV << "No neighbors available. Cannot send packet." << endl;
        return;
    }
    
    // Select a destination from neighbors (furthest one to show multi-hop potential)
    Coord myPos = mobility->getPositionAt(simTime());
    int destNodeId = -1;
    double maxDist = 0;
    Coord destPos;
    
    for (auto& pair : neighborTable) {
        double dist = (pair.second.position - myPos).length();
        if (dist > maxDist) {
            maxDist = dist;
            destNodeId = pair.first;
            destPos = pair.second.position;
        }
    }
    
    if (destNodeId == -1) {
        EV << "Could not select destination." << endl;
        return;
    }
    
    destinationPos = destPos;
    
    // Create a data packet
    StabilityMessage* dataMsg = new StabilityMessage();
    populateWSM(dataMsg);
    
    // Set message type to DATA
    dataMsg->setMessageType(DATA);
    
    // Set sender info
    dataMsg->setSenderPos(myPos);
    dataMsg->setSenderVelocity(myCurrentVelocity);
    dataMsg->setSenderAddress(myId);
    
    // Set routing information
    dataMsg->setSourceId(myId);
    dataMsg->setDestinationId(destNodeId);
    dataMsg->setDestinationPos(destinationPos);
    dataMsg->setGenerationTime(simTime());
    dataMsg->setRoutingMode(static_cast<int>(currentMode));
    dataMsg->setHopCount(0);
    dataMsg->setTotalDistance(0.0);
    
    // Set payload
    char payload[100];
    sprintf(payload, "Message from Car%d to Car%d at %.2fs", myId, destNodeId, simTime().dbl());
    dataMsg->setDataString(payload);
    
    EV << "Created packet: " << payload << endl;
    
    // Select next hop based on routing mode
    int nextHop = -1;
    
    switch(currentMode) {
        case GREEDY:
            nextHop = selectNextHopGreedy();
            break;
        case LET_BASED:
            nextHop = selectNextHopLET();
            break;
        case STABILITY:
            nextHop = selectNextHopStability();
            break;
    }
    
    packetsSent++;
    sentCount[currentMode]++;
    
    if (nextHop == -1) {
        EV << "No suitable next hop found. Packet DROPPED." << endl;
        packetsDropped++;
        droppedCount[currentMode]++;
        getParentModule()->bubble("NO NEXT HOP!");
        delete dataMsg;
        return;
    }
    
    // Check if destination is the next hop (direct delivery)
    if (nextHop == destNodeId) {
        packetsDelivered++;
        deliveredCount[currentMode]++;
        EV << "Direct delivery to destination! Packet DELIVERED." << endl;
        getParentModule()->bubble("DIRECT DELIVERY!");
        delete dataMsg;
        return;
    }
    
    // Send the packet
    sendDown(dataMsg);
    
    std::string routingType;
    switch(currentMode) {
        case GREEDY: routingType = "GREEDY"; break;
        case LET_BASED: routingType = "LET"; break;
        case STABILITY: routingType = "STABILITY"; break;
    }
    
    EV << routingType << " routing: Sending to Car" << nextHop 
       << " (destination: Car" << destNodeId << ")" << endl;
    
    char bubbleText[100];
    sprintf(bubbleText, "%s->Car%d", routingType.c_str(), nextHop);
    getParentModule()->bubble(bubbleText);
}

int StabilityApp::selectNextHopGreedy() {
    // Select neighbor closest to destination
    Coord myPos = mobility->getPositionAt(simTime());
    double myDistToDest = (destinationPos - myPos).length();
    
    int bestNode = -1;
    double bestDist = myDistToDest;
    
    EV << "GREEDY: My distance to dest: " << myDistToDest << "m" << endl;
    
    for (auto& pair : neighborTable) {
        int nodeId = pair.first;
        NeighborInfo& info = pair.second;
        
        double nodeDist = (destinationPos - info.position).length();
        
        EV << "  Car" << nodeId << ": distance=" << nodeDist << "m";
        
        // Greedy: pick node that makes most progress toward destination
        if (nodeDist < bestDist) {
            EV << " -> BEST SO FAR" << endl;
            bestDist = nodeDist;
            bestNode = nodeId;
        } else {
            EV << " (no progress)" << endl;
        }
    }
    
    if (bestNode != -1) {
        EV << "GREEDY selected: Car" << bestNode << " (progress: " 
           << (myDistToDest - bestDist) << "m)" << endl;
    } else {
        EV << "GREEDY: No node makes progress!" << endl;
    }
    
    return bestNode;
}

int StabilityApp::selectNextHopLET() {
    // Select neighbor with highest LET that also makes progress
    Coord myPos = mobility->getPositionAt(simTime());
    double myDistToDest = (destinationPos - myPos).length();
    
    int bestNode = -1;
    double bestLET = 0.0;
    
    for (auto& pair : neighborTable) {
        int nodeId = pair.first;
        NeighborInfo& info = pair.second;
        
        double nodeDist = (destinationPos - info.position).length();
        
        // Must make progress toward destination
        if (nodeDist < myDistToDest && info.let > bestLET) {
            bestLET = info.let;
            bestNode = nodeId;
        }
    }
    
    return bestNode;
}

int StabilityApp::selectNextHopStability() {
    // Select stable neighbor closest to destination
    Coord myPos = mobility->getPositionAt(simTime());
    double myDistToDest = (destinationPos - myPos).length();
    
    int bestNode = -1;
    double bestDist = myDistToDest;
    
    EV << "STABILITY: Checking neighbors..." << endl;
    EV << "  My distance to dest: " << myDistToDest << "m" << endl;
    
    int stableCount = 0;
    int unstableCount = 0;
    
    for (auto& pair : neighborTable) {
        int nodeId = pair.first;
        NeighborInfo& info = pair.second;
        
        double nodeDist = (destinationPos - info.position).length();
        double relSpeed = (info.velocity - myCurrentVelocity).length();
        
        EV << "  Car" << nodeId << ": ";
        EV << "relSpeed=" << relSpeed << "m/s, ";
        EV << "distance=" << nodeDist << "m, ";
        EV << "stable=" << (info.isStable ? "YES" : "NO");
        
        // Only consider stable neighbors
        if (!info.isStable) {
            EV << " -> REJECTED (unstable)" << endl;
            unstableCount++;
            continue;
        }
        
        stableCount++;
        
        // Pick stable node that makes most progress
        if (nodeDist < bestDist) {
            EV << " -> SELECTED (best stable)" << endl;
            bestDist = nodeDist;
            bestNode = nodeId;
        } else {
            EV << " (no progress)" << endl;
        }
    }
    
    EV << "STABILITY: Found " << stableCount << " stable, " 
       << unstableCount << " unstable neighbors" << endl;
    
    // If no stable neighbor found, fallback to any neighbor
    if (bestNode == -1) {
        EV << "No stable neighbors, falling back to greedy..." << endl;
        return selectNextHopGreedy();
    }
    
    EV << "STABILITY selected: Car" << bestNode << " (stable & makes progress)" << endl;
    
    return bestNode;
}

void StabilityApp::finish() {
    MyVeinsApp::finish();
    
    // Output statistics
    EV << "=== ROUTING STATISTICS ===" << endl;
    EV << "Mode: ";
    switch(currentMode) {
        case GREEDY: EV << "GREEDY"; break;
        case LET_BASED: EV << "LET-BASED"; break;
        case STABILITY: EV << "STABILITY"; break;
    }
    EV << endl;
    
    EV << "Packets Sent: " << packetsSent << endl;
    EV << "Packets Delivered: " << packetsDelivered << endl;
    EV << "Packets Dropped: " << packetsDropped << endl;
    
    if (packetsSent > 0) {
        double deliveryRatio = (double)packetsDelivered / packetsSent * 100.0;
        double dropRatio = (double)packetsDropped / packetsSent * 100.0;
        EV << "Delivery Ratio: " << deliveryRatio << "%" << endl;
        EV << "Drop Ratio: " << dropRatio << "%" << endl;
    }
    
    // Record statistics
    recordScalar("packetsSent", packetsSent);
    recordScalar("packetsDelivered", packetsDelivered);
    recordScalar("packetsDropped", packetsDropped);
    
    if (packetsSent > 0) {
        recordScalar("deliveryRatio", (double)packetsDelivered / packetsSent);
        recordScalar("dropRatio", (double)packetsDropped / packetsSent);
    }
}
