// StabilityMessage.msg
import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;

namespace veins;

// Message types
enum MessageType {
    BEACON = 0;      // Regular beacon message
    DATA = 1;        // Data packet for routing
}

// We extend the standard frame to add our custom fields
packet StabilityMessage extends BaseFrame1609_4 {
    // Basic vehicle information
    Coord senderVelocity;
    Coord senderPos;
    int senderAddress = -1;        // Sender node ID
    
    // Message type
    int messageType = BEACON;
    
    // Data packet fields (used when messageType == DATA)
    int hopCount = 0;              // Number of hops taken
    int sourceId = -1;             // Original sender ID
    int destinationId = -1;        // Target destination ID
    Coord destinationPos;          // Destination position
    simtime_t generationTime = 0;  // When packet was created
    int routingMode = 0;           // 0=GREEDY, 1=LET, 2=STABILITY
    string dataString;             // Payload data string
    
    // Statistics fields
    double totalDistance = 0.0;    // Total distance traveled
    int droppedAttempts = 0;       // Number of times forwarding failed
}
