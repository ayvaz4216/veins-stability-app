package org.car2x.veins.modules.application.traci;

import org.car2x.veins.modules.application.traci.MyVeinsApp;

simple StabilityApp extends MyVeinsApp
{
    parameters:
        @class(veins::StabilityApp);
        
        // Routing mode: 0=GREEDY, 1=LET_BASED, 2=STABILITY
        int routingMode = default(2);
        
        // Transmission range in meters
        double transmissionRange @unit(m) = default(300m);
        
        @signal[packetSent](type=long);
        @signal[packetDelivered](type=long);
        @signal[packetDropped](type=long);
        
        @statistic[packetSent](record=count,vector);
        @statistic[packetDelivered](record=count,vector);
        @statistic[packetDropped](record=count,vector);
}