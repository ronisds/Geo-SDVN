package org.car2x.veins.modules.heterogeneous.application;

//
// A simple application that just sends messages.
//
simple SimpleApp {
	parameters:
		bool debug = default(false);
		bool infoLogging = default(false);
		
		int headerLengthBytes = default(17byte) @unit(byte);
		int beaconLengthBytes = default(8byte) @unit(byte);
		double maxOffset = default(0.005s) @unit(s);
		
		double beaconToSDVNControllerInterval = default(1s) @unit(s);
		
		double maximumWaitingTime = default(0.1s) @unit(s);
		int maximumTries = default(1);
		
		double tablemissMinimumWaitingTime = default(0.1s) @unit(s);
		double tablemissAdditionalWaitingTime = default(0.2s) @unit(s);
		int tablemissMaximumTries = default(3);
		
    gates:
        output toDecisionMaker;
        input  fromDecisionMaker;
}
