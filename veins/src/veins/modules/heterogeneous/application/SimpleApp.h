//
// Copyright (C) 2006-2016 Florian Hagenauer <hagenauer@ccs-labs.org>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef __VEINS_SIMPLEAPP_H_
#define __VEINS_SIMPLEAPP_H_

#include <omnetpp.h>
#include "veins/modules/heterogeneous/messages/HeterogeneousMessage_m.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/base/utils/SimpleLogger.h"

#include "FlowTable.h"

using Veins::TraCIMobility;
using Veins::TraCIScenarioManager;
using Veins::TraCIScenarioManagerAccess;

using namespace std;

/**
 * @brief
 * An application that implements the Geo-SDVN protocol in cars. Also choose one random vehicle to broadcast periodically a message using the Geo-SDVN protocol.
 *
 *
 * @author Roniel Soares
 */

class SimpleApp: public cSimpleModule {
public:
    static int messageCount;
protected:

    /*
     * ----------------- Miscellaneous -----------------
     */
    TraCIMobility *mobility;
    int toDecisionMaker;
    int fromDecisionMaker;
    std::string sumoId;
    bool debug;
    bool infoLogging;
    int headerLengthBytes;
    int beaconLengthBytes;

    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);


    /*
     * ----------------- Used only for testing purpose -----------------
     *
     * One random vehicle will be chosen to periodically broadcast a message using the Geo-SDVN protocol.
     * The message is sent to a region defined by "regionCenterToSent" and "regionRadiusToSent" and has
     * a fixed identifier "geocastIdenfier".
     * "sendGeocastMsg" is the self-message used to perform the action.
     */
    static const int geocastIdenfier = 5;
    Coord regionCenterToSent;
    int regionRadiusToSent;
    cMessage* sendGeocastMsg;

    /*
     * ----------------- Geo-SDVN -----------------
     */

    // Stores the message text the vehicle is actually trying to send.
    // Used while the vehicle is waiting for the controller response about how to send the message.
    // It is empty if the vehicle has already sent the message.
    string waitingControllerResponseForMessage;


    set<int> treatedMessages;                       // Messages IDs already treated by the vehicle (rebroadcasted or not).
	double beaconToSDVNControllerInterval;          // default is 1 second.
	cMessage* sendBeaconToSDVNControllerMsg;        // Self-message to periodically send beacons to the SDVN-Controller.
	cMessage* tablemissMinimumTimeOutMsg;           // Self-message responsible for firing the Table-tablemissMinimumWaitingTime timeout.
	cMessage* tablemissAdditionalTimeOutMsg;        // Self-message responsible for firing the Table-tablemissMinimumWaitingTime timeout.

	FlowTable flowTable;

	// Protocol parameters
	double maximumWaitingTime;
	int maximumTries;
	double tablemissMinimumWaitingTime;
	double tablemissAdditionalWaitingTime;
	int tablemissMaximumTries;

	int geocastCurrentTry = 0;                      // Counter for the current value of the MaximumTries
	int tablemissCurrentTry = 0;                    // Counter for the current value of the Table-missMaximumTries
	HeterogeneousMessage *messageToSend;            // Message that is being sent
	cMessage* broadcastTimeOutMsg;                  // Self-message responsible for firing maximumWaitingTime timeout.
	vector<string> waitingCurrentNextHops;          // Stores the vehicles IDs of the NextHopIDs subset that has not rebroadcasted the message yet.



	void treatMessage(HeterogeneousMessage *msg);                           // Used to treat the message (check if need to broadacast, drop, etc)
	bool geocastMessage(string msg, Coord regionCenter, int regionRadius);  // Broadcast the message (or fire a table-miss if needed)
	void requestAction(Coord regionCenter, int regionRadius);               // Send a table-miss to the SDVN controller

	// Helper methods
	void sendBeaconToSDVNController();
	void sendMessageToServer(HeterogeneousMessage *msg);
	void sendBroadcastMessage(HeterogeneousMessage *msg, vector<string> nextHops = vector<string>());

};

int SimpleApp::messageCount = 0;

#endif
