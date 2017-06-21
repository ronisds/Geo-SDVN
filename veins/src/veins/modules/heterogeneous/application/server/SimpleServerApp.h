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

#ifndef SIMPLESERVER_H_
#define SIMPLESERVER_H_

#include <omnetpp.h>
#include "ApplicationBase.h"
#include "INETDefs.h"
#include "UDPSocket.h"
#include "IPv4Address.h"
#include "veins/modules/heterogeneous/messages/HeterogeneousMessage_m.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/base/utils/SimpleLogger.h"

#include <map>
#include <stack>

/**
 * @brief
 * An application that implements the Geo-SDVN protocol in controller.
 *  Some features are implemented in a limited way. For example, the vehicle that
 *  starts the broadcast need to be in the region of interest.
 *
 * @author Roniel Soares
 */

using Veins::TraCIScenarioManager;
using Veins::TraCIScenarioManagerAccess;

using namespace std;

// Entry for the Vehicle Table created with the beacons received.
// It is used to maintain information about the state of the network.
struct SDVNControllerVehicleEntry {
    string vehicleID;
    Coord vehiclePosition;
    simtime_t timestamp;

    SDVNControllerVehicleEntry() {
        vehicleID = "0";
        vehiclePosition = Coord();
        timestamp = simTime();
    }

    SDVNControllerVehicleEntry(const string& _vehicleID, const Coord& _vehiclePosition, const simtime_t& _timestamp) {
        vehicleID = _vehicleID;
        vehiclePosition = _vehiclePosition;
        timestamp = _timestamp;
    }

};

// Entry for the Forwarde Table.
// The controller stores informations about all flows created recently.
struct ForwardEntry {
    string senderID;
    int geocastID;
    Coord regionCenter;
    int regionRadius;
    simtime_t timestamp;
    vector<vector<string> > forwarders;
    map<string, vector<string> > forwardersNextHops;
    vector<int> componentsSize;
    int componentSizeFromSender = 1;

    ForwardEntry() { timestamp = simTime(); }

    ForwardEntry(const string& _senderID, const int& _geocastID, const Coord& _regionCenter = Coord(), const int& _regionRadius = 0) {
        timestamp = simTime();
        senderID = _senderID;
        geocastID = _geocastID;
        regionCenter = _regionCenter;
        regionRadius = _regionRadius;
    }

    void addForwardList(const vector<string>& forwardList) {
        vector<string> newVector(forwardList);
        forwarders.push_back(newVector);
    }

    bool operator==(const ForwardEntry& other) const {
        return senderID == other.senderID && geocastID == other.geocastID;
    }

    bool operator<(const ForwardEntry& other) const {
        return timestamp < other.timestamp;
    }
};

class SimpleServerApp: public ApplicationBase {
	protected:
        int transmissionRange = 280;

		UDPSocket socket;
		TraCIScenarioManager* manager;

		long receivedMessages;
		bool debug;
		bool infoLogging;

		int headerLengthBytes;
		double vehicleEntryDuration;
		map<string, SDVNControllerVehicleEntry> vehicleTable;
		map<string, vector<string> > adjacencyList;
		map<pair<string, int>, ForwardEntry> forwardTable;

		cMessage* updateVehicleTableMsg;


		// Statistics
		simsignal_t controllerOverheadIn;
		simsignal_t controllerOverheadOut;
		simsignal_t controllerGeocastControlOverheadIn;
		simsignal_t controllerGeocastControlOverheadOut;

		void updateVehicleTable();
		void updateAdjacencyList();
		void updateForwardTable();

		void treatActionRequest(string senderID, int matchIdentifier, Coord regionCenter, int regionRadius);
		void forwardMessage(HeterogeneousMessage *msg);
		void createForwardEntry(ForwardEntry e);
		void sendFlowTableUpdateForEntry(ForwardEntry e);
		void sendMessage(string destinationID, string messageName, string messageData);
		void forwardMessageToVehicle(HeterogeneousMessage *msg, string destinationAddress);

	public:
		SimpleServerApp();
		virtual ~SimpleServerApp();

		virtual int numInitStages() const {
			return 4;
		}
		virtual void initialize(int stage);
		virtual void finish();
		virtual void handleMessageWhenUp(cMessage *msg);

		virtual bool handleNodeStart(IDoneCallback *doneCallback);
		virtual bool handleNodeShutdown(IDoneCallback *doneCallback);
		virtual void handleNodeCrash();
};

#endif /* SIMPLESERVER_H_ */
