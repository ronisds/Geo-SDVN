/*
 * FlowTable.h
 *
 *  Created on: Nov 17, 2016
 *      Author: roniel
 */


#ifndef FLOWTABLE_H_
#define FLOWTABLE_H_

#include <utility>
#include "veins/base/utils/Coord.h"
#include "veins/base/utils/SimpleLogger.h"

using namespace std;

enum FlowAction {
    Drop,
    SendToNextHop,
    SendToSDVNController,
    SendToNextHopAndSDVNController
};

class FlowTable {
protected:
    struct FlowEntry {

        // Match fields
        string senderID;
        int matchIdentifier;

        // Actions
        Coord regionCenter;
        int regionRadius; // In meters
        FlowAction action;

        // List of neighbors I need to ensure they will also broadcast the message
        vector<string> nextHopIds;

        simtime_t timestamp;

        FlowEntry() {
            senderID = "";
            matchIdentifier = 0;
            regionCenter = Coord();
            regionRadius = 0.0;
            timestamp = simTime();
        }

        FlowEntry(const string& _senderID, const int& _matchIdentifier, const Coord& _regionCenter = Coord(), const int& _regionRadius = 0, const FlowAction& _action = Drop, const vector<string>& _nextHopIds = vector<string>()) {
            senderID = _senderID;
            matchIdentifier = _matchIdentifier;
            regionCenter = _regionCenter;
            regionRadius = _regionRadius;
            action = _action;
            nextHopIds = _nextHopIds;
            timestamp = simTime();
        }


        bool operator==(const FlowEntry& other) const {
            return senderID == other.senderID && matchIdentifier == other.matchIdentifier;
        }

        bool operator<(const FlowEntry& other) const {
            return string(senderID + to_string(matchIdentifier)) < string(other.senderID + to_string(other.matchIdentifier));
        }
    };

    set<FlowEntry> table;

public:
    FlowTable();
    virtual ~FlowTable();

    bool hasMatch(string senderID, int matchIdentifier);
    pair<FlowAction, vector<string> > getAction(string senderID, int matchIdentifier);
    void setEntry(string senderID, int matchIdentifier, Coord regionCenter, int regionRadius, FlowAction action, vector<string> nextHopIds);
    void removeEntry(string senderID, int matchIdentifier);
    void refreshTable(double timeout = 5);
};

#endif /* FLOWTABLE_H_ */
