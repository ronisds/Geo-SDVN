/*
 * FlowTable.cc
 *
 *  Created on: Nov 17, 2016
 *      Author: roniel
 */

#include "FlowTable.h"

FlowTable::FlowTable() {
    // TODO Auto-generated constructor stub

}

FlowTable::~FlowTable() {
    // TODO Auto-generated destructor stub
}

bool FlowTable::hasMatch(string senderID, int matchIdentifier) {
    FlowEntry entry(senderID, matchIdentifier);
    return ((int)table.count(entry) > 0);
}

pair<FlowAction, vector<string> > FlowTable::getAction(string senderID, int matchIdentifier) {
    set<FlowEntry>::iterator it = table.find(FlowEntry(senderID, matchIdentifier));
    if (it != table.end()) {
        return make_pair(it->action, it->nextHopIds);
    }
    else return make_pair(Drop, vector<string>());
}

void FlowTable::setEntry(string senderID, int matchIdentifier, Coord regionCenter, int regionRadius, FlowAction action, vector<string> nextHopIds) {
    set<FlowEntry>::iterator it = table.find(FlowEntry(senderID, matchIdentifier));
    if (it != table.end()) table.erase(it);
    table.insert(FlowEntry(senderID, matchIdentifier, regionCenter, regionRadius, action, nextHopIds));
}

void FlowTable::removeEntry(string senderID, int matchIdentifier) {
    FlowEntry entry(senderID, matchIdentifier);
    table.erase(entry);
}

void FlowTable::refreshTable(double timeout) {
    set<FlowEntry>::iterator nextIt;
    for(set<FlowEntry>::iterator it = table.begin(); it != table.end();it = nextIt) {
        nextIt = it;
        ++nextIt;
        if (it->timestamp + timeout < simTime())
            table.erase(it);
    }
}
