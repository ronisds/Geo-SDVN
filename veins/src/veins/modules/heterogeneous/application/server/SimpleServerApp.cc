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

#include "SimpleServerApp.h"

#include "StationaryMobility.h"

Define_Module(SimpleServerApp);

SimpleServerApp::SimpleServerApp() {}

SimpleServerApp::~SimpleServerApp() {}

void SimpleServerApp::initialize(int stage){
	ApplicationBase::initialize(stage);
	if(stage == 0){
		debug = par("debug").boolValue();
		infoLogging = par("infoLogging").boolValue();
		receivedMessages = 0;
		manager = TraCIScenarioManagerAccess().get();
		ASSERT(manager);

		headerLengthBytes = par("headerLengthBytes").longValue();
		vehicleEntryDuration = par("vehicleEntryDuration").doubleValue();
		transmissionRange = 300 - par("guardRange").longValue();

		updateVehicleTableMsg = new cMessage("updateVehicleTable");

		controllerOverheadIn = registerSignal("ControllerOverheadIn");
		controllerOverheadOut = registerSignal("ControllerOverheadOut");
		controllerGeocastControlOverheadIn = registerSignal("ControllerGeocastControlOverheadIn");
		controllerGeocastControlOverheadOut = registerSignal("ControllerGeocastControlOverheadOut");
		// Update vehicle table every 1 second.
		scheduleAt(simTime() + 1, updateVehicleTableMsg);

	}
}

void SimpleServerApp::finish(){
	INFO_ID("Received " << receivedMessages << " messages via LTE.");
}

void SimpleServerApp::handleMessageWhenUp(cMessage *msg){
    if (msg->isSelfMessage()) {
        if (msg == updateVehicleTableMsg) {
            updateVehicleTable();
            updateAdjacencyList();
            updateForwardTable();
            scheduleAt(simTime() + 1, updateVehicleTableMsg);
        }

        return;
    }

	HeterogeneousMessage* heterogeneousMessage = dynamic_cast<HeterogeneousMessage*>(msg);

	if(heterogeneousMessage){
		receivedMessages++;
		if (string(heterogeneousMessage->getName()) == "BeaconToSDVNControllerMsg") {
		    string vehicleID = heterogeneousMessage->getSourceAddress();
		    Coord vehiclePosition = heterogeneousMessage->getSenderPos();
		    vehicleTable[vehicleID] = SDVNControllerVehicleEntry(vehicleID, vehiclePosition, simTime());
//		    INFO_ID("Controlador recebeu beacon de " << vehicleID << "(" << to_string(vehiclePosition.x) << "," << to_string(vehiclePosition.y) << ")");
		    return;
		}

		if (string(heterogeneousMessage->getName()) == "actionRequest") {
		    emit(controllerGeocastControlOverheadIn, 1);
		    char messageData[100];
		    strcpy(messageData, heterogeneousMessage->getWsmData());
		    char *token;
		    token = strtok(messageData, "|");
		    vector<string> elements;
		    while(token != NULL) {
		        elements.push_back(string(token));
		        token = strtok(NULL, "|");
		    }
		    INFO_ID("Action request received with data: " << heterogeneousMessage->getWsmData() << " creation time = " << heterogeneousMessage->getCreationTime() << " arrival time = " << heterogeneousMessage->getArrivalTime());

		    ASSERT2(elements.size() == 6, "Action request with wrong number of items (" + to_string((int)elements.size()) + ")");

		    string senderID = elements[0];
		    int matchIdentifier = atoi(elements[1].c_str());
		    double x = atof(elements[2].c_str());
		    double y = atof(elements[3].c_str());
		    double z = atof(elements[4].c_str());
		    Coord regionCenter(x,y,z);
		    int regionRadius = atoi(elements[5].c_str());

		    treatActionRequest(senderID, matchIdentifier, regionCenter, regionRadius);

		    return;
        }

		if (string(heterogeneousMessage->getName()) == "data") {
		    emit(controllerOverheadIn, 1);
		    INFO_ID("Message received from " << heterogeneousMessage->getSenderAddress() << " to forward");
		    forwardMessage(heterogeneousMessage);
		    return;
		}



//
//        /*
//         * Server replies with a simple message. Note that no additional parameters (like exact
//         * message size) are set and therefore transmission will more likely succeed. If you use
//         * this function set it correctly to get realistic results.
//         */
//        HeterogeneousMessage *reply = new HeterogeneousMessage("Server Reply");
//        IPv4Address address = manager->getIPAddressForID(sourceAddress);
//        reply->setSourceAddress("server");
//        INFO_ID("Sending Message back to " << address);
//        socket.sendTo(reply, address, 4242);
	}
	delete msg;
}

bool SimpleServerApp::handleNodeStart(IDoneCallback *doneCallback){
    socket.setOutputGate(gate("udpOut"));
    int localPort = par("localPort");
    socket.bind(localPort);
    return true;
}

bool SimpleServerApp::handleNodeShutdown(IDoneCallback *doneCallback){
    return true;
}

void SimpleServerApp::handleNodeCrash(){}


void SimpleServerApp::updateVehicleTable() {
    map<string, SDVNControllerVehicleEntry>::iterator nextIt;
    int cnt1 = vehicleTable.size();
    for(map<string, SDVNControllerVehicleEntry>::iterator it = vehicleTable.begin(); it != vehicleTable.end();it = nextIt) {
        nextIt = it;
        ++nextIt;
        SDVNControllerVehicleEntry entry = it->second;
        if (entry.timestamp + vehicleEntryDuration < simTime())
            vehicleTable.erase(it);
    }
    INFO_ID("Quantidade de veiculos: " << cnt1 << " " << vehicleTable.size());
}

void SimpleServerApp::updateAdjacencyList() {
    adjacencyList.clear();
    int cnt = 0;
    for(map<string, SDVNControllerVehicleEntry>::iterator it = vehicleTable.begin(); it != vehicleTable.end();it++) {
        map<string, SDVNControllerVehicleEntry>::iterator it2 = it;
        ++it2;
        for(; it2 != vehicleTable.end();it2++) {
            if (it->second.vehiclePosition.distance(it2->second.vehiclePosition) < transmissionRange) {
                adjacencyList[it->first].push_back(it2->first);
                adjacencyList[it2->first].push_back(it->first);
                ++cnt;
            }
        }
    }
}

// REMOVER AS ENTRADAS ANTIGAS
// VERIFICAR SE ALGUMA ENTRADA PRECISA MODIFICAR OS FORWARDERS, E AVISAR OS VEICULOS QUE PRECISAREM MODIFICAR A TABELA
// TESTAR VARIAR A FREQUENCIA DE CHAMADAS DESSE METODO
// IMPLEMENTACAO BASICA POR ENQUANTO, PRECISA REVER, PRECISA PERMITIR QUE O SENDER ESTEJA FORA DA REGIAO
// Currently implementation only check and send the Table-missResponse messages to the vehicles that needs the update
void SimpleServerApp::updateForwardTable() {
    map<pair<string, int>, ForwardEntry>::iterator nextIt;
    for(map<pair<string, int>, ForwardEntry>::iterator it = forwardTable.begin(); it != forwardTable.end(); it = nextIt) {
        nextIt = it;
        ++nextIt;
        if (vehicleTable.count(it->second.senderID) == 0) continue;
        set<string> oldForwarders;
        for(int i = 0; i < (int)it->second.forwarders.size(); ++i) {
            if (it->second.componentsSize[i] > 1)
                oldForwarders.insert(it->second.forwarders[i].begin(), it->second.forwarders[i].end());
        }

        int haveBefore = it->second.forwarders.size(), oldComponentSizeFromSender = it->second.componentSizeFromSender;
        map<string, set<string> > oldForwardersNextHops;
        set<string> sentTo;
        for(map<string, vector<string> >::iterator it2 = it->second.forwardersNextHops.begin(); it2 != it->second.forwardersNextHops.end(); ++it2) {
            oldForwardersNextHops[it2->first].insert(it2->second.begin(), it2->second.end());
        }

        createForwardEntry(it->second);

        ForwardEntry newEntry = forwardTable[make_pair(it->second.senderID, it->second.geocastID)];

        int haveNow = newEntry.forwarders.size();

        set<string> newForwarders;
        for(int i = 0; i < (int)newEntry.forwarders.size(); ++i) {
            if (newEntry.componentsSize[i] > 1)
                newForwarders.insert(newEntry.forwarders[i].begin(), newEntry.forwarders[i].end());
        }
        INFO_ID("ForwardEntry " << it->first.first << " " << it->first.second << ": " << oldForwarders.size() << " " << newForwarders.size());
        for(set<string>::iterator it2 = oldForwarders.begin(); it2 != oldForwarders.end(); ++it2) {
            if (newForwarders.count(*it2) == 0) {
                string messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|0|0|";
                sentTo.insert(*it2);
                sendMessage(*it2, "actionResponse", messageData);
            }
        }
        for(set<string>::iterator it2 = newForwarders.begin(); it2 != newForwarders.end(); ++it2) {
            if (*it2 != newEntry.senderID && oldForwarders.count(*it2) == 0) {
                string messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|1|";
                vector<string> nextHops = newEntry.forwardersNextHops[*it2];
                messageData += to_string(nextHops.size()) + "|";
                for(int k = 0; k < nextHops.size(); ++k)
                    messageData += nextHops[k] + "|";
                sentTo.insert(*it2);
                sendMessage(*it2, "actionResponse", messageData);
            }
        }

        if ((oldComponentSizeFromSender == 1 && newEntry.componentSizeFromSender > 1) || (oldComponentSizeFromSender > 1 && newEntry.componentSizeFromSender == 1)) {
            if (haveNow == 1) {
                string messageData;
                if (newEntry.componentSizeFromSender == 1) messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|0|0|";
                else {
                    messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|1|";
                    vector<string> nextHops = newEntry.forwardersNextHops[newEntry.senderID];
                    messageData += to_string(nextHops.size()) + "|";
                    for(int k = 0; k < nextHops.size(); ++k)
                        messageData += nextHops[k] + "|";
                }
                sentTo.insert(newEntry.senderID);
                sendMessage(newEntry.senderID, "actionResponse", messageData);
            } else {
                string messageData;
                if (newEntry.componentSizeFromSender == 1) messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|2|0|";
                else {
                    messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|3|";
                    vector<string> nextHops = newEntry.forwardersNextHops[newEntry.senderID];
                    messageData += to_string(nextHops.size()) + "|";
                    for(int k = 0; k < nextHops.size(); ++k)
                        messageData += nextHops[k] + "|";
                }
                sentTo.insert(newEntry.senderID);
                sendMessage(newEntry.senderID, "actionResponse", messageData);
            }
        } else {
            if (haveBefore == 1 && haveNow > 1) {
                string messageData;
                if (newEntry.componentSizeFromSender == 1) messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|2|0|";
                else {
                    messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|3|";
                    vector<string> nextHops = newEntry.forwardersNextHops[newEntry.senderID];
                    messageData += to_string(nextHops.size()) + "|";
                    for(int k = 0; k < nextHops.size(); ++k)
                        messageData += nextHops[k] + "|";
                }
                sentTo.insert(newEntry.senderID);
                sendMessage(newEntry.senderID, "actionResponse", messageData);
            }
            if (haveBefore > 1 && haveNow == 1) {
                string messageData;
                if (newEntry.componentSizeFromSender == 1) messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|0|0|";
                else {
                    messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|1|";
                    vector<string> nextHops = newEntry.forwardersNextHops[newEntry.senderID];
                    messageData += to_string(nextHops.size()) + "|";
                    for(int k = 0; k < nextHops.size(); ++k)
                        messageData += nextHops[k] + "|";
                }
                sentTo.insert(newEntry.senderID);
                sendMessage(newEntry.senderID, "actionResponse", messageData);
            }
        }
        for(map<string, set<string> >::iterator it2 = oldForwardersNextHops.begin(); it2 != oldForwardersNextHops.end(); ++it2) {
            if (sentTo.count(it2->first) == 0 && newEntry.forwardersNextHops.count(it2->first) > 0) {
                vector<string> newNextHops = newEntry.forwardersNextHops[it2->first];
                bool isDiff = it2->second.size() != newNextHops.size();
                if (!isDiff) {
                    for(int i = 0; i < newNextHops.size(); ++i) {
                        if (it2->second.count(newNextHops[i]) == 0) {
                            isDiff = true;
                            break;
                        }
                    }
                }
                if (isDiff) {
                    string messageData;
                    if (it2->first == newEntry.senderID && newEntry.forwarders.size() > 1) {
                        messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|3|";
                    } else {
                        messageData = newEntry.senderID + "|" + to_string(newEntry.geocastID) + "|" + to_string(newEntry.regionCenter.x) + "|" + to_string(newEntry.regionCenter.y) + "|" + to_string(newEntry.regionCenter.z) + "|" + to_string(newEntry.regionRadius) + "|1|";
                    }
                    vector<string> nextHops = newEntry.forwardersNextHops[it2->first];
                    messageData += to_string(nextHops.size()) + "|";
                    for(int k = 0; k < nextHops.size(); ++k)
                        messageData += nextHops[k] + "|";
                    sendMessage(it2->first, "actionResponse", messageData);
                }

            }
        }
    }

}

// Deveria verificar se ja existe entrada pra essa request
// Se existe, enviar acao necessaria apenas para o senderID
// Se nao existe, criar entrada e enviar as acoes necessarias para todos os envolvidos
// ESTA IMPLEMENTADO APENAS DE FORMA PARCIAL PARA AS PRIMEIRAS SIMULACOES
void SimpleServerApp::treatActionRequest(string senderID, int matchIdentifier, Coord regionCenter, int regionRadius) {
    ForwardEntry entry(senderID, matchIdentifier, regionCenter, regionRadius);
    createForwardEntry(entry);
    sendFlowTableUpdateForEntry(entry);

}

void SimpleServerApp::forwardMessage(HeterogeneousMessage *msg) {
    string sourceAddress = msg->getGeocastSourceAddress();
    int geocastID = msg->getGeocastIdenfier();
    if (forwardTable.count(make_pair(sourceAddress, geocastID))) {
        ForwardEntry entry = forwardTable[make_pair(sourceAddress, geocastID)];
        for(int i = 0; i < (int)entry.forwarders.size(); ++i) {
            if ((int)entry.forwarders[i].size() == 0 || entry.forwarders[i][0] == sourceAddress)
                continue;
            string destination = entry.forwarders[i][0];
            forwardMessageToVehicle(msg->dup(), destination);
        }
    }
}


void SimpleServerApp::createForwardEntry(ForwardEntry e) {
    set<string> allVehiclesInRegion;
    string allVehiclesString, allVehiclesStringWithPos;
    for(map<string, SDVNControllerVehicleEntry>::iterator it = vehicleTable.begin(); it != vehicleTable.end(); ++it) {
        allVehiclesStringWithPos += it->first + "(" + to_string(it->second.vehiclePosition.x) + "," + to_string(it->second.vehiclePosition.y) + "," + to_string(it->second.vehiclePosition.z) + ") ";
        if (it->second.vehiclePosition.distance(e.regionCenter) <= e.regionRadius) {
            allVehiclesInRegion.insert(it->first);
            allVehiclesString += it->first + "(" + to_string(it->second.vehiclePosition.x) + "," + to_string(it->second.vehiclePosition.y) + "," + to_string(it->second.vehiclePosition.z) + ") ";
        }
    }
    INFO_ID("Regiao (" << e.regionCenter.x << "," << e.regionCenter.y << ") - " << e.regionRadius);
    INFO_ID("Forwarders ( " << e.senderID << ", " << e.geocastID << "): Quantidade de veiculos = " << allVehiclesInRegion.size());
    INFO_ID("Lista: " << allVehiclesString);
    INFO_ID("Todos veiculos: " << allVehiclesStringWithPos);
    e.forwarders.clear();
    e.componentsSize.clear();
    int forwardersCount = 0;
    set<string> visited;
    for(set<string>::iterator it = allVehiclesInRegion.begin(); it != allVehiclesInRegion.end(); ++it) {
        if (visited.count(*it) != 0)
            continue;


        set<string> V; // Currenct connected component

        queue<string> s;
        s.push(*it);
        V.insert(*it);

        while(!s.empty()) {
            string u = s.front();
            s.pop();
            vector<string> neighbors = adjacencyList[u];
            for(int i = 0; i < (int)neighbors.size(); ++i) {
                if (allVehiclesInRegion.count(neighbors[i]) == 0) continue;
                if (V.count(neighbors[i]) == 0) {
                    V.insert(neighbors[i]);
                    s.push(neighbors[i]);
                }
            }
        }
        visited.insert(V.begin(),V.end());

        // Algorithm from "Recent Developments in Coopeerative Control and Optimization" (chapter 4)
        // Modificated because need e.senderID in CDS
        set<string> D(V), F;

        if (D.count(e.senderID) > 0)
            F.insert(e.senderID);
        INFO_ID("Tamanho do componente " << V.size());
        INFO_ID("Tamanho do componente inicial " << D.size());
        INFO_ID("Componente possui veiculo inicial " << F.size());
        while(D.size() > 1 && D.size() != F.size()) {
            //Calculate the degree of all vertices of D - F
            map<string, int> degree;
            for(set<string>::iterator it2 = D.begin(); it2 != D.end(); ++it2) {
                if (adjacencyList.count(*it2) == 0 || F.count(*it2) > 0) continue;
                vector<string> neighbors = adjacencyList[*it2];
                int count = 0;
                for(int i = 0; i < (int)neighbors.size(); ++i)
                    if (D.count(neighbors[i]) > 0)
                        ++count;
                degree[*it2] = count;
            }
            //Find the vertex with minimum degree
            int minDegree = 123456789;
            string u;
            for(map<string, int>::iterator it2 = degree.begin(); it2 != degree.end(); ++it2)
                if (it2->second < minDegree) {
                    minDegree = it2->second;
                    u = it2->first;
                }
            //Check if D - {u} is connected
            queue<string> q;
            set<string> vis;
            for(set<string>::iterator it2 = D.begin(); it2 != D.end(); ++it2) // Get any vertex in D - {u} to start BFS
                if (*it2 != u) {
                    q.push(*it2);
                    vis.insert(*it2);
                    break;
                }

            while(!q.empty()) {
                string uu = q.front();
                q.pop();
                vector<string> neighbors = adjacencyList[uu];
                for(int i = 0; i < (int)neighbors.size(); ++i) {
                    string v = neighbors[i];
                    if (v != u && D.count(v) > 0 && vis.count(v) == 0) {
                        vis.insert(v);
                        q.push(v);
                    }
                }
            }
            if ((int)vis.size() != (int)D.size() - 1) { // Not connected
                F.insert(u);
            } else { // Still connected
                D.erase(u);
                bool hasFixed = false;
                int bestDegree = -1;
                string w;
                for(int i = 0; i < (int)adjacencyList[u].size(); ++i) {
                    if (D.count(adjacencyList[u][i]) > 0) {
                        if (F.count(adjacencyList[u][i]) > 0) hasFixed = true;
                        int curDegree = degree[adjacencyList[u][i]];
                        if (curDegree > bestDegree) {
                            curDegree = bestDegree;
                            w = adjacencyList[u][i];
                        }
                    }
                }
               if (!hasFixed) {
                   F.insert(w);
               }
            }

        }
        INFO_ID("Tamanho do componente final " << D.size() << " " << F.size());
        vector<string> way;
        if (D.count(e.senderID) > 0) way.push_back(e.senderID);
        for(set<string>::iterator it2 = D.begin(); it2 != D.end(); ++it2)
            if (*it2 != e.senderID)
                way.push_back(*it2);

        string stringList, componentStr;
        forwardersCount += way.size();
        for(int i = 0; i < (int)way.size(); ++i)
            stringList += way[i] + " ";
        for(set<string>::iterator it2 = V.begin(); it2 != V.end(); ++it2)
            componentStr += *it2 + " ";
        INFO_ID(">>>:" << D.size() << ": " << stringList);
        INFO_ID(">>>:" << V.size() << ": " << componentStr);
        e.forwarders.push_back(way);
        e.componentsSize.push_back(V.size());
        if (V.count(e.senderID) > 0) e.componentSizeFromSender = V.size();
    }


    // Make next hops...
    e.forwardersNextHops.clear();
    for(int i = 0; i < e.forwarders.size(); ++i) {

        if (e.componentsSize[i] > 1) {
            set<string> currentComponent(e.forwarders[i].begin(), e.forwarders[i].end());
            stack<string> st;
            set<string> vis;

            st.push(e.forwarders[i].front());
            vis.insert(e.forwarders[i].front());

            while(!st.empty()) {
                string cur = st.top();
                st.pop();
                for(int j = 0; j < (int)adjacencyList[cur].size(); ++j) {
                    string nextHop = adjacencyList[cur][j];
                    if (currentComponent.count(nextHop) > 0 && vis.count(nextHop) == 0) {
                        e.forwardersNextHops[cur].push_back(nextHop);
                        vis.insert(nextHop);
                        st.push(nextHop);
                    }
                }
            }
        }
    }

    INFO_ID("Total de forwarders: " << forwardersCount);
    forwardTable[make_pair(e.senderID, e.geocastID)] = e;

}

void SimpleServerApp::sendFlowTableUpdateForEntry(ForwardEntry e) {
    if (forwardTable.count(make_pair(e.senderID, e.geocastID)) == 0) return;
    ForwardEntry entry = forwardTable[make_pair(e.senderID, e.geocastID)];
    string from;

    set<string> allVehiclesInRegion;
    for(map<string, SDVNControllerVehicleEntry>::iterator it = vehicleTable.begin(); it != vehicleTable.end(); ++it) {
        if (it->second.vehiclePosition.distance(e.regionCenter) <= e.regionRadius)
            allVehiclesInRegion.insert(it->first);
    }
    for(int i = 0; i < (int)entry.forwarders.size(); ++i) {
        for(int j = 0; j < (int)entry.forwarders[i].size(); ++j) {
            from = entry.forwarders[i][j];
            if (from == entry.senderID || (int)entry.componentsSize[i] == 1) continue;
            allVehiclesInRegion.erase(from);
            string messageData = entry.senderID + "|" + to_string(entry.geocastID) + "|" + to_string(entry.regionCenter.x) + "|" + to_string(entry.regionCenter.y) + "|" + to_string(entry.regionCenter.z) + "|" + to_string(entry.regionRadius) + "|1|";
            vector<string> nextHops = entry.forwardersNextHops[from];
            messageData += to_string(nextHops.size()) + "|";
            for(int k = 0; k < nextHops.size(); ++k)
                messageData += nextHops[k] + "|";
            sendMessage(from, "actionResponse", messageData);
        }
    }
    allVehiclesInRegion.erase(entry.senderID);
    for(set<string>::iterator it = allVehiclesInRegion.begin(); it != allVehiclesInRegion.end(); ++it) {
        string messageData = entry.senderID + "|" + to_string(entry.geocastID) + "|" + to_string(entry.regionCenter.x) + "|" + to_string(entry.regionCenter.y) + "|" + to_string(entry.regionCenter.z) + "|" + to_string(entry.regionRadius) + "|0|0|";
        sendMessage(*it, "actionResponse", messageData);
    }

    if ((int)entry.forwarders.size() > 1) {
        string messageData;
        if (entry.componentSizeFromSender == 1) messageData = entry.senderID + "|" + to_string(entry.geocastID) + "|" + to_string(entry.regionCenter.x) + "|" + to_string(entry.regionCenter.y) + "|" + to_string(entry.regionCenter.z) + "|" + to_string(entry.regionRadius) + "|2|0|";
        else {
            messageData = entry.senderID + "|" + to_string(entry.geocastID) + "|" + to_string(entry.regionCenter.x) + "|" + to_string(entry.regionCenter.y) + "|" + to_string(entry.regionCenter.z) + "|" + to_string(entry.regionRadius) + "|3|";
            vector<string> nextHops = entry.forwardersNextHops[e.senderID];
            messageData += to_string(nextHops.size()) + "|";
            for(int k = 0; k < nextHops.size(); ++k)
                messageData += nextHops[k] + "|";
        }

        sendMessage(entry.senderID, "actionResponse", messageData);
    } else {
        string messageData;
        if (entry.componentSizeFromSender == 1) messageData = entry.senderID + "|" + to_string(entry.geocastID) + "|" + to_string(entry.regionCenter.x) + "|" + to_string(entry.regionCenter.y) + "|" + to_string(entry.regionCenter.z) + "|" + to_string(entry.regionRadius) + "|0|0|";
        else {
            messageData = entry.senderID + "|" + to_string(entry.geocastID) + "|" + to_string(entry.regionCenter.x) + "|" + to_string(entry.regionCenter.y) + "|" + to_string(entry.regionCenter.z) + "|" + to_string(entry.regionRadius) + "|1|";
            vector<string> nextHops = entry.forwardersNextHops[e.senderID];
            messageData += to_string(nextHops.size()) + "|";
            for(int k = 0; k < nextHops.size(); ++k)
                messageData += nextHops[k] + "|";
        }
        sendMessage(entry.senderID, "actionResponse", messageData);
    }

}

void SimpleServerApp::sendMessage(string destinationID, string messageName, string messageData) {
    HeterogeneousMessage *message = new HeterogeneousMessage();
    message->setName(messageName.c_str());
    IPv4Address address = manager->getIPAddressForID(destinationID);
	if (address.isUnspecified()) return;
    message->setSourceAddress("server");
    message->setWsmData(messageData.c_str());
    int dataByteLength = (int)messageData.size();
    message->setByteLength(headerLengthBytes + dataByteLength);
    INFO_ID("Sending Message from server to " << destinationID << " = " << address);
    emit(controllerGeocastControlOverheadOut, 1);
    socket.sendTo(message, address, 4242);
}

void SimpleServerApp::forwardMessageToVehicle(HeterogeneousMessage *msg, string destinationAddress) {
    IPv4Address address = manager->getIPAddressForID(destinationAddress);
    msg->setSourceAddress("server");
	if (address.isUnspecified()) return;
	emit(controllerOverheadOut, 1);
    socket.sendTo(msg, address, 4242);
}
