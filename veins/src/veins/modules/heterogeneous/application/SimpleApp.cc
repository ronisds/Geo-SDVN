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

#include "SimpleApp.h"

Define_Module(SimpleApp);

using Veins::TraCIMobilityAccess;

void SimpleApp::initialize(int stage) {
	if (stage == 0) {

		debug = par("debug").boolValue();
		infoLogging = par("infoLogging").boolValue();
		toDecisionMaker = findGate("toDecisionMaker");
		fromDecisionMaker = findGate("fromDecisionMaker");

		mobility = TraCIMobilityAccess().get(getParentModule());
		ASSERT(mobility);
		sumoId = mobility->getExternalId();

		headerLengthBytes = par("headerLengthBytes").longValue();
		beaconLengthBytes = par("beaconLengthBytes").longValue();

		// Initialize parameters
		beaconToSDVNControllerInterval = par("beaconToSDVNControllerInterval").doubleValue();
		maximumWaitingTime = par("maximumWaitingTime").doubleValue();
        maximumTries = par("maximumTries").longValue();
        tablemissMinimumWaitingTime = par("tablemissMinimumWaitingTime").doubleValue();
        tablemissAdditionalWaitingTime = par("tablemissAdditionalWaitingTime").doubleValue();
        tablemissMaximumTries = par("tablemissMaximumTries").longValue();

        // Self-messages
		sendBeaconToSDVNControllerMsg = new cMessage("sendBeaconToSDVNController");
		sendGeocastMsg = new cMessage("SendGeocast");
		tablemissMinimumTimeOutMsg = new cMessage("tablemissMinimumTimeOutMsg");
		tablemissAdditionalTimeOutMsg = new cMessage("tablemissAdditionalTimeOutMsg");
		broadcastTimeOutMsg = new cMessage("broadcastTimeOutMsg");

		scheduleAt(simTime() + 0.001, sendBeaconToSDVNControllerMsg);
		if (sumoId == "Car_0") {
		    regionCenterToSent = mobility->getCurrentPosition();
		    regionRadiusToSent = 1500;
		    INFO_ID("Veiculo " << sumoId << " enviará geocast para regiao (" << regionCenterToSent.x << "," << regionCenterToSent.y << ") em 3 segundos");
		    scheduleAt(simTime() + 3.0, sendGeocastMsg);
		}
	}
}

void SimpleApp::handleMessage(cMessage *msg) {
	if (msg->isSelfMessage()) {
	    if (msg == sendBeaconToSDVNControllerMsg) { // Send beacons
	        sendBeaconToSDVNController();
	        scheduleAt(simTime() + beaconToSDVNControllerInterval, sendBeaconToSDVNControllerMsg);
	        return;
	    }
	    if (msg == sendGeocastMsg) { // Send geocast (testing purpose)
	        if (mobility->getCurrentPosition().distance(regionCenterToSent) < regionRadiusToSent) {
	            TraCIScenarioManagerAccess().get()->vehicleCreateMessage(sumoId, SimpleApp::messageCount, regionCenterToSent, regionRadiusToSent, simTime());
	            tablemissCurrentTry = 1;
	            geocastMessage("Mensagem teste enviada por geocast", regionCenterToSent, regionRadiusToSent);
	            INFO_ID("ENVIANDO: Veiculo " << sumoId << " enviará geocast para regiao (" << regionCenterToSent.x << "," << regionCenterToSent.y << ") em 5 segundos");
	        }
	        scheduleAt(simTime() + 5.0, sendGeocastMsg);
	    }
	    if (msg == tablemissMinimumTimeOutMsg) { // Table-miss minimum timeout
	        INFO_ID("Table-miss minimum timeout com mensagem " << waitingControllerResponseForMessage);
	        if (waitingControllerResponseForMessage != "") { // Msg not sent
	            if (!flowTable.hasMatch(sumoId, geocastIdenfier)) { // Response not received
	                INFO_ID("Acabou o tempo e nao recebeu a resposta");
	                scheduleAt(simTime() + tablemissAdditionalWaitingTime, tablemissAdditionalTimeOutMsg); // Schedule the additional time-miss timeout
	            } else { // Response received
	                INFO_ID("Acabou o tempo e recebeu a resposta");
	                if (geocastMessage(waitingControllerResponseForMessage, regionCenterToSent, regionRadiusToSent)) { // Send the message
	                    waitingControllerResponseForMessage = "";
	                }
	            }
	        }
	    }
	    if (msg == tablemissAdditionalTimeOutMsg) { // Table-miss additional timeout
	        INFO_ID("Table-miss additional timeout com mensagem " << waitingControllerResponseForMessage);
	        if (waitingControllerResponseForMessage != "") { // Msg not sent
	            if (!flowTable.hasMatch(sumoId, geocastIdenfier)) { // Response not received
	                if (tablemissCurrentTry < tablemissMaximumTries) { // Try again
	                    ++tablemissCurrentTry;
	                    if (geocastMessage(waitingControllerResponseForMessage, regionCenterToSent, regionRadiusToSent)) { // This will fire the table-miss
	                        waitingControllerResponseForMessage = "";
	                    }
	                } else { // Maximum tries reached
	                    tablemissCurrentTry = 0;
	                    INFO_ID("Não conseguiu enviar a action request pro servidor");
	                }
	            } else { // Response received
                    INFO_ID("Acabou o tempo adicional e já recebeu a resposta");
	            }
	        }
	    }
	    if (msg == broadcastTimeOutMsg) {
	        sendBroadcastMessage(messageToSend->dup());
	    }
	} else {
		HeterogeneousMessage *heterogeneousMessage = dynamic_cast<HeterogeneousMessage *>(msg);

		if(heterogeneousMessage){
		    if (heterogeneousMessage->getName() == string("data")) {

		        int idx = -1;
		        for(int i = 0; i < waitingCurrentNextHops.size(); ++i) {
		            if (waitingCurrentNextHops[i] == heterogeneousMessage->getSourceAddress()) {
		                idx = i;
		                break;
		            }
		        }
		        if (idx != -1) waitingCurrentNextHops.erase(waitingCurrentNextHops.begin() + idx);


		        if (heterogeneousMessage->getRegionCenter().distance(mobility->getCurrentPosition()) <= heterogeneousMessage->getRegionRadius()) {

		            if (treatedMessages.count(heterogeneousMessage->getMessageID()) == 0) {
		                bool willForward = false;
		                if (flowTable.getAction(heterogeneousMessage->getGeocastSourceAddress(), heterogeneousMessage->getGeocastIdenfier()).first == SendToNextHop || flowTable.getAction(heterogeneousMessage->getGeocastSourceAddress(), heterogeneousMessage->getGeocastIdenfier()).first == SendToNextHopAndSDVNController) {
		                    willForward = true;
		                }
		                INFO_ID("Veiculo " << sumoId << " recebeu e transmite = " << willForward);
		                TraCIScenarioManagerAccess().get()->vehicleReceivedMessage(sumoId, heterogeneousMessage->getMessageID(), simTime(), willForward);
		            }

		            INFO_ID("Veiculo " << sumoId << " recebeu dado de " << heterogeneousMessage->getGeocastSourceAddress() << " por meio de " << heterogeneousMessage->getSourceAddress() << " acao = " << flowTable.getAction(heterogeneousMessage->getGeocastSourceAddress(), heterogeneousMessage->getGeocastIdenfier()).first);
		            treatMessage(heterogeneousMessage);
		        }
		    } else if (heterogeneousMessage->getName() == string("actionResponse")) {
		        char messageData[100];
		        strcpy(messageData, heterogeneousMessage->getWsmData());
		        char *token;
		        token = strtok(messageData, "|");
		        vector<string> elements;
		        while(token != NULL) {
		            elements.push_back(string(token));
		            token = strtok(NULL, "|");
		        }


		        ASSERT2(elements.size() >= 8, "Action response with wrong number of items (" + to_string((int)elements.size()) + ")");

		        string senderID = elements[0];
		        int matchIdentifier = atoi(elements[1].c_str());
		        double x = atof(elements[2].c_str());
		        double y = atof(elements[3].c_str());
		        double z = atof(elements[4].c_str());
		        int regionRadius = atoi(elements[5].c_str());
		        int code = atoi(elements[6].c_str());
		        INFO_ID(sumoId << ": Action response received with data: " << heterogeneousMessage->getWsmData() << " and code " << code);
		        FlowAction action;
		        if (code == 0) action = Drop;
		        else if (code == 1) action = SendToNextHop;
		        else if (code == 2) action = SendToSDVNController;
		        else action = SendToNextHopAndSDVNController;

		        int numberOfNextHops = atoi(elements[7].c_str());
		        ASSERT2(elements.size() == 8 + numberOfNextHops, "Action response with wrong number of items because of next hops (" + to_string((int)elements.size()) + ")");

		        vector<string> nextHops;
		        for(int i = 0; i < numberOfNextHops; ++i) {
		            nextHops.push_back(elements[i + 8]);
		        }

		        Coord regionCenter = Coord(x, y, z);

		        flowTable.setEntry(senderID, matchIdentifier, regionCenter, regionRadius, action, nextHops);

		        if (tablemissAdditionalTimeOutMsg->isScheduled()) { // Send the message if in additional time
		            INFO_ID("Recebeu a resposta no tempo adicional. Vai enviar os dados");
		            cancelEvent(tablemissAdditionalTimeOutMsg);
		            if (geocastMessage(waitingControllerResponseForMessage, regionCenterToSent, regionRadiusToSent)) { // Send the message
		                waitingControllerResponseForMessage = "";
		            }
		        }
		    }
		}
	}
}

void SimpleApp::treatMessage(HeterogeneousMessage *msg) {
    if (treatedMessages.count(msg->getMessageID()) != 0)
        return;

    treatedMessages.insert(msg->getMessageID());

    pair<FlowAction, vector<string> > action = flowTable.getAction(msg->getGeocastSourceAddress(), msg->getGeocastIdenfier());
    INFO_ID(sumoId << ": Treating Message: " << msg->getMessageID() << " with action " << action.first);
    switch (action.first) {
    case Drop:
        break;
    case SendToNextHop:
        sendBroadcastMessage(msg->dup(), action.second);
        break;
    case SendToSDVNController:
        sendMessageToServer(msg->dup());
        break;
    case SendToNextHopAndSDVNController:
        sendBroadcastMessage(msg->dup(), action.second);
        sendMessageToServer(msg->dup());
        break;
    default:
        break;
    }
}

void SimpleApp::sendMessageToServer(HeterogeneousMessage *msg) {
    msg->setNetworkType(LTE);
    msg->setSourceAddress(sumoId.c_str());
    msg->setDestinationAddress("server");
    send(msg, toDecisionMaker);
}

void SimpleApp::sendBroadcastMessage(HeterogeneousMessage *msg, vector<string> nextHops) {
    if (geocastCurrentTry == 0) {
        string ss;
        for(int i = 0; i < nextHops.size(); ++i) ss += nextHops[i] + " ";
        INFO_ID("Vai fazer a primeira tentativa de envio com next hops " << nextHops.size() << ": " << ss);
        geocastCurrentTry = 1;
        waitingCurrentNextHops = nextHops;
        messageToSend = msg->dup();
        scheduleAt(simTime() + maximumWaitingTime, broadcastTimeOutMsg);
    } else if (waitingCurrentNextHops.size() > 0) {
        string ss;
        for(int i = 0; i < waitingCurrentNextHops.size(); ++i) ss += waitingCurrentNextHops[i] + " ";
        INFO_ID("Nao conseguiu enviar para todos os vizinhos ainda: " << ss);
        if (geocastCurrentTry < maximumTries) {
            INFO_ID("Tentando enviar novamente");
            ++geocastCurrentTry;
            scheduleAt(simTime() + maximumWaitingTime, broadcastTimeOutMsg);
        } else {
            messageToSend = NULL;
            geocastCurrentTry = 0;
            waitingCurrentNextHops.clear();
            INFO_ID("Nao conseguiu enviar para todos os vizinhos!!!");
            return;
        }
    } else {
        messageToSend = NULL;
        geocastCurrentTry = 0;
        INFO_ID("Ja conseguiu enviar a mensagem!!!");
        return;
    }
    msg->setNetworkType(DSRC);
    msg->setSourceAddress(sumoId.c_str());
    INFO_ID(sumoId << ": Broadcasting message: " << msg->getMessageID());
    send(msg, toDecisionMaker);
}

void SimpleApp::sendBeaconToSDVNController() {
    HeterogeneousMessage* beaconMessage = new HeterogeneousMessage();
    beaconMessage->setName("BeaconToSDVNControllerMsg");
    beaconMessage->setByteLength(headerLengthBytes + beaconLengthBytes);
    sendMessageToServer(beaconMessage);
}

// Msg must not be empty
bool SimpleApp::geocastMessage(string msg, Coord regionCenter, int regionRadius) {
    if (msg == "") return false;
    if (!flowTable.hasMatch(sumoId, geocastIdenfier)) {
        waitingControllerResponseForMessage = msg;
        INFO_ID("Agendando mensagem " << waitingControllerResponseForMessage);
        cancelEvent(tablemissMinimumTimeOutMsg);
        cancelEvent(tablemissAdditionalTimeOutMsg);
        scheduleAt(simTime() + tablemissMinimumWaitingTime, tablemissMinimumTimeOutMsg);
        requestAction(regionCenter, regionRadius);
        return false;
    }
    int dataLengthBytes = msg.size();
    HeterogeneousMessage* dataMessage = new HeterogeneousMessage();

    dataMessage->setMessageID(SimpleApp::messageCount++);
    dataMessage->setGeocastSourceAddress(sumoId.c_str());
    dataMessage->setGeocastIdenfier(geocastIdenfier);
    dataMessage->setName("data");
    dataMessage->setByteLength(headerLengthBytes + dataLengthBytes);
    dataMessage->setWsmData(msg.c_str());
    dataMessage->setRegionCenter(regionCenter);
    dataMessage->setRegionRadius(regionRadius);
    treatMessage(dataMessage);
    return true;
}

void SimpleApp::requestAction(Coord regionCenter, int regionRadius) {
    HeterogeneousMessage *requestMessage = new HeterogeneousMessage();
    requestMessage->setName("actionRequest");
    string requestData = sumoId + "|" + to_string(geocastIdenfier) + "|" + to_string(regionCenter.x) + "|" + to_string(regionCenter.y) + "|" + to_string(regionCenter.z) + "|" + to_string(regionRadius);
    INFO_ID(sumoId << ": Sending action request with data: " << requestData << " and packet size = " << headerLengthBytes + requestData.size());
    requestMessage->setWsmData(requestData.c_str());
    requestMessage->setByteLength(headerLengthBytes + requestData.size());
    sendMessageToServer(requestMessage);
}
