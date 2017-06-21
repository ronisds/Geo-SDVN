package org.car2x.veins.modules.lte;

import org.car2x.veins.base.modules.IBaseApplLayer;

simple ExampleDecisionMaker like IBaseApplLayer {

	parameters:
		@class(ExampleDecisionMaker);
		@signal[DSRCMessagesSent](type="long");
		@statistic[DSRCMessagesSent](title="DSRCMessagesSent"; source="DSRCMessagesSent"; record=count);
		@signal[lteMessagesSent](type="long");
		@statistic[lteMessagesSent](title="lteMessagesSent"; source="lteMessagesSent"; record=count);
//		@signal[dontCareMessagesSent](type="long");
//		@statistic[dontCareMessagesSent](title="dontCareMessagesSent"; source="dontCareMessagesSent"; record=vector, stats, histogram);
		@signal[DSRCMessagesReceived](type="long");
		@statistic[DSRCMessagesReceived](title="DSRCMessagesReceived"; source="DSRCMessagesReceived"; record=count);
		@signal[lteMessagesReceived](type="long");
		@statistic[lteMessagesReceived](title="lteMessagesReceived"; source="lteMessagesReceived"; record=count);
		int headerLength = default(88bit) @unit(bit);
		double maxOffset = default(0s) @unit(s);
		bool infoLogging = default(true);

    gates:
        input fromApplication;
        output toApplication;

        input fromLte;
        output toLte;

        input lowerLayerIn;
        output lowerLayerOut;
        input lowerControlIn;
        output lowerControlOut;
}
