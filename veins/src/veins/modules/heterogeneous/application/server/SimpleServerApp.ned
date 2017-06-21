package org.car2x.veins.modules.heterogeneous.application.server;

import inet.applications.IUDPApp;

simple SimpleServerApp like IUDPApp {
	parameters:
	    
	    @signal[ControllerOverheadIn](type="long");
		@statistic[ControllerOverheadIn](title="ControllerOverheadIn"; source="ControllerOverheadIn"; record=stats);
		@signal[ControllerOverheadOut](type="long");
		@statistic[ControllerOverheadOut](title="ControllerOverheadOut"; source="ControllerOverheadOut"; record=stats);
		@signal[ControllerGeocastControlOverheadIn](type="long");
		@statistic[ControllerGeocastControlOverheadIn](title="ControllerGeocastControlOverheadIn"; source="ControllerGeocastControlOverheadIn"; record=stats);
	    @signal[ControllerGeocastControlOverheadOut](type="long");
		@statistic[ControllerGeocastControlOverheadOut](title="ControllerGeocastControlOverheadOut"; source="ControllerGeocastControlOverheadOut"; record=stats);
		
		int localPort = default(4242);
		bool debug = default(false);
		bool infoLogging = default(false);
		
		int headerLengthBytes = default(17byte) @unit(byte);
		
		double vehicleEntryDuration = default(5s) @unit(s);
		int guardRange = default(20m) @unit(m);
	gates:
		input udpIn @labels(UDPControlInfo/up);
		output udpOut @labels(UDPControlInfo/down);
}
