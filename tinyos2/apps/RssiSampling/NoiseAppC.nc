/******************************************************************
    NoiseAppC.nc created by HyungJune Lee (abbado@stanford.edu)
 ******************************************************************/

#include "StorageVolumes.h"
#include "NoiseSample.h"

configuration NoiseAppC
{
}

implementation
{
	components MainC;
	components LedsC;
	components new Alarm32khz32C() as Alarm0;
	components new CC2420RssiC() as RssiC;
	components NoiseSampleP as App;
	components ActiveMessageC;
	components SerialActiveMessageC;
	components new SerialAMSenderC(AM_SERIAL_ID);
	components new BlockStorageC(VOLUME_BLOCK);
	components HplCC2420PinsC as Pins;

	App->MainC.Boot;
	App.Leds->LedsC;
	App.Alarm0->Alarm0; 
	App.Resource->RssiC;
	App.RSSI->RssiC;
	App.CSN->Pins.CSN;
	App.RadioControl->ActiveMessageC;
	App.SerialControl->SerialActiveMessageC;
	App.AMSend->SerialAMSenderC;
	App.Packet->SerialAMSenderC;
	App.BlockRead->BlockStorageC;
	App.BlockWrite->BlockStorageC;
}


