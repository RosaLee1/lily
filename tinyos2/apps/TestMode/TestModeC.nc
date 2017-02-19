// TestModeC.nc, 2016-06-07 

/**
 * Implementation for TestMode application.  Use spi to write values 
 * to registers and issue strobes.
 *
 * @author: Ruiling Gao 
 **/

module TestModeC @safe()
{
  uses interface CC2420Register as mdmctrl1;
  uses interface CC2420Register as manor;
  uses interface CC2420Register as dactst;
  uses interface CC2420Register as toptst;
  uses interface CC2420Register as MAIN;
  uses interface CC2420Strobe as stxccaon;
  uses interface CC2420Strobe as SXOSCON;
  uses interface Boot;
}
implementation
{
  event void Boot.booted()
  {
    call MAIN.write(0x0000);
    call SXOSCON.strobe();
    call manor.write(0x0100);
    call toptst.write(0x0004);
    call mdmctrl1.write(0x0508);
    call dactst.write(0x1800);
    call stxccaon.strobe();

   // dbg("TestModeC", "Timer 0 fired @ %s.\n", sim_time_string());
  }
}

