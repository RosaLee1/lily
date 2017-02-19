// TestModeAppC.nc, 2016-06-07 

/**
 * TestMode is a basic application that sets a mote into test mode.
 * It does so by reseting the radio chip, writing appropriate values to 
 * registers, and issuing correct strobes. It uses the
 * spi component to achieve this goal.
 *
 * @author Ruiling Gao gaoxiaopang7323@gmail.com
 **/

configuration TestModeAppC
{
}
implementation
{
  components MainC, TestModeC as App;
  components new CC2420SpiC() as spi1;
  components new CC2420SpiC() as spi2;
  components new CC2420SpiC() as spi3;
  components new CC2420SpiC() as spi4;
  components new CC2420SpiC() as spi5;
  components new CC2420SpiC() as spi6;
  components new CC2420SpiC() as spi7;

  App -> MainC.Boot;

  App.mdmctrl1 -> spi1.MDMCTRL1;
  App.manor -> spi2.MANOR;
  App.stxccaon -> spi3.STXON;
  App.dactst -> spi4.DACTST;
  App.toptst -> spi5.TOPTST;
  App.MAIN -> spi6.MAIN;
  App.SXOSCON -> spi7.SXOSCON;
}

