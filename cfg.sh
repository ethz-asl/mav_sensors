echo "configDataPort 921600 1" > /dev/ttyUSB0
sleep 1
echo "sensorStop" > /dev/ttyUSB0
sleep 1
echo "flushCfg" > /dev/ttyUSB0
sleep 1
echo "dfeDataOutputMode 1" > /dev/ttyUSB0
sleep 1
echo "channelCfg 15 7 0" > /dev/ttyUSB0
sleep 1
echo "adcCfg 2 1" > /dev/ttyUSB0
sleep 1
echo "adcbufCfg -1 0 1 1 1" > /dev/ttyUSB0
sleep 1
echo "profileCfg 0 77 267 7 57.14 0 0 70 1 256 5209 0 0 30" > /dev/ttyUSB0
sleep 1
echo "chirpCfg 0 0 0 0 0 0 0 1" > /dev/ttyUSB0
sleep 1
echo "chirpCfg 1 1 0 0 0 0 0 2" > /dev/ttyUSB0
sleep 1
echo "chirpCfg 2 2 0 0 0 0 0 4" > /dev/ttyUSB0
sleep 1
echo "frameCfg 0 2 16 0 100 1 0" > /dev/ttyUSB0
sleep 1
echo "lowPower 0 0" > /dev/ttyUSB0
sleep 1
echo "guiMonitor -1 1 1 0 0 0 1" > /dev/ttyUSB0
sleep 1
echo "cfarCfg -1 0 2 8 4 3 0 15 1" > /dev/ttyUSB0
sleep 1
echo "cfarCfg -1 1 0 4 2 3 1 15 1" > /dev/ttyUSB0
sleep 1
echo "multiObjBeamForming -1 1 0.5" > /dev/ttyUSB0
sleep 1
echo "clutterRemoval -1 0" > /dev/ttyUSB0
sleep 1
echo "calibDcRangeSig -1 0 -5 8 256" > /dev/ttyUSB0
sleep 1
echo "extendedMaxVelocity -1 0" > /dev/ttyUSB0
sleep 1
echo "lvdsStreamCfg -1 0 0 0" > /dev/ttyUSB0
sleep 1
echo "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0" > /dev/ttyUSB0
sleep 1
echo "measureRangeBiasAndRxChanPhase 0 1.5 0.2" > /dev/ttyUSB0
sleep 1
echo "CQRxSatMonitor 0 3 5 121 0" > /dev/ttyUSB0
sleep 1
echo "CQSigImgMonitor 0 127 4" > /dev/ttyUSB0
sleep 1
echo "analogMonitor 0 0" > /dev/ttyUSB0
sleep 1
echo "aoaFovCfg -1 -90 90 -90 90" > /dev/ttyUSB0
sleep 1
echo "cfarFovCfg -1 0 0 8.92" > /dev/ttyUSB0
sleep 1
echo "cfarFovCfg -1 1 -1 1.00" > /dev/ttyUSB0
sleep 1
echo "calibData 0 0 0" > /dev/ttyUSB0
sleep 1
echo "sensorStart" > /dev/ttyUSB0
