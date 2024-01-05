from tof.sensor import Sensor

if __name__ == "__main__":

    sensorIsGood = True

    sensor = Sensor.open("/dev/ttyUSB0")
    
    try :
        (versionMajor, versionMinor) = sensor.getVersion()
        print("Sensor version: %s.%s" % (versionMajor, versionMinor))

        # single measurement
        print("Single measurement ...")
        sensor.setMode(0)

        (distance, temporature, amplitude) = sensor.start()
        print("Distance: %s  Temporature: %s Amplitude: %s" % (distance, temporature, amplitude))
    except :
        print("Somthing wrong with the single measurement.")
    
    sensor.close()
