from tof.sensor import Sensor

if __name__ == "__main__":

    sensorIsGood = True

    sensor = Sensor.open("/dev/ttyUSB0")
    
    try :
        (versionMajor, versionMinor) = sensor.getVersion()
        print("Sensor version: %s.%s" % (versionMajor, versionMinor))

        # continuous measurement
        print("Continuous measurement ...")
        sensor.setMode(1)
        
        (distance, temporature, amplitude) = sensor.start()

        # to continuously read the measurement
        while (1) :
            (distance, temporature, amplitude) = sensor.measure()
            print("Distance: %s  Temporature: %s Amplitude: %s" % (distance, temporature, amplitude))
    except :
        print("Somthing wrong with the continuous measurement.")
    
    sensor.close()
