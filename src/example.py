from marvelmind import MarvelmindHedge
from time import sleep
import sys

def main():
    hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) # create MarvelmindHedge thread
    
    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start() # start thread
    while True:
        try:
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()

            if (hedge.positionUpdated):
                hedge.print_position()
                
            if (hedge.distancesUpdated):
                hedge.print_distances()
                
            if (hedge.rawImuUpdated):
                hedge.print_raw_imu()
                
            if (hedge.fusionImuUpdated):
                hedge.print_imu_fusion()
                
            if (hedge.telemetryUpdated):
                hedge.print_telemetry()
                
            if (hedge.qualityUpdated):
                hedge.print_quality()
                
            if (hedge.waypointsUpdated):
                hedge.print_waypoint()
        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()
main()
