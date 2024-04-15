import time
import os
import sys
from datetime import datetime
import time

import logging
import logging.handlers
import subprocess

class CsvLogger(object):

    def __init__(self, path = '.', loggername = 'foo', header='',format = '%s'):
        self.taget_path = path
        self.logger_name = loggername
        self.header = header
        self.format = ', '+format
        self.log_ext = '.csv'
        self.time_suffix = str('{:%Y_%m_%d_%H_%M}'.format(datetime.now()))
        self.path_logger_suffix = self.taget_path + '/'
        try:
            if not os.path.exists(self.path_logger_suffix):
                os.makedirs(self.path_logger_suffix)
            else:
                print('[CsvLogger] Path exists, no action is required')
        except OSError:
                print('[CsvLogger] Creation of the directory %s failed'%(self.path_logger_suffix))
        else:
            print('[CsvLogger] Successfully created the directory %s' % (self.path_logger_suffix))

        self.path_logname = self.path_logger_suffix + self.logger_name + '_' + self.time_suffix + self.log_ext
        # Set up a specific logger with our desired output level
        self.csvlogger = logging.getLogger(loggername)
        self.csvlogger.setLevel(logging.DEBUG)
        # Add the log message handler to the logger
        #handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=2000, backupCount=4000)
        handler = logging.FileHandler(self.path_logname)
        self.csvlogger.addHandler(handler)

        self.csvlogger.debug('date,ms,' + self.header)

        formatter = logging.Formatter('%(asctime)s %(message)s')
        handler.setFormatter(formatter)
        self.csvlogger.addHandler(handler)

    def logData(self, data):
        self.csvlogger.debug(self.format%data)

if __name__ == '__main__':
    csvHdlr1 = CsvLogger(path = './log/csvtest1', loggername = 'csvlogger1', header='bid,x,y,z,pitch,roll,yaw,volt,temp,lat,long',format ='%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f')
    csvHdlr1.logData((15,-1,2,0,15.1,-123.4,315.322,15.1,10,-86.2315,40.32315))
    csvHdlr2 = CsvLogger(path = './log/csvtest2', loggername = 'csvlogger2', header='a,b,c',format ='%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f')
    csvHdlr2.logData((15,1,2,0,15.1,-123.4,315.322,15.1,10,-86.2315,40.32315))
    csvHdlr1.logData((15,-1,2,0,15.1,-123.4,315.322,15.1,10,-86.2315,40.32315))
