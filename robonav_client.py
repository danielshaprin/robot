#!/usr/bin/python3

from struct import *
from collections import namedtuple
from enum import IntEnum
import socket, sys

SERVER_HOST = "77.70.61.150"
SERVER_PORT = 41999

class QueryResponseId(IntEnum):
    NEW_SESSION  = 42000001
    STATUS       = 42000002
    SET_SPEEDS   = 42000003
    SET_STEERING = 42000004
    WAIT         = 42000005
    FINISH       = 42000006
    #
    ERROR_CODE   = 45000001

def getASCIIZ(xbytes):
    i = 0
    l = len(xbytes)
    while i < l and xbytes[i]:
        i += 1
    return xbytes[:i].decode("utf-8")

def query(code, sessionKey, arguments, mode=""):
    tosend = pack("<II", code, sessionKey)
    if arguments:
        tosend += pack("<" + (mode * len(arguments)), *arguments)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((SERVER_HOST, SERVER_PORT))
    s.sendall(tosend)
    reply = s.recv(800)
    s.close()
    if int.from_bytes(reply[:4], "little") == QueryResponseId.ERROR_CODE:
        errorCode = reply[4]
        errorMsg = getASCIIZ(reply[5:])
        print("%s call failed with error code %d, msg is '%s'" % (code, errorCode, errorMsg))
        sys.exit(99)
    return reply

RoboNavStatus = namedtuple("RoboNavStatus","gpsLat gpsLon hdg spdFL spdFR spdBL spdBR steerFL steerFR distanceToTarget")

class RoboNavClient(object):
    def __init__(self, seed):
        reply = query(QueryResponseId.NEW_SESSION, 0, [seed, 0], "I")
        rspId, key, lat, lon = unpack("<IIdd", reply)
        self._sk = key
        self.targetLat = lat
        self.targetLon = lon

    def getKey(self):
        return self._sk

    def status(self):
        reply = query(QueryResponseId.STATUS, self._sk, [])
        return RoboNavStatus._make(unpack("<ddffffffff", reply[4:]))

    def setSpeeds(self, FL, FR, BL, BR):
        query(QueryResponseId.SET_SPEEDS, self._sk, [FL, FR, BL, BR], "f")

    def setSteering(self, steerFL, steerFR):
        query(QueryResponseId.SET_STEERING, self._sk, [steerFL, steerFR], "f")

    def wait(self, time):
        query(QueryResponseId.WAIT, self._sk, [time], "d")

    def finish(self):
        reply = query(QueryResponseId.FINISH, self._sk, [])
        print("Finish called. Response from server is: '%s'" % getASCIIZ(reply[5:]))
