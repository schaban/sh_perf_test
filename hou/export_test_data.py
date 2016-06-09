import sys
import hou
import os
import imp
import re
import inspect
from math import *
from array import array

def getExePath():
	return os.path.dirname(os.path.abspath(inspect.getframeinfo(inspect.currentframe()).filename))

def libLoad(libName, libPath = None):
	if not libPath: libPath = getExePath()
	libFile, libFname, libDescr = imp.find_module(libName, [libPath])
	return imp.load_module(libName, libFile, libFname, libDescr)

def libImpAll(libName, libPath = None):
	libLoad(libName, libPath)
	return "from " + libName + " import *"

def libImp(libName, libPath = None):
	libLoad(libName, libPath)
	return "import " + libName


if __name__=="__main__":
	exePath = getExePath()
	libPath = exePath + "/../tool/"
	outBase = exePath + "/../data/"
	if not os.path.exists(outBase): os.makedirs(outBase)
	exec(libImpAll("exp_xgeo", libPath))

	envPath = "/obj/ENV/OUT"
	envSOP = hou.node(envPath)
	if envSOP:
		print "Exporting environment model from", envSOP.path()
		geo = GeoExporter()
		geo.build(envSOP, bvhFlg = True)
		outPath = outBase + "env.xgeo"
		geo.save(outPath)

	tgtPath = "/obj/TGT/OUT"
	tgtSOP = hou.node(tgtPath)
	if tgtSOP:
		print "Exporting target model from", tgtSOP.path()
		geo = GeoExporter()
		geo.build(tgtSOP)
		outPath = outBase + "tgt.xgeo"
		geo.save(outPath)


