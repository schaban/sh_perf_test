# Author: Sergey Chaban <sergey.chaban@gmail.com>

import sys
import hou
from math import *

def pathToName(path):
	return path[path.rfind("/")+1:]

def texPathToName(path):
	name = pathToName(path)
	if not "op:" in path:
		name = name[:name.rfind(".")]
	return name

def findSOPEXP(rootPath = "/obj", expNodeName = "EXP"):
	sopEXP = None
	objList = hou.node(rootPath).children()
	for obj in objList:
		if obj.type().name() == "geo":
			tstSOP = hou.node(obj.path() + "/" + expNodeName)
			if tstSOP:
				sopEXP = tstSOP
				break
	return sopEXP

def findDeform(rootPath = "/obj"):
	objLst = hou.node(rootPath).children()
	for obj in objLst:
		if obj.type().name() == "geo":
			sopLst = obj.children()
			for sop in sopLst:
				if sop.type().name() == "deform":
					return sop
	return None

def findSkinNode(rootPath = "/obj"):
	skin = None
	deform = findDeform(rootPath)
	if deform:
		inp = deform.inputs()
		if len(inp) > 0:
			skin = inp[0]
	return skin

def getSkinAttr(geo):
	if not geo: return None
	skinAttr = geo.findPointAttrib("pCapt")
	if not skinAttr:
		skinAttr = geo.findPointAttrib("boneCapture")
	return skinAttr

def getSkinNames(geo):
	skinAttr = getSkinAttr(geo)
	if not skinAttr: return None
	newSkin = True
	try: getattr(skinAttr, "indexPairPropertyTables")
	except: newSkin = False
	lst = []
	if newSkin:
		tbl = skinAttr.indexPairPropertyTables()[0]
		n = tbl.numIndices()
		for i in xrange(n):
			jname = tbl.stringPropertyValueAtIndex("pCaptPath", i)
			jname = jname.split("/cregion")[0]
			lst.append(jname)
	else:
		sopPath = geo.sopNode().path()
		n = int(hou.hscript('echo `detailsnummap("' + sopPath + '", "pCaptPath")`')[0].split()[0])
		for i in xrange(n):
			lst.append(hou.hscript('echo `detailsmap("' + sopPath + '", "pCaptPath", ' + str(i) + ')`')[0].split("/cregion")[0])
	return lst

def getPntUV(pnt, attr, flipV = True):
	uv = pnt.attribValue(attr)
	u = uv[0]
	v = uv[1]
	if flipV: v = 1.0 - v
	return [u, v]


def qget(r):
	return (hou.Quaternion(r[0], (1, 0, 0)), hou.Quaternion(r[1], (0, 1, 0)), hou.Quaternion(r[2], (0, 0, 1)))

def qord(rord):
	id = 0
	for i in xrange(3): id += (ord(rord[i]) - ord('x')) << (i*2)
	return (id / 3) - 2

def qrot(rot, rord):
	tbl = [
		lambda qx, qy, qz: qx * qy * qz,
		lambda qx, qy, qz: qx * qz * qy,
		None,
		None,
		lambda qx, qy, qz: qy * qx * qz,
		None,
		lambda qx, qy, qz: qy * qz * qx,
		None,
		None,
		lambda qx, qy, qz: qz * qx * qy,
		lambda qx, qy, qz: qz * qy * qx
	]
	(qx, qy, qz) = qget(rot)
	q = tbl[qord(rord)](qx, qy, qz)
	return q.normalized()

def qapply(q, v): return v * hou.Matrix4(q.extractRotationMatrix3())

def qinv(q):
	(x, y, z, w) = q
	return hou.Quaternion(-x, -y, -z, w)

def qgetAxisX(q):
	(x, y, z, w) = q
	return hou.Vector3(1.0 - 2.0*y*y - 2.0*z*z, 2.0*x*y + 2.0*w*z, 2.0*x*z - 2.0*w*y)

def qgetAxisY(q):
	(x, y, z, w) = q
	return hou.Vector3(2.0*x*y - 2.0*w*z, 1.0 - 2.0*x*x - 2.0*z*z, 2.0*y*z + 2.0*w*x)

def qgetAxisZ(q):
	(x, y, z, w) = q
	return hou.Vector3(2.0*x*z + 2.0*w*y, 2.0*y*z - 2.0*w*x, 1.0 - 2.0*x*x - 2.0*y*y)

def asinSafe(x):
	x = min(max(x, -1.0), 1.0)
	return asin(x)

def qexyz(q):
	iq = qinv(q)
	(xx, yy, zz) = qgetAxisZ(iq)
	s = 1.0 / max(1.0e-8, hypot(yy, zz))
	yy *= s
	zz *= s
	rx = hou.hmath.radToDeg(atan2(yy, zz))
	ry = hou.hmath.radToDeg(-asinSafe(xx))
	qz = hou.Quaternion(ry, (0, 1, 0)) * hou.Quaternion(rx, (1, 0, 0))
	v = qapply(qz, qgetAxisY(iq))
	rz = hou.hmath.radToDeg(atan2(v[0], v[1]))
	return [rx, ry, rz]

def qexzy(q):
	iq = qinv(q)
	(xx, yy, zz) = qgetAxisY(iq)
	s = 1.0 / max(1.0e-8, hypot(yy, zz))
	yy *= s
	zz *= s
	rx = hou.hmath.radToDeg(-atan2(zz, yy))
	rz = hou.hmath.radToDeg(asinSafe(xx))
	qy = hou.Quaternion(rz, (0, 0, 1)) * hou.Quaternion(rx, (1, 0, 0))
	v = qapply(qy, qgetAxisZ(iq))
	ry = hou.hmath.radToDeg(-atan2(v[0], v[2]))
	return [rx, ry, rz]

def qeyxz(q):
	iq = qinv(q)
	(xx, yy, zz) = qgetAxisZ(iq)
	s = 1.0 / max(1.0e-8, hypot(xx, zz))
	xx *= s
	zz *= s
	ry = hou.hmath.radToDeg(-atan2(xx, zz))
	rx = hou.hmath.radToDeg(asinSafe(yy))
	qz = hou.Quaternion(rx, (1, 0, 0)) * hou.Quaternion(ry, (0, 1, 0))
	v = qapply(qz, qgetAxisX(iq))
	rz = hou.hmath.radToDeg(-atan2(v[1], v[0]))
	return [rx, ry, rz]

def qeyzx(q):
	iq = qinv(q)
	(xx, yy, zz) = qgetAxisX(iq)
	s = 1.0 / max(1.0e-8, hypot(xx, zz))
	xx *= s
	zz *= s
	ry = hou.hmath.radToDeg(atan2(zz, xx))
	rz = hou.hmath.radToDeg(-asinSafe(yy))
	qx = hou.Quaternion(rz, (0, 0, 1)) * hou.Quaternion(ry, (0, 1, 0))
	v = qapply(qx, qgetAxisZ(iq))
	rx = hou.hmath.radToDeg(atan2(v[1], v[2]))
	return [rx, ry, rz]

def qezxy(q):
	iq = qinv(q)
	(xx, yy, zz) = qgetAxisY(iq)
	s = 1.0 / max(1.0e-8, hypot(xx, yy))
	xx *= s
	yy *= s
	rz = hou.hmath.radToDeg(atan2(xx, yy))
	rx = hou.hmath.radToDeg(-asinSafe(zz))
	qy = hou.Quaternion(rx, (1, 0, 0)) * hou.Quaternion(rz, (0, 0, 1))
	v = qapply(qy, qgetAxisX(iq))
	ry = hou.hmath.radToDeg(atan2(v[2], v[0]))
	return [rx, ry, rz]

def qezyx(q):
	iq = qinv(q)
	(xx, yy, zz) = qgetAxisX(iq)
	s = 1.0 / max(1.0e-8, hypot(xx, yy))
	xx *= s
	yy *= s
	rz = hou.hmath.radToDeg(-atan2(yy, xx))
	ry = hou.hmath.radToDeg(asinSafe(zz))
	qx = hou.Quaternion(ry, (0, 1, 0)) * hou.Quaternion(rz, (0, 0, 1))
	v = qapply(qx, qgetAxisY(iq))
	rx = hou.hmath.radToDeg(-atan2(v[2], v[1]))
	return [rx, ry, rz]

def qgetRot(q, rord, std=False):
	if std: return q.extractEulerRotates(rord)
	tbl = [qezyx, qeyzx, None, None, qezxy, None, qexzy, None, None, qeyxz, qexyz]
	return tbl[qord(rord)](q)

# http://www.geometrictools.com/Documentation/ConstrainedQuaternions.pdf
def qgetClosestXY(q):
	x = q[0]
	y = q[1]
	z = q[2]
	w = q[3]
	det = abs(-x*y - z*w)
	if det < 0.5:
		d = sqrt(abs(1.0 - 4.0*det*det))
		a = x*w - y*z
		b = w*w - x*x + y*y - z*z
		if b >= 0.0:
			s0 = a
			c0 = (d + b)*0.5
		else:
			s0 = (d - b)*0.5
			c0 = a
		ilen = rcp0(hypot(s0, c0))
		s0 *= ilen
		c0 *= ilen
		s1 = y*c0 - z*s0
		c1 = w*c0 + x*s0
		ilen = rcp0(hypot(s1, c1))
		s1 *= ilen
		c1 *= ilen
		x = s0*c1
		y = c0*s1
		z = -s0*s1
		w = c0*c1
	else:
		ilen = 1.0 / sqrt(det)
		x *= ilen
		y = 0.0
		z = 0.0
		w *= ilen
	return hou.Quaternion([x, y, z, w])

def qgetClosestYX(q):
	qxy = qgetClosestXY(hou.Quaternion([q[0], q[1], -q[2], q[3]]))
	return hou.Quaternion([qxy[0], qxy[1], -qxy[2], qxy[3]])

def qgetClosestYZ(q):
	qxy = qgetClosestXY(hou.Quaternion([q[1], q[2], q[0], q[3]]))
	return hou.Quaternion([qxy[2], qxy[0], qxy[1], qxy[3]])

def qgetClosestZY(q):
	qxy = qgetClosestXY(hou.Quaternion([q[1], q[2], -q[0], q[3]]))
	return hou.Quaternion([-qxy[2], qxy[0], qxy[1], qxy[3]])


def getMinFrame():
	return int(hou.hscript("frange")[0].split()[2])

def getMaxFrame():
	return int(hou.hscript("frange")[0].split()[4])

def getRotOrd(node):
	rOrd = "xyz"
	if node:
		rOrdParm = node.parm("rOrd")
		if rOrdParm: rOrd = rOrdParm.evalAsString()
	return rOrd

def getXformOrd(node):
	xOrd = "srt"
	if node:
		xOrdParm = node.parm("xOrd")
		if xOrdParm: xOrd = xOrdParm.evalAsString()
	return xOrd

def evalFrmRot(node, fno):
	rOrd = getRotOrd(node)
	parms = hou.parmTuple(node.path() + "/r")
	return qrot(parms.evalAtFrame(fno), rOrd)

def evalFrmPos(node, fno):
	parms = hou.parmTuple(node.path() + "/t")
	return hou.Vector3(parms.evalAtFrame(fno))

def evalFrmScl(node, fno):
	parms = hou.parmTuple(node.path() + "/s")
	return hou.Vector3(parms.evalAtFrame(fno))

def getParamKeys(prm):
	kf = None
	trk = prm.overrideTrack()
	if trk: kf = prm.getReferencedParm().keyframes()
	else: kf = prm.keyframes()
	return kf

def getParamKeysInRange(prm, start, end):
	kf = None
	trk = prm.overrideTrack()
	if trk: kf = prm.getReferencedParm().keyframesInRange(start, end)
	else: kf = prm.keyframesInRange(start, end)
	return kf

def ckParmsAnim(parms):
	for prm in parms:
		keys = getParamKeys(prm)
		if keys: return True
	return False

def hasRotAnim(node):
	parms = hou.parmTuple(node.path() + "/r")
	return ckParmsAnim(parms)

def hasPosAnim(node):
	parms = hou.parmTuple(node.path() + "/t")
	return ckParmsAnim(parms)

def hasSclAnim(node):
	parms = hou.parmTuple(node.path() + "/s")
	return ckParmsAnim(parms)

def setLinKey(prm, val, fno):
	key = hou.Keyframe()
	key.setExpression("linear()")
	key.setFrame(fno)
	key.setValue(val)
	prm.setKeyframe(key)

def mkNode(basePath, nodeType, nodeName):
	node = hou.node(basePath + "/" + nodeName)
	if not node: node = hou.node(basePath).createNode(nodeType, nodeName)
	return node

def mkNull(basePath, nullName):
	return mkNode(basePath, "null", nullName)

def getRGBComponentName(img, cname):
	idx = img.components("C").index(cname)
	return ["r","g","b"][idx]


class BaseTrack:
	def __init__(self, node, minFrame, maxFrame, trkName):
		self.node = node
		self.minFrame = minFrame
		self.maxFrame = maxFrame
		self.trkName = trkName
		self.data = None
		self.getData()
		self.calcBBox()

	def getData(self): pass

	def calcBBox(self):
		if not self.data: return
		self.vmin = self.data[0]
		self.vmax = self.vmin
		for v in self.data[1:]:
			self.vmin = [min(self.vmin[i], v[i]) for i in xrange(len(v))]
			self.vmax = [max(self.vmax[i], v[i]) for i in xrange(len(v))]
		self.bbsize = [self.vmax[i] - self.vmin[i] for i in xrange(len(self.vmax))]

	def isConst(self):
		if not self.data: return True
		if not self.bbsize: return True
		for x in self.bbsize:
			if x: return False
		return True

	def getAxisMask(self):
		msk = 0
		for i, x in enumerate(self.bbsize):
			if x: msk |= 1 << i
		return msk


class RotTrack(BaseTrack):
	def __init__(self, node, minFrame, maxFrame):
		BaseTrack.__init__(self, node, minFrame, maxFrame, "r")

	def getData(self):
		self.data = []
		for fno in xrange(self.minFrame, self.maxFrame+1):
			q = evalFrmRot(self.node, fno)
			self.data.append(q)

class PosTrack(BaseTrack):
	def __init__(self, node, minFrame, maxFrame):
		BaseTrack.__init__(self, node, minFrame, maxFrame, "t")

	def getData(self):
		self.data = []
		for fno in xrange(self.minFrame, self.maxFrame+1):
			v = evalFrmPos(self.node, fno)
			self.data.append(v)

class SclTrack(BaseTrack):
	def __init__(self, node, minFrame, maxFrame):
		BaseTrack.__init__(self, node, minFrame, maxFrame, "s")

	def getData(self):
		self.data = []
		for fno in xrange(self.minFrame, self.maxFrame+1):
			v = evalFrmScl(self.node, fno)
			self.data.append(v)


class AnimNode:
	def __init__(self, hrc, hnode, lvl):
		self.hrc = hrc
		self.hnode = hnode
		self.lvl = lvl
		self.rotTrk = None
		self.posTrk = None
		self.sclTrk = None
		self.rOrd = getRotOrd(hnode)
		self.xOrd = getXformOrd(hnode)
		if hasRotAnim(self.hnode):
			self.rotTrk = RotTrack(self.hnode, self.hrc.minFrame, self.hrc.maxFrame)
		if hasPosAnim(self.hnode):
			self.posTrk = PosTrack(self.hnode, self.hrc.minFrame, self.hrc.maxFrame)
		if hasSclAnim(self.hnode):
			self.sclTrk = SclTrack(self.hnode, self.hrc.minFrame, self.hrc.maxFrame)

	def getName(self):
		if self.hnode: return self.hnode.name()
		return "<???>"

	def hasRot(self): return self.rotTrk != None
	def hasPos(self): return self.posTrk != None
	def hasScl(self): return self.sclTrk != None
	def isAnimated(self):
		return self.hasRot() or self.hasPos() or self.hasScl()

	def isConstRot(self):
		if self.hasRot(): return self.rotTrk.isConst()
		return True

	def getRotData(self):
		if self.hasRot(): return self.rotTrk.data
		return None

	def getRotMask(self):
		if self.hasRot(): return self.rotTrk.getAxisMask()
		return 0

	def isConstPos(self):
		if self.hasPos(): return self.posTrk.isConst()
		return True

	def getPosData(self):
		if self.hasPos(): return self.posTrk.data
		return None

	def getPosMask(self):
		if self.hasPos(): return self.posTrk.getAxisMask()
		return 0

	def isConstScl(self):
		if self.hasScl(): return self.sclTrk.isConst()
		return True

	def getSclData(self):
		if self.hasScl(): return self.sclTrk.data
		return None

	def getSclMask(self):
		if self.hasScl(): return self.sclTrk.getAxisMask()
		return 0


class AnimHrc:
	def __init__(self, rootPath, minFrame, maxFrame):
		self.fps = hou.fps()
		self.minFrame = int(minFrame)
		self.maxFrame = int(maxFrame)
		self.nodes = []
		self.build(hou.node(rootPath), 0)
		self.maxLvl = 0
		for node in self.nodes:
			self.maxLvl = max(self.maxLvl, node.lvl)

	def build(self, hnode, lvl):
		self.nodes.append(AnimNode(self, hnode, lvl))
		for link in hnode.outputConnectors()[0]:
			self.build(link.outputNode(), lvl+1)

class NodeList:
	def __init__(self):
		self.nodes = []
	
	def netBuild(self, rootPath="/obj"):
		for node in hou.node(rootPath).children(): self.netBuildSub(node)

	def netBuildSub(self, node):
		self.nodes.append(node)
		for sub in node.children(): self.netBuildSub(sub)

	def hrcBuild(self, rootPath="/obj/root"):
		rootNode = hou.node(rootPath)
		self.hrcBuildSub(rootNode)

	def hrcBuildSub(self, node):
		self.nodes.append(node)
		for link in node.outputConnectors()[0]:
			self.hrcBuildSub(link.outputNode())


def getChannelGroups():
	lst = hou.hscript("chgls")[0].split()
	return lst

def getChannelsInGroup(grpName):
	lst = hou.hscript("chgls -l " + grpName)[0].split()
	if len(lst): del lst[0] # remove group name from the list
	return lst

def isRotChannel(chPath):
	return chPath.endswith("/rx") or chPath.endswith("/ry") or chPath.endswith("/rz")

class FcvNameInfo:
	def __init__(self, chName, nodeName, nodePath):
		self.chName = chName
		self.nodeName = nodeName
		self.nodePath = nodePath

class FCurve:
	def __init__(self, chPath, minFrame, maxFrame, nameInfo = None):
		self.minFrame = int(minFrame)
		self.maxFrame = int(maxFrame)
		self.minVal = 0.0
		self.maxVal = 0.0
		self.slopesFlg = False
		self.sameFuncFlg = False
		self.cmnFunc = -1
		self.prm = hou.parm(chPath)
		if nameInfo:
			self.chName = nameInfo.chName
			self.nodeName = nameInfo.nodeName
			self.nodePath = nameInfo.nodePath
		else:
			self.chName = self.prm.name()
			self.nodeName = self.prm.node().name()
			self.nodePath = "/"
			nodePath = self.prm.node().path()
			sep = nodePath.rfind("/")
			if sep >= 0: self.nodePath = nodePath[:sep]
		self.keys = getParamKeys(self.prm)
		if not self.keys: return
		baked = []
		for fno in xrange(self.minFrame, self.maxFrame+1):
			baked.append(self.prm.evalAtFrame(fno))
		self.minVal = baked[0]
		self.maxVal = self.minVal
		for v in baked[1:]:
			self.minVal = min(self.minVal, v)
			self.maxVal = max(self.maxVal, v)

		if self.isConst(): return

		self.rangeKeys = []
		for k in self.keys:
			fno = int(k.frame())
			if fno >= self.minFrame and fno <= self.maxFrame:
				self.rangeKeys.append(k)

		n = len(self.rangeKeys)
		bakeFlg = n < 2 or int(self.rangeKeys[0].frame()) != self.minFrame or int(self.rangeKeys[n-1].frame()) != self.maxFrame
		validFuncLst = ["constant()", "linear()", "cubic()"]
		if not bakeFlg:
			for k in self.rangeKeys:
				funcFlg = k.isExpressionSet() and (k.expression() in validFuncLst)
				if not funcFlg:
					bakeFlg = True
					break

		if bakeFlg:
			self.sameFuncFlg = True
			self.cmnFunc = FCurve.F_LINEAR
			self.vals = baked
		else:
			self.segFunc = []
			for k in self.rangeKeys:
				self.segFunc.append(validFuncLst.index(k.expression()))
			func0 = self.segFunc[0]
			self.sameFuncFlg = True
			for i, func in enumerate(self.segFunc):
				if func != func0:
					self.sameFuncFlg = False
					break
			if self.sameFuncFlg:
				self.cmnFunc = func0
			self.vals = []
			self.fnos = []
			self.lslopes = []
			self.rslopes = []
			for i, k in enumerate(self.rangeKeys):
				self.fnos.append(int(k.frame()) - self.minFrame)
				self.vals.append(k.value())
				lslope = 0
				rslope = 0
				if self.segFunc[i] == FCurve.F_CUBIC and k.isSlopeSet():
					self.slopesFlg = True
					if k.isSlopeTied():
						lslope = k.slope()
					else:
						lslope = k.inSlope()
					rslope = k.slope()
				self.lslopes.append(lslope)
				self.rslopes.append(rslope)

		#print self.nodePath, self.nodeName, self.chName, self.isConst()

	def isConst(self):
		return self.maxVal - self.minVal == 0

	def isAnimated(self):
		return self.keys != None

	def isBaked(self):
		return len(self.vals) == self.maxFrame - self.minFrame + 1

	def isSameFunc(self):
		if self.isConst(): return True
		return self.sameFuncFlg

	def getKeyNum(self):
		if not self.isAnimated(): return 0
		if self.isConst(): return 0
		if self.isBaked(): return self.maxFrame - self.minFrame + 1
		return len(self.vals)

FCurve.F_CONSTANT = 0
FCurve.F_LINEAR = 1
FCurve.F_CUBIC = 2


class BBox:
	def __init__(self): self.reset()
	def reset(self): self.bbox = None

	def isValid(self):
		if self.bbox: return self.bbox.isValid()
		return False

	def addPos(self, pos):
		if self.bbox:
			self.bbox.enlargeToContain(pos)
		else:
			self.bbox = hou.BoundingBox(pos[0], pos[1], pos[2], pos[0], pos[1], pos[2])

	def fromHouBBox(self, hbb):
		vmin = hbb.minvec()
		vmax = hbb.maxvec()
		self.bbox = hou.BoundingBox(vmin[0], vmin[1], vmin[2], vmax[0], vmax[1], vmax[2])

	def addBox(self, box):
		if self.bbox:
			if box.isValid():
				self.bbox.enlargeToContain(box.bbox)
		else:
			self.fromHouBBox(box.bbox)

	def getMinPos(self):
		if self.isValid(): return self.bbox.minvec()
		return None

	def getMaxPos(self):
		if self.isValid(): return self.bbox.maxvec()
		return None

	def getCenter(self):
		if self.isValid(): return self.bbox.center()
		return None

	def getSize(self):
		if self.isValid(): return self.bbox.sizevec()
		return None

	def getMaxExtent(self):
		if self.isValid():
			s = self.getSize()
			return max(s[0], max(s[1], s[2]))
		return 0.0

	def getMaxAxis(self):
		if self.isValid():
			s = self.getSize()
			d = self.getMaxExtent()
			if d == s[0]: return 0
			if d == s[1]: return 1
			return 2
		return -1

class PntSkin:
	def __init__(self, pnt, skinAttr):
		self.pnt = pnt
		skin = pnt.floatListAttribValue(skinAttr)
		nwgt = len(skin)/2
		iw = []
		for i in xrange(nwgt):
			idx = int(skin[i*2])
			if idx >= 0: iw.append([idx, skin[i*2 + 1]])
		iw.sort(cmp=lambda iw0, iw1 : -cmp(iw0[1], iw1[1]))
		self.idx = []
		self.wgt = []
		for i in xrange(len(iw)):
			self.idx.append(iw[i][0])
			self.wgt.append(iw[i][1])

	def getWgtNum(self): return len(self.idx)

	def findWgt(self, id):
		if id in self.idx:
			i = self.idx.index(id)
			return self.wgt[i]
		return 0.0


class MtlInfo:
	def __init__(self, shopPath):
		self.path = shopPath
		self.shop = hou.node(shopPath)
		self.mtl = None
		self.baseMapPath = ""
		shopTypeName = self.shop.type().name()
		if shopTypeName == "material":
			for sub in self.shop.children():
				if sub.type().name() == "suboutput":
					mtl = sub.inputConnections()[0].inputNode()
					if mtl.type().name() == "v_layered":
						# for uvquickshade SOP
						self.mtl = mtl
						prm = mtl.parm("map_base")
						self.baseMapPath = prm.evalAsString()
					elif mtl.type().name() == "vopsurface":
						# Surface Shader Builder
						self.mtl = mtl
						prm = mtl.parm("ogl_tex1")
						if prm: self.baseMapPath = prm.evalAsString()
					elif mtl.type().name() == "GL_Basic":
						self.mtl = mtl
						prm = mtl.parm("g_texBase")
						self.baseMapPath = prm.evalAsString()
		elif shopTypeName == "mantrasurface":
			self.mtl = self.shop
			prm = self.mtl.parm("diff_colorTexture")
			if prm: self.baseMapPath = prm.evalAsString()
		elif shopTypeName == "principledshader":
			self.mtl = self.shop
			prm = self.mtl.parm("basecolor_texture")
			if prm: self.baseMapPath = prm.evalAsString()

def getSOPMtlList(sop):
	geo = sop.geometry()
	atr = geo.findPrimAttrib("shop_materialpath")
	mtls = []
	if atr:
		map = {}
		for prim in geo.prims():
			shopPath = prim.attribValue(atr)
			if not shopPath in map:
				id = len(mtls)
				map[shopPath] = id
				mtls.append(MtlInfo(shopPath))
	return mtls

def getSOPBaseMapList(sop):
	mtls = getSOPMtlList(sop)
	texs = []
	if mtls and len(mtls):
		for mtl in mtls:
			tpath = mtl.baseMapPath
			if tpath and len(tpath) and not tpath in texs:
				texs.append(tpath)
	return texs
