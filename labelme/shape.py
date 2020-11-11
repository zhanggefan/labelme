import copy
import math

from qtpy import QtCore
from qtpy import QtGui
import numpy as np
import labelme.utils

# TODO(unknown):
# - [opt] Store paths instead of creating new ones at each paint.


DEFAULT_LINE_COLOR = QtGui.QColor(0, 255, 0, 128)  # bf hovering
DEFAULT_FILL_COLOR = QtGui.QColor(0, 255, 0, 128)  # hovering
DEFAULT_SELECT_LINE_COLOR = QtGui.QColor(255, 255, 255)  # selected
DEFAULT_SELECT_FILL_COLOR = QtGui.QColor(0, 255, 0, 155)  # selected
DEFAULT_VERTEX_FILL_COLOR = QtGui.QColor(0, 255, 0, 255)  # hovering
DEFAULT_HVERTEX_FILL_COLOR = QtGui.QColor(255, 255, 255, 255)  # hovering


class Shape(object):
    P_SQUARE, P_ROUND = 0, 1

    MOVE_VERTEX, NEAR_VERTEX = 0, 1

    # The following class variables influence the drawing of all shape objects.
    line_color = DEFAULT_LINE_COLOR
    fill_color = DEFAULT_FILL_COLOR
    select_line_color = DEFAULT_SELECT_LINE_COLOR
    select_fill_color = DEFAULT_SELECT_FILL_COLOR
    vertex_fill_color = DEFAULT_VERTEX_FILL_COLOR
    hvertex_fill_color = DEFAULT_HVERTEX_FILL_COLOR
    point_type = P_ROUND
    point_size = 8
    scale = 1.0

    def __init__(
            self,
            label=None,
            line_color=None,
            shape_type=None,
            flags=None,
            group_id=None,
            orientation=0,
    ):
        self.label = label
        self.group_id = group_id
        self.points = []
        self.fill = False
        self.selected = False
        self.shape_type = shape_type
        self.flags = flags
        self.other_data = {}
        self.ori=0
        self.ori_sum=orientation
        self._highlightIndex = None
        self._highlightMode = self.NEAR_VERTEX
        self._highlightSettings = {
            self.NEAR_VERTEX: (4, self.P_ROUND),
            self.MOVE_VERTEX: (1.5, self.P_SQUARE),
        }

        self._closed = False

        if line_color is not None:
            # Override the class line_color attribute
            # with an object attribute. Currently this
            # is used for drawing the pending line a different color.
            self.line_color = line_color

        self.shape_type = shape_type

    @property
    def shape_type(self):
        return self._shape_type

    @shape_type.setter
    def shape_type(self, value):
        if value is None:
            value = "polygon"
        if value not in [
            "polygon",
            "rectangle",
            "rotation_rectangle",
            "point",
            "line",
            "circle",
            "linestrip",
        ]:
            raise ValueError("Unexpected shape_type: {}".format(value))
        self._shape_type = value

    def close(self):
        self._closed = True

    def addPoint(self, point):
        if self.points and point == self.points[0]:
            self.close()
        else:
            self.points.append(point)

    def canAddPoint(self):
        return self.shape_type in ["polygon", "linestrip"]

    def popPoint(self):
        if self.points:
            return self.points.pop()
        return None

    def insertPoint(self, i, point):
        self.points.insert(i, point)

    def removePoint(self, i):
        self.points.pop(i)

    def isClosed(self):
        return self._closed

    def setOpen(self):
        self._closed = False

    def getRectFromLine(self, pt1, pt2):
        x1, y1 = pt1.x(), pt1.y()
        x2, y2 = pt2.x(), pt2.y()
        return QtCore.QRectF(x1, y1, x2 - x1, y2 - y1)

    def getRotationPoints(self, ori,ori_sum,pt1, pt2): #ori_sum-ori:current orientation
        R_ro_to_rect = np.asarray([[math.cos(ori-ori_sum), -math.sin(ori-ori_sum)],
                                   [math.sin(ori-ori_sum), math.cos(ori-ori_sum)]])
        R_rect_to_ro=np.asarray([[math.cos(ori_sum),-math.sin(ori_sum)],
                                 [math.sin(ori_sum),math.cos(ori_sum)]])
        cur_center=np.asarray([[(pt1.x() + pt2.x()) * 0.5],
                               [(pt1.y() + pt2.y()) * 0.5]])
        cur_pt1=np.asarray([[pt1.x()],
                            [pt1.y()]])
        cur_pt2=np.asarray([[pt2.x()],
                            [pt2.y()]])
        raw_pt1=R_ro_to_rect.dot(cur_pt1-cur_center)+cur_center
        raw_pt2=R_ro_to_rect.dot(cur_pt2-cur_center)+cur_center
        wh=raw_pt2-raw_pt1
        center=raw_pt1+0.5*wh
        # center = np.asarray([[pt1.x() + 0.5 * w],
        #                      [pt1.y() + 0.5 * h]])
        offsets = np.asarray([[0, 0], [wh[0][0], 0], [wh[0][0], wh[1][0]], [0, wh[1][0]]])- center.reshape([1, 2])
        re = []
        for offset in offsets:
            ro_pt = R_rect_to_ro.dot(raw_pt1+offset.reshape(2,1))
            ro_pt += center
            re.append(QtCore.QPointF(ro_pt[0], ro_pt[1]))
        return re

    def paint(self, painter):
        if self.points:
            color = (
                self.select_line_color if self.selected else self.line_color
            )
            pen = QtGui.QPen(color)
            # Try using integer sizes for smoother drawing(?)
            pen.setWidth(max(1, int(round(2.0 / self.scale))))
            painter.setPen(pen)

            line_path = QtGui.QPainterPath()
            vrtx_path = QtGui.QPainterPath()

            if self.shape_type == "rectangle":
                assert len(self.points) in [1, 2]
                if len(self.points) == 2:
                    rectangle = self.getRectFromLine(*self.points)
                    line_path.addRect(rectangle)
                for i in range(len(self.points)):
                    self.drawVertex(vrtx_path, i)
            elif self.shape_type == "rotation_rectangle":
                assert len(self.points) in [1, 2]
                if len(self.points) == 2 :
                    lt,rt,rb,lb = self.getRotationPoints(self.ori,self.ori_sum,*self.points)
                    self.ori=0
                    line_path.moveTo(lt)
                    line_path.lineTo(rt)
                    line_path.lineTo(rb)
                    line_path.lineTo(lb)
                    line_path.lineTo(lt)
                    self.points=[lt,rb]
                    # self.points=lt,rt,rb,lb
                    #line_path.addRect(rectangle)

                for i in range(len(self.points)):
                    self.drawVertex(vrtx_path, i)

            elif self.shape_type == "circle":
                assert len(self.points) in [1, 2]
                if len(self.points) == 2:
                    rectangle = self.getCircleRectFromLine(self.points)
                    line_path.addEllipse(rectangle)
                for i in range(len(self.points)):
                    self.drawVertex(vrtx_path, i)
            elif self.shape_type == "linestrip":
                line_path.moveTo(self.points[0])
                for i, p in enumerate(self.points):
                    line_path.lineTo(p)
                    self.drawVertex(vrtx_path, i)
            else:
                line_path.moveTo(self.points[0])
                # Uncommenting the following line will draw 2 paths
                # for the 1st vertex, and make it non-filled, which
                # may be desirable.
                # self.drawVertex(vrtx_path, 0)

                for i, p in enumerate(self.points):
                    line_path.lineTo(p)
                    self.drawVertex(vrtx_path, i)
                if self.isClosed():
                    line_path.lineTo(self.points[0])

            painter.drawPath(line_path)
            painter.drawPath(vrtx_path)
            painter.fillPath(vrtx_path, self._vertex_fill_color)
            if self.fill:
                color = (
                    self.select_fill_color
                    if self.selected
                    else self.fill_color
                )
                painter.fillPath(line_path, color)

    def drawVertex(self, path, i):
        d = self.point_size / self.scale
        shape = self.point_type
        point = self.points[i]
        if i == self._highlightIndex:
            size, shape = self._highlightSettings[self._highlightMode]
            d *= size
        if self._highlightIndex is not None:
            self._vertex_fill_color = self.hvertex_fill_color
        else:
            self._vertex_fill_color = self.vertex_fill_color
        if shape == self.P_SQUARE:
            path.addRect(point.x() - d / 2, point.y() - d / 2, d, d)
        elif shape == self.P_ROUND:
            path.addEllipse(point, d / 2.0, d / 2.0)
        else:
            assert False, "unsupported vertex shape"

    def nearestVertex(self, point, epsilon):
        min_distance = float("inf")
        min_i = None
        for i, p in enumerate(self.points):
            dist = labelme.utils.distance(p - point)
            if dist <= epsilon and dist < min_distance:
                min_distance = dist
                min_i = i
        return min_i

    def nearestEdge(self, point, epsilon):
        min_distance = float("inf")
        post_i = None
        for i in range(len(self.points)):
            line = [self.points[i - 1], self.points[i]]
            dist = labelme.utils.distancetoline(point, line)
            if dist <= epsilon and dist < min_distance:
                min_distance = dist
                post_i = i
        return post_i

    def containsPoint(self, point):
        return self.makePath().contains(point)

    def getCircleRectFromLine(self, line):
        """Computes parameters to draw with `QPainterPath::addEllipse`"""
        if len(line) != 2:
            return None
        (c, point) = line
        r = line[0] - line[1]
        d = math.sqrt(math.pow(r.x(), 2) + math.pow(r.y(), 2))
        rectangle = QtCore.QRectF(c.x() - d, c.y() - d, 2 * d, 2 * d)
        return rectangle

    def makePath(self):
        if self.shape_type == "rectangle":
            path = QtGui.QPainterPath()
            if len(self.points) == 2:
                rectangle = self.getRectFromLine(*self.points)
                path.addRect(rectangle)
        elif self.shape_type == "rotation_rectangle":
            path = QtGui.QPainterPath()
            if len(self.points) == 2:
                lt, rt, rb, lb = self.getRotationPoints(self.ori, self.ori_sum,*self.points)
                self.ori=0
                path.moveTo(lt)
                path.lineTo(rt)
                path.lineTo(rb)
                path.lineTo(lb)
                path.lineTo(lt)
                self.points=[lt,rb]
        elif self.shape_type == "circle":
            path = QtGui.QPainterPath()
            if len(self.points) == 2:
                rectangle = self.getCircleRectFromLine(self.points)
                path.addEllipse(rectangle)
        else:
            path = QtGui.QPainterPath(self.points[0])
            for p in self.points[1:]:
                path.lineTo(p)
        return path

    def boundingRect(self):
        return self.makePath().boundingRect()

    def moveBy(self, offset):
        self.points = [p + offset for p in self.points]

    def moveVertexBy(self, i, offset):
        self.points[i] = self.points[i] + offset

    def highlightVertex(self, i, action):
        self._highlightIndex = i
        self._highlightMode = action

    def highlightClear(self):
        self._highlightIndex = None

    def copy(self):
        return copy.deepcopy(self)

    def __len__(self):
        return len(self.points)

    def __getitem__(self, key):
        return self.points[key]

    def __setitem__(self, key, value):
        self.points[key] = value


class HelpShape(Shape):
    def _inside(self, points, size):
        return points.x() >= 0 and points.y() >= 0 and points.x() <= size.width() - 1 and points.y() <= size.height() - 1

    def paint(self, painter, size):
        if self.points and self.points[0] != self.points[1]:
            color = (
                self.select_line_color if self.selected else self.line_color
            )
            pen = QtGui.QPen(color)
            # Try using integer sizes for smoother drawing(?)
            pen.setWidth(0.1)
            pen.setStyle(QtCore.Qt.DashLine)
            painter.setPen(pen)

            direct1 = direct2 = self.points[1] - self.points[0]
            while self._inside(self.points[0] + direct1, size):
                direct1 *= 2
            pt1 = self.points[0] + direct1
            while self._inside(self.points[0] - direct2, size):
                direct2 *= 2
            pt2 = self.points[0] - direct1

            painter.drawLine(self.points[1], pt1)
            painter.drawLine(self.points[0], pt2)
            pen.setStyle(QtCore.Qt.SolidLine)
            pen.setColor(QtGui.QColor(255, 0, 0, 128))
            painter.setPen(pen)
            painter.drawLine(*self.points)

class LimitBox(Shape):
    def paint(self, painter, size):
        if self.points and self.points[0] != self.points[1]:
            color = (
                self.select_line_color if self.selected else self.line_color
            )
            pen = QtGui.QPen(color)
            # Try using integer sizes for smoother drawing(?)
            pen.setWidth(0.1)
            pen.setStyle(QtCore.Qt.DashLine)
            painter.setPen(pen)

            direct1 = direct2 = self.points[1] - self.points[0]
            while self._inside(self.points[0] + direct1, size):
                direct1 *= 2
            pt1 = self.points[0] + direct1
            while self._inside(self.points[0] - direct2, size):
                direct2 *= 2
            pt2 = self.points[0] - direct1

            painter.drawLine(self.points[1], pt1)
            painter.drawLine(self.points[0], pt2)
            pen.setStyle(QtCore.Qt.SolidLine)
            pen.setColor(QtGui.QColor(255, 0, 0, 128))
            painter.setPen(pen)
            painter.drawLine(*self.points)
