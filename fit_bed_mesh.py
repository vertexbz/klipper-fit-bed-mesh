# Fit Mesh Bed Leveling
#
# Copyright (C) 2023 Adam Makswiej <vertexbz@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, collections, json
from . import bed_mesh, probe

class FitBedMesh(bed_mesh.BedMeshCalibrate):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.exclude_object = self.printer.load_object(config, 'exclude_object')

        self.bed_mesh_config = ConfigWrapper(config.getsection("bed_mesh"))
        base_bed_mesh = config.printer.load_object(config, "bed_mesh")

        bed_mesh.BedMeshCalibrate.__init__(self, self.bed_mesh_config, base_bed_mesh)

        self.offset = config.getfloat('offset', 5.)
        self.density = config.getfloat('density', 20.)
        self.min_resolution = config.getint('min_resolution', 3)
        if self.density < 1.:
            config.error("fit_bed_mesh: density is too low")

        self.gcode.register_command(
            'FIT_BED_MESH_CALIBRATE', 
            self.cmd_FIT_BED_MESH_CALIBRATE,
            desc="Perform Fit Mesh Bed Leveling"
        )
        

    def _generate_points(self, error):
        self.points = []
        return

    def _get_adjusted_points(self):
        if not self.points:
            return [(0,0), (0,0), (0,0)]
        return bed_mesh.BedMeshCalibrate._get_adjusted_points(self)

    def _get_objects(self):
        return self.exclude_object.objects

    def cmd_FIT_BED_MESH_CALIBRATE(self, gcmd):
        if not self._get_objects():
            raise gcmd.error("fit_bed_mesh: No objects detected")

        points = self.generate_points(gcmd.error)
        if not points:
            raise gcmd.error("fit_bed_mesh: Unable to generate coordinates")

        self.substituted_indices = self._validate_points(points, gcmd.error)
        self.points = points

        self.probe_helper = probe.ProbePointsHelper(self.bed_mesh_config, self.probe_finalize, self._get_adjusted_points())
        self.probe_helper.minimum_points(3)
        self.probe_helper.use_xy_offsets(True)
 
        self.cmd_BED_MESH_CALIBRATE(gcmd)

    def _object_to_points(self, object):
        polygon = object["polygon"]

        x_min = min(map(lambda point : point[0], polygon)) - self.offset
        y_min = min(map(lambda point : point[1], polygon)) - self.offset
        x_max = max(map(lambda point : point[0], polygon)) + self.offset
        y_max = max(map(lambda point : point[1], polygon)) + self.offset

        x_min = max([self.mesh_min[0], x_min])
        y_min = max([self.mesh_min[1], y_min])
        x_max = min([self.mesh_max[0], x_max])
        y_max = min([self.mesh_max[1], y_max])

        x_cnt = max([int((x_max - x_min) / self.density), self.min_resolution])
        y_cnt = max([int((y_max - y_min) / self.density), self.min_resolution])

        x_dist = (x_max - x_min) / (x_cnt - 1)
        y_dist = (y_max - y_min) / (y_cnt - 1)

        pos_y = y_min
        points = []
        for i in range(y_cnt):
            for j in range(x_cnt):
                if not i % 2:
                    # move in positive directon
                    pos_x = x_min + j * x_dist
                else:
                    # move in negative direction
                    pos_x = x_max - j * x_dist
                if self.radius is None:
                    # rectangular bed, append
                    points.append((pos_x, pos_y))
                else:
                    # round bed, check distance from origin
                    dist_from_origin = math.sqrt(pos_x*pos_x + pos_y*pos_y)
                    if dist_from_origin <= self.radius:
                        points.append(
                            (self.origin[0] + pos_x, self.origin[1] + pos_y))
            pos_y += y_dist

        return points

    def generate_points(self, error):
        pointsLists = map(self._object_to_points, self._get_objects())
        return [point for points in pointsLists for point in points]

    def _validate_points(self, points, error):
        x_cnt = self.mesh_config['x_count']
        y_cnt = self.mesh_config['y_count']
        min_x, min_y = self.mesh_min
        max_x, max_y = self.mesh_max
        x_dist = (max_x - min_x) / (x_cnt - 1)
        y_dist = (max_y - min_y) / (y_cnt - 1)
        # floor distances down to next hundredth
        x_dist = math.floor(x_dist * 100) / 100
        y_dist = math.floor(y_dist * 100) / 100
        if x_dist < 1. or y_dist < 1.:
            raise error("bed_mesh: min/max points too close together")

        if self.radius is not None:
            # round bed, min/max needs to be recalculated
            y_dist = x_dist
            new_r = (x_cnt // 2) * x_dist
            min_x = min_y = -new_r
            max_x = max_y = new_r
        else:
            # rectangular bed, only re-calc max_x
            max_x = min_x + x_dist * (x_cnt - 1)

        substituted_indices = collections.OrderedDict() 

        if not self.faulty_regions:
            return substituted_indices
   
        # Check to see if any points fall within faulty regions
        last_y = points[0][1]
        is_reversed = False
        for i, coord in enumerate(points):
            if not bed_mesh.isclose(coord[1], last_y):
                is_reversed = not is_reversed
            last_y = coord[1]
            adj_coords = []
            for min_c, max_c in self.faulty_regions:
                if bed_mesh.within(coord, min_c, max_c, tol=.00001):
                    # Point lies within a faulty region
                    adj_coords = [
                        (min_c[0], coord[1]), (coord[0], min_c[1]),
                        (coord[0], max_c[1]), (max_c[0], coord[1])]
                    if is_reversed:
                        # Swap first and last points for zig-zag pattern
                        first = adj_coords[0]
                        adj_coords[0] = adj_coords[-1]
                        adj_coords[-1] = first
                    break
            if not adj_coords:
                # coord is not located within a faulty region
                continue
            valid_coords = []
            for ac in adj_coords:
                # make sure that coordinates are within the mesh boundary
                if self.radius is None:
                    if bed_mesh.within(ac, (min_x, min_y), (max_x, max_y), .000001):
                        valid_coords.append(ac)
                else:
                    dist_from_origin = math.sqrt(ac[0]*ac[0] + ac[1]*ac[1])
                    if dist_from_origin <= self.radius:
                        valid_coords.append(ac)
            if not valid_coords:
                raise error("fit_bed_mesh: Unable to generate coordinates"
                            " for faulty region at index: %d" % (i))

            substituted_indices[i] = valid_coords

        return substituted_indices

def load_config(config):
    return FitBedMesh(config)

class ConfigWrapper(object):
    def __init__(self, config):
        self.config = config

    def __getattr__(self, attr):
        if attr == "printer":
            return PrinterWrapper(self.config.printer)
        return getattr(self.config, attr)

    def get_printer(self):
        return PrinterWrapper(self.config.get_printer())

class PrinterWrapper(object):
    def __init__(self, printer):
        self.printer = printer

    def __getattr__(self, attr):
        return getattr(self.printer, attr)

    def lookup_object(self, name, default=None):
        obj = self.printer.lookup_object(name, default)
        if name == "gcode":
            return GcodeWrapper(obj)
        return obj

class GcodeWrapper(object):
    def __init__(self, gcode):
        self.gcode = gcode

    def __getattr__(self, attr):
        return getattr(self.gcode, attr)

    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if cmd != "BED_MESH_CALIBRATE":
            return self.gcode.register_command(cmd, func, when_not_ready, desc)

