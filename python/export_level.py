import bpy
import math
import json
scene = bpy.data.scenes["Scene"]

objects = bpy.context.selected_objects

class ParkingLot:
    def __init__(self, name, pos, w, h, parking_spots):
        self.name = name
        self.pos = pos
        self.w = w
        self.h = h
        self.parking_spots = parking_spots
        
    def __str__(self):
        spots = ['  parking_spots {{ position {{ x:{0} y:{1} }} }}'.format(spot[0], spot[1])
                  for spot in self.parking_spots]
        return 'parking_lot {{\n name:"{0}" position {{ x:{1} y:{2} }} width:{3} height:{4} '.format(
            self.name, self.pos[0], self.pos[1], self.w, self.h) + '\n' + '\n'.join(spots) + '\n}\n'

class RoadSegment:
    def __init__(self, name):
        self.name = name
        self.points = []
    
    def __str__(self):
        points = ["points {{ x:{0} y:{1} }} ".format(c[0], c[1]) for c in self.points]
        return 'road_segment {{ name:"{0}" {1} }}\n'.format(self.name, ' '.join(points))

class IntersectionControl:
    def __init__(self, name, type, pos, dir, time=0):
        self.name = name
        self.type = type
        self.pos = (pos[0], pos[1])
        self.dir = dir
        self.time = time

    def __str__(self):
        return 'intersection_control {{ type:{0} name:"{1}" position {{ x:{2} y:{3} }} dir {{ x:{4} y:{5} }} time:{6} }}'.format(self.type, self.name, self.pos[0], self.pos[1], self.dir[0], self.dir[1], self.time)

def get_mesh_data(mesh_data, mat=None):
    if mat:
        vertices = []
        print(mat)
        for v in mesh_data.vertices:
            vout = mat * v.co
            vertices.append((vout.x, vout.y, v.co.z))
    else:
        vertices = [(v.co.x, v.co.y, v.co.z) for v in mesh_data.vertices]
    tris = [{"v": (p.vertices[0], p.vertices[1], p.vertices[2])} for p in mesh_data.polygons]

    material = {}
    if mesh_data.materials:
        m = mesh_data.materials[0]
        material['diffuse'] = (m.diffuse_color[0], m.diffuse_color[1], m.diffuse_color[2])
        material['specular'] = (m.specular_color[0], m.specular_color[1], m.specular_color[2])

    uvs = []
    if mesh_data.uv_layers.active != None:
        uv_layer = mesh_data.uv_layers.active.data        
        me = mesh_data
        ti = 0
        uvs = {}
        ordered_uvs = []
        for poly in mesh_data.polygons:
            inds = []
            for loop_index in range(poly.loop_start, poly.loop_start + poly.loop_total):
                uv = (uv_layer[loop_index].uv[0], uv_layer[loop_index].uv[1])
                if not uv in uvs:
                    ordered_uvs.append(uv)
                    uvs[uv] = len(uvs)
                inds.append(uvs[uv])
            tris[ti]["uv"] = tuple(inds)
            ti += 1
        uvs = ordered_uvs
    return {'verts': vertices, 'uvs': uvs, 'tris': tris, 'material': material}
    
exported_objects = []
for o in objects:
    if o.type == 'CURVE':
        print(o.name)
        i = 0
        for sp in o.data.splines:
            print("spline %d %d" % (len(sp.points), len(sp.bezier_points)))
            seg = RoadSegment(o.name + ".%03d" % i) 
            for p in sp.points:
                seg.points.append((p.co[0], p.co[1]))
            for bp in sp.bezier_points:
                print(bp.co)
            exported_objects.append(seg)
            i += 1    
    elif o.type == 'MESH':
        obj = {}
        obj['name'] = o.name
        # obj['mesh'] = mesh
        obj['pos'] = (o.location[0], o.location[1], o.location[2])
        obj['rot'] = math.atan2(-o.matrix_world[0][1], o.matrix_world[0][0])
        
        # Get the mesh data
        mesh = get_mesh_data(o.data)
        verts = mesh['verts']
        mn = (0, 0)
        mx = (0, 0)
        for v in verts:
            print(v)
            mn = (min(mn[0], v[0]), min(mn[1], v[1]))
            mx = (max(mx[0], v[0]), max(mn[1], v[1]))
        w = mx[0] - mn[0]
        h = mx[1] - mn[1]
        print('dims:', w, h)
        clocs = []
        for c in o.children:
            clocs.append((c.location[0] - o.location[0], c.location[1] - o.location[1]))
        
        obj = ParkingLot(o.name, (o.location[0], o.location[1]), w, h, clocs)
        print(obj)
        exported_objects.append(obj)
    elif o.type == 'EMPTY':
        print('emtpy')
        # The default direction is on y-axis
        dir = (o.matrix_world[0][1], o.matrix_world[1][1])

        control = None
        if o.name.startswith('Stop'):
            control = IntersectionControl(o.name, 'STOP', o.location, dir)
        elif o.name.startswith('Yield'):
            control = IntersectionControl(o.name, 'YIELD', o.location, dir)
        elif o.name.startswith('Lights'):
            control = IntersectionControl(o.name, 'LIGHTS', o.location, dir, o.get('time'))
        if control:
            exported_objects.append(control)                   

with open('/tmp/first_level.textpb', 'w') as f:
    for e in exported_objects:
        f.write(str(e))
    #model = json.dumps({'objects': exported_objects, 'joints': exported_joints})
#with open('/tmp/first_level.json', 'w') as f:
#    f.write("%s" % model)