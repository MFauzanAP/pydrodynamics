import yaml

from lib.utils import EnvironmentParams, ForceCoefficients, PhysicalParams, ElectricalParams, ThrusterData, ThrusterParams, Position, Velocity

class ParamsManager:
    """
        Class to manage and load vehicle parameters from a YAML file.
    """
    def __init__(self, path):
        self.key_map = {
            'gravity': 'env.gravity',
            'density': 'env.density',

            'mass': 'physical.mass',
            'volume': 'physical.volume',

            'xg': 'physical.com.x',
            'yg': 'physical.com.y',
            'zg': 'physical.com.z',

            'xb': 'physical.cob.x',
            'yb': 'physical.cob.y',
            'zb': 'physical.cob.z',

            'ixx': 'physical.inertia.x',
            'iyy': 'physical.inertia.y',
            'izz': 'physical.inertia.z',

            'apx': 'physical.projected_area.x',
            'apy': 'physical.projected_area.y',
            'apz': 'physical.projected_area.z',

            'drag_xu': 'physical.drag.x.u',
            'drag_xv': 'physical.drag.x.v',
            'drag_xw': 'physical.drag.x.w',
            'drag_xp': 'physical.drag.x.p',
            'drag_xq': 'physical.drag.x.q',
            'drag_xr': 'physical.drag.x.r',

            'drag_yu': 'physical.drag.y.u',
            'drag_yv': 'physical.drag.y.v',
            'drag_yw': 'physical.drag.y.w',
            'drag_yp': 'physical.drag.y.p',
            'drag_yq': 'physical.drag.y.q',
            'drag_yr': 'physical.drag.y.r',

            'drag_zu': 'physical.drag.z.u',
            'drag_zv': 'physical.drag.z.v',
            'drag_zw': 'physical.drag.z.w',
            'drag_zp': 'physical.drag.z.p',
            'drag_zq': 'physical.drag.z.q',
            'drag_zr': 'physical.drag.z.r',

            'drag_ku': 'physical.drag.k.u',
            'drag_kv': 'physical.drag.k.v',
            'drag_kw': 'physical.drag.k.w',
            'drag_kp': 'physical.drag.k.p',
            'drag_kq': 'physical.drag.k.q',
            'drag_kr': 'physical.drag.k.r',

            'drag_mu': 'physical.drag.m.u',
            'drag_mv': 'physical.drag.m.v',
            'drag_mw': 'physical.drag.m.w',
            'drag_mp': 'physical.drag.m.p',
            'drag_mq': 'physical.drag.m.q',
            'drag_mr': 'physical.drag.m.r',

            'drag_nu': 'physical.drag.n.u',
            'drag_nv': 'physical.drag.n.v',
            'drag_nw': 'physical.drag.n.w',
            'drag_np': 'physical.drag.n.p',
            'drag_nq': 'physical.drag.n.q',
            'drag_nr': 'physical.drag.n.r',

            'am_xu': 'physical.added_mass.x.u',
            'am_xv': 'physical.added_mass.x.v',
            'am_xw': 'physical.added_mass.x.w',
            'am_xp': 'physical.added_mass.x.p',
            'am_xq': 'physical.added_mass.x.q',
            'am_xr': 'physical.added_mass.x.r',

            'am_yu': 'physical.added_mass.y.u',
            'am_yv': 'physical.added_mass.y.v',
            'am_yw': 'physical.added_mass.y.w',
            'am_yp': 'physical.added_mass.y.p',
            'am_yq': 'physical.added_mass.y.q',
            'am_yr': 'physical.added_mass.y.r',

            'am_zu': 'physical.added_mass.z.u',
            'am_zv': 'physical.added_mass.z.v',
            'am_zw': 'physical.added_mass.z.w',
            'am_zp': 'physical.added_mass.z.p',
            'am_zq': 'physical.added_mass.z.q',
            'am_zr': 'physical.added_mass.z.r',

            'am_ku': 'physical.added_mass.k.u',
            'am_kv': 'physical.added_mass.k.v',
            'am_kw': 'physical.added_mass.k.w',
            'am_kp': 'physical.added_mass.k.p',
            'am_kq': 'physical.added_mass.k.q',
            'am_kr': 'physical.added_mass.k.r',

            'am_mu': 'physical.added_mass.m.u',
            'am_mv': 'physical.added_mass.m.v',
            'am_mw': 'physical.added_mass.m.w',
            'am_mp': 'physical.added_mass.m.p',
            'am_mq': 'physical.added_mass.m.q',
            'am_mr': 'physical.added_mass.m.r',

            'am_nu': 'physical.added_mass.n.u',
            'am_nv': 'physical.added_mass.n.v',
            'am_nw': 'physical.added_mass.n.w',
            'am_np': 'physical.added_mass.n.p',
            'am_nq': 'physical.added_mass.n.q',
            'am_nr': 'physical.added_mass.n.r',

            'voltage': 'electrical.voltage',
            'capacity': 'electrical.capacity',
        }

        self.params_folder = "/".join(path.split('/')[:-1]) + '/'
        self.load(path)

    def load(self, path):
        with open(path, 'r') as f:
            raw = yaml.safe_load(f)
        self.name = raw['name']
        self.verbose = raw['verbose']
        self.env = EnvironmentParams(**raw['env'])
        self.physical = PhysicalParams(
            mass=raw['physical']['mass'],
            volume=raw['physical']['volume'],
            com=Position(
                x=raw['physical']['com']['x'],
                y=raw['physical']['com']['y'],
                z=raw['physical']['com']['z']
            ),
            cob=Position(
                x=raw['physical']['cob']['x'],
                y=raw['physical']['cob']['y'],
                z=raw['physical']['cob']['z']
            ),
            inertia=Position(
                x=raw['physical']['inertia']['x'],
                y=raw['physical']['inertia']['y'],
                z=raw['physical']['inertia']['z']
            ),
            projected_area=Position(
                x=raw['physical']['projected_area']['x'],
                y=raw['physical']['projected_area']['y'],
                z=raw['physical']['projected_area']['z']
            ),
            drag=ForceCoefficients(
                x=Velocity(
                    u=raw['physical']['drag']['x']['u'],
                    v=raw['physical']['drag']['x']['v'],
                    w=raw['physical']['drag']['x']['w'],
                    p=raw['physical']['drag']['x']['p'],
                    q=raw['physical']['drag']['x']['q'],
                    r=raw['physical']['drag']['x']['r']
                ),
                y=Velocity(
                    u=raw['physical']['drag']['y']['u'],
                    v=raw['physical']['drag']['y']['v'],
                    w=raw['physical']['drag']['y']['w'],
                    p=raw['physical']['drag']['y']['p'],
                    q=raw['physical']['drag']['y']['q'],
                    r=raw['physical']['drag']['y']['r']
                ),
                z=Velocity(
                    u=raw['physical']['drag']['z']['u'],
                    v=raw['physical']['drag']['z']['v'],
                    w=raw['physical']['drag']['z']['w'],
                    p=raw['physical']['drag']['z']['p'],
                    q=raw['physical']['drag']['z']['q'],
                    r=raw['physical']['drag']['z']['r']
                ),
                k=Velocity(
                    u=raw['physical']['drag']['k']['u'],
                    v=raw['physical']['drag']['k']['v'],
                    w=raw['physical']['drag']['k']['w'],
                    p=raw['physical']['drag']['k']['p'],
                    q=raw['physical']['drag']['k']['q'],
                    r=raw['physical']['drag']['k']['r']
                ),
                m=Velocity(
                    u=raw['physical']['drag']['m']['u'],
                    v=raw['physical']['drag']['m']['v'],
                    w=raw['physical']['drag']['m']['w'],
                    p=raw['physical']['drag']['m']['p'],
                    q=raw['physical']['drag']['m']['q'],
                    r=raw['physical']['drag']['m']['r']
                ),
                n=Velocity(
                    u=raw['physical']['drag']['n']['u'],
                    v=raw['physical']['drag']['n']['v'],
                    w=raw['physical']['drag']['n']['w'],
                    p=raw['physical']['drag']['n']['p'],
                    q=raw['physical']['drag']['n']['q'],
                    r=raw['physical']['drag']['n']['r']
                ),
            ),
            added_mass=ForceCoefficients(
                x=Velocity(
                    u=raw['physical']['added_mass']['x']['u'],
                    v=raw['physical']['added_mass']['x']['v'],
                    w=raw['physical']['added_mass']['x']['w'],
                    p=raw['physical']['added_mass']['x']['p'],
                    q=raw['physical']['added_mass']['x']['q'],
                    r=raw['physical']['added_mass']['x']['r']
                ),
                y=Velocity(
                    u=raw['physical']['added_mass']['y']['u'],
                    v=raw['physical']['added_mass']['y']['v'],
                    w=raw['physical']['added_mass']['y']['w'],
                    p=raw['physical']['added_mass']['y']['p'],
                    q=raw['physical']['added_mass']['y']['q'],
                    r=raw['physical']['added_mass']['y']['r']
                ),
                z=Velocity(
                    u=raw['physical']['added_mass']['z']['u'],
                    v=raw['physical']['added_mass']['z']['v'],
                    w=raw['physical']['added_mass']['z']['w'],
                    p=raw['physical']['added_mass']['z']['p'],
                    q=raw['physical']['added_mass']['z']['q'],
                    r=raw['physical']['added_mass']['z']['r']
                ),
                k=Velocity(
                    u=raw['physical']['added_mass']['k']['u'],
                    v=raw['physical']['added_mass']['k']['v'],
                    w=raw['physical']['added_mass']['k']['w'],
                    p=raw['physical']['added_mass']['k']['p'],
                    q=raw['physical']['added_mass']['k']['q'],
                    r=raw['physical']['added_mass']['k']['r']
                ),
                m=Velocity(
                    u=raw['physical']['added_mass']['m']['u'],
                    v=raw['physical']['added_mass']['m']['v'],
                    w=raw['physical']['added_mass']['m']['w'],
                    p=raw['physical']['added_mass']['m']['p'],
                    q=raw['physical']['added_mass']['m']['q'],
                    r=raw['physical']['added_mass']['m']['r']
                ),
                n=Velocity(
                    u=raw['physical']['added_mass']['n']['u'],
                    v=raw['physical']['added_mass']['n']['v'],
                    w=raw['physical']['added_mass']['n']['w'],
                    p=raw['physical']['added_mass']['n']['p'],
                    q=raw['physical']['added_mass']['n']['q'],
                    r=raw['physical']['added_mass']['n']['r']
                ),
            ),
        )
        self.electrical = ElectricalParams(**raw['electrical'])
        self.thrusters = ThrusterParams(
            data=raw['thrusters']['data'],
            list=[ThrusterData(
                name=t['name'],
                pos=Position(t['pos']['x'], t['pos']['y'], t['pos']['z']),
                dir=Position(t['dir']['x'], t['dir']['y'], t['dir']['z'])
            ) for t in raw['thrusters']['list']]
        )

        # Add thrusters to key_map by name and by index
        for i, t in enumerate(raw['thrusters']['list']):
            name = t['name']
            self.key_map[f'{name}_x'] = f'thrusters.list.{i}.pos.x'
            self.key_map[f'{name}_y'] = f'thrusters.list.{i}.pos.y'
            self.key_map[f'{name}_z'] = f'thrusters.list.{i}.pos.z'
            self.key_map[f'{name}_dir_x'] = f'thrusters.list.{i}.dir.x'
            self.key_map[f'{name}_dir_y'] = f'thrusters.list.{i}.dir.y'
            self.key_map[f'{name}_dir_z'] = f'thrusters.list.{i}.dir.z'

            self.key_map[f't{i + 1}_x'] = f'thrusters.list.{i}.pos.x'
            self.key_map[f't{i + 1}_y'] = f'thrusters.list.{i}.pos.y'
            self.key_map[f't{i + 1}_z'] = f'thrusters.list.{i}.pos.z'
            self.key_map[f't{i + 1}_dir_x'] = f'thrusters.list.{i}.dir.x'
            self.key_map[f't{i + 1}_dir_y'] = f'thrusters.list.{i}.dir.y'
            self.key_map[f't{i + 1}_dir_z'] = f'thrusters.list.{i}.dir.z'

        if self.verbose: print(f'Successfully loaded params for vehicle {self.name} from {path}')

    def get(self, key):
        """Utility function to get a deep nested attribute from params."""
        if key not in self.key_map:
            raise KeyError(f'Key {key} not found in params')
        keys = self.key_map[key].split('.')
        value = self
        for k in keys:
            if str.isnumeric(k):
                k = int(k)
                value = value[k]
            else:
                value = getattr(value, k)
        return value