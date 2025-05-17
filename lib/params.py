import yaml

from utils import EnvironmentParams, PhysicalParams, ElectricalParams, ThrusterData, ThrusterParams, Position

class ParamsManager:
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
            )
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