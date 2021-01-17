from source.world import World
from source.slam import SLAM2D
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--steps', type=int)
parser.add_argument('--num_landmarks', type=int)
parser.add_argument('--world_size', type=int, default=100)
parser.add_argument('--measurement_range', type=int, default=50.0)
parser.add_argument('--measurement_noise', type=int, default=2.0)
parser.add_argument('--motion_noise', type=int, default=2.0)
args = parser.parse_args()

if __name__ == '__main__':
    world = World(args.world_size, args.num_landmarks)
    slam = SLAM2D(args.steps, args.world_size, args.num_landmarks, args.measurement_noise, args.motion_noise)
    # Make data and run SLAM
    world.make_data(args.steps, args.measurement_range, args.motion_noise, args.measurement_noise)
    slam.run_slam(world.data)
    # Get estimated robot and landmark positions
    poses, landmarks = slam.get_poses_landmarks()
    # Print results
    slam.show_results()
    world.display_world(poses, landmarks)
