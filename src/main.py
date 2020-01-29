from feeder import Feeder
from wall import Wall
from brick import Small, Big

def main():
     #TODO : input the taking coordinates for the b6 runner
    taking_pose_b6 = Pose()
    taking_pose_b6.position.x = 0
    taking_pose_b6.position.y = -0.5
    taking_pose_b6.position.y = 0
    taking_pose_b6.orientation.x = 0
    taking_pose_b6.orientation.y = 0
    taking_pose_b6.orientation.z = 0
    taking_pose_b6.orientation.w = 1

    f1 = Feeder(6, Big(), taking_pose_b6)

    #TODO : input the taking coordinates for the b5 runner
    taking_pose_b5 = Pose()
    taking_pose_b5.position.x = -0.3
    taking_pose_b5.position.y = -0.5
    taking_pose_b5.position.y = 0
    taking_pose_b5.orientation.x = 0
    taking_pose_b5.orientation.y = 0
    taking_pose_b5.orientation.z = 0
    taking_pose_b5.orientation.w = 1

    f2 = Feeder(5, Big(), taking_pose_b5)

    #TODO : input the taking coordinates for the s2 runner
    taking_pose_s2 = Pose()
    taking_pose_s2.position.x = 0.3
    taking_pose_s2.position.y = -0.5
    taking_pose_s2.position.y = 0
    taking_pose_s2.orientation.x = 0
    taking_pose_s2.orientation.y = 0
    taking_pose_s2.orientation.z = 0
    taking_pose_s2.orientation.w = 1

    f3 = Feeder(2, Small(), taking_pose_s2)

    feeders = [f1,f2,f3]
    wall = Wall(feeders)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Killed by user')
        sys.exit(0)
