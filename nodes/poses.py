from geometry_msgs.msg import Pose

use_ndt_map = True

if use_ndt_map:

    HOME = Pose()
    HOME.position.x = 0.0
    HOME.position.y = 0.0
    HOME.orientation.z = 0.0
    HOME.orientation.w = 1.0

    ROOM1 = Pose()
    ROOM1.position.x = 0.391741
    ROOM1.position.y = -0.567732
    ROOM1.orientation.z = 0.7072022009881881
    ROOM1.orientation.w = 0.7070113485068414

    ROOM2 = Pose()
    ROOM2.position.x = 0.5802170038223267
    ROOM2.position.y = 1.7951760292053223
    ROOM2.orientation.z = 0.7072022009881881
    ROOM2.orientation.w = 0.7070113485068414

    ROOM3 = Pose()
    ROOM3.position.x = 0.316734
    ROOM3.position.y = -1.32965
    ROOM3.orientation.z = 0.0
    ROOM3.orientation.w = 1.0

    ROOM4 = Pose()
    ROOM4.position.x = -0.386171
    ROOM4.position.y = -1.66915
    ROOM4.orientation.z = -1.0
    ROOM4.orientation.w = 0.0

else:
    HOME = Pose()
    HOME.position.x = 1.2030736207962036
    HOME.position.y = 2.258392810821533
    HOME.orientation.w = 1.0

    ROOM1 = Pose()
    ROOM1.position.x = 1.616312861442566
    ROOM1.position.y = 1.6851954460144043
    ROOM1.orientation.z = 0.7072022009881881
    ROOM1.orientation.w = 0.7070113485068414

    ROOM2 = Pose()
    ROOM2.position.x = 0.5802170038223267
    ROOM2.position.y = 1.7951760292053223
    ROOM2.orientation.z = 0.7072022009881881
    ROOM2.orientation.w = 0.7070113485068414

    ROOM3 = Pose()
    ROOM3.position.x = 0.8934671878814697
    ROOM3.position.y = 0.5834417343139648
    ROOM3.orientation.z = 0.9999994215024537
    ROOM3.orientation.w = 0.0010756369080212524

    ROOM4 = Pose()
    ROOM4.position.x = 1.4859161376953125
    ROOM4.position.y = 0.9865974187850952
    ROOM4.orientation.z = 0.0
    ROOM4.orientation.w = 1.0

ROOMS = [ROOM1, ROOM2, ROOM3, ROOM4]
