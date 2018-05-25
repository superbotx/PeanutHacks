# from botXsrc.botXexport import botXexport
# from botXsrc.peanut_arm_api.arm_component import ArmComponent
from botXsrc.peanut_arm_api.gripper_component import GripperComponent

"""
botXexport is a dictionary containing all the reusable components you
developed for the project, and you will use them in the main program.
"""
def main():

    print('starting app ...')

    # ac = ArmComponent()
    # ac.setup()
    # my_pose = dict()
    # my_pose['position'] = {'x': 0.2, 'y': -0.27, 'z': 0.48}
    # my_pose['orientation'] = {'x': 0.67, 'y': -0.15, 'z': -0.69, 'w': 0.17}
    # try:
    #     ac.move_to(my_pose)
    # except ValueError as e:
    #     print(e)

    gc = GripperComponent()
    gc.setup()
    units = {'data':"percent"}
    finger_positions = {'data':[30., 30., 30.]}
    try:
        gc.gripper_to(units, finger_positions)
    except ValueError as e:
        print(e)

    print('all tasks finished')

"""
This is the only script that should be running from terminal so that the
program can gather modules correctly, so we need to specify main as entry point.
"""
if __name__ == '__main__':
    main()
