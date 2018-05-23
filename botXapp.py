from botXsrc.botXexport import botXexport

"""
botXexport is a dictionary containing all the reusable components you
developed for the project, and you will use them in the main program.
"""
def main():

    print('starting app ...')
    robot = botXexport['peanut_hacks_bot']['module']()
    task = botXexport['peanut_hacks_demo']['module'](robot)

    robot.start()
    # task.run(target_object='cup')


    
    print('all tasks finished')

"""
This is the only script that should be running from terminal so that the
program can gather modules correctly, so we need to specify main as entry point.
"""
if __name__ == '__main__':
    main()
