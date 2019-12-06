import anymaldriverpanel
#import anymalfootstepplanner
import anymalplanner
#import autonomytaskpanel

#import robottasks_drs as rt_drs

from director import tasklaunchpanel
from director import applogic
from director import teleoppanel
import director.objectmodel as om

from director import viewcolors
from director import cameraview

from director import tasklaunchpanel
from collections import OrderedDict


def startup(robotSystem, globalsDict=None):
    rs = robotSystem

    viewBackgroundLightHandler = globalsDict['viewBackgroundLightHandler']

    # Panels:

    anymalPlanner = anymalplanner.AnymalPlanner(robotSystem)
    #anymalFootstepPlanner = anymalfootstepplanner.AnymalFootstepPlanner(robotSystem, anymalPlanner)

    #rs._add_fields(anymalPlanner=anymalPlanner)

    #rt_drs.robotSystem = rs

    #navigationDriver = navigationdriver.NavigationDriver()
    #navigationDriverPanel = navigationdriverpanel.init(navigationDriver, anymalPlanner)

    #autonomyTaskPanel = autonomytaskpanel.init(anymalPlanner, anymalFootstepPlanner)
    anymalDriverPanel = anymaldriverpanel.init(anymalPlanner)
    #anymalDriverPanel.signal_frame.connect(autonomyTaskPanel.updateFrame)
    #anymalDriverPanel.signal_frame.connect(navigationDriverPanel.updateFrame)



    # Export variables to globals so that they can be accessed from the console
    #if globalsDict is not None:
    if 1==0:

        globalsDict['anymalDriverPanel'] = anymalDriverPanel
        #globalsDict['autonomyTaskPanel'] = autonomyTaskPanel
        #globalsDict['anymalPlanner'] = anymalPlanner
        #globalsDict['anymalFootstepPlanner'] = anymalFootstepPlanner
