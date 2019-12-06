sys.path.append(os.path.join(director.getDRCBaseDir(), 'src'))

import director_anymal_b_simple.startup
director_anymal_b_simple.startup.startup(robotSystem, globals())
