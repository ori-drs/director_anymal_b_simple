#!/bin/bash
git clone git@github.com:ori-drs/anymal_b_simple_description.git

# director dependencies (not managed by us)
git clone git@github.com:ori-drs/QtPropertyBrowser.git qt_property_browser
git clone git@github.com:ori-drs/ctkPythonConsole.git ctk_python_console
git clone git@github.com:ori-drs/PythonQt.git python_qt
git clone git@github.com:ori-drs/PointCloudLibraryPlugin.git pcl_plugin

# director and related
git clone git@github.com:ori-drs/vtk_ros.git
git clone git@github.com:ori-drs/director.git
# already cloned:
#git clone git@github.com:ori-drs/director_anymal_b_simple.git


# Module within repo needed from these:
# multisense_image_utils only (used in Director and vtk_ros)
git clone git@github.com:ori-drs/cv_utils.git
# forward_kinematics only (used in Director and pronto)
git clone git@github.com:ori-drs/kinematic_utils.git