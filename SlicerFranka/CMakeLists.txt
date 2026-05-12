cmake_minimum_required(VERSION 3.16.3...3.19.7 FATAL_ERROR)

project(SlicerFranka)

set(MODULE_NAME SlicerFranka)
set(MODULE_TITLE ${MODULE_NAME})

set(MODULE_PYTHON_SCRIPTS
    ${MODULE_NAME}.py
    ${MODULE_NAME}Lib/__init__.py
    ${MODULE_NAME}Lib/constants.py
    ${MODULE_NAME}Lib/ros_interface.py
    ${MODULE_NAME}Lib/trajectory.py
    ${MODULE_NAME}Lib/visualization.py
    ${MODULE_NAME}Lib/ui_helpers.py
    ${MODULE_NAME}Lib/widget.py
    ${MODULE_NAME}Lib/logic.py
  )

set(MODULE_PYTHON_RESOURCES
    Resources/Icons/${MODULE_NAME}.png
    Resources/UI/${MODULE_NAME}.ui
  )

slicerMacroBuildScriptedModule(
  NAME ${MODULE_NAME}
#   TITLE ${MODULE_TITLE}
#   EXPORT_DIRECTIVE ${MODULE_EXPORT_DIRECTIVE}
  SCRIPTS ${MODULE_PYTHON_SCRIPTS}
  RESOURCES ${MODULE_PYTHON_RESOURCES}
  WITH_GENERIC_TESTS
)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_CATEGORY "Robotics")
set(EXTENSION_CONTRIBUTORS "Iseoluwa (MedCVR)")
set(EXTENSION_DESCRIPTION "SlicerFranka")
# set(EXTENSION_ICONURL "https://www.example.com/Slicer/Extensions/SlicerFranka.png")
# set(EXTENSION_SCREENSHOTURLS "https://www.example.com/Slicer/Extensions/SlicerFranka/Screenshots/1.png")
set(EXTENSION_DEPENDS "NA") # Specified as a list or "NA" if no dependencies

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
