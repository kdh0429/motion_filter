LINT_PATHS="motion_filters/ setup.py gui_main.py meshcat_viz.py"
echo ${LINT_PATHS}
isort ${LINT_PATHS}
black -l 127 ${LINT_PATHS}