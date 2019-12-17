# feedback_visualizations

NOTE: The `mvt_plan_listener.py` module REQUIRES KDL to run! This is included in the Docker image of this repository though.

```
# In the src/ directory ...

git clone https://github.com/gt-ros-pkg/hrl-kdl.git
cd hrl-kdl/pykdl_utils/
python setup.py install
cd ../hrl_geom/
python setup.py install
cd ../..
catkin build
```