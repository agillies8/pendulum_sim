# This script is executed the first time the script node computes, or the next time
# it computes after this script is modified or the 'Reset' button is pressed.
#
# The following callback functions may be defined in this script:
#     setup(db): Called immediately after this script is executed
#     compute(db): Called every time the node computes (should always be defined)
#     cleanup(db): Called when the node is deleted or the reset button is pressed
# Available variables:
#    db: og.Database The node interface, attributes are exposed like db.inputs.foo
#                    Use db.log_error, db.log_warning to report problems.
#    og: The omni.graph.core module




def setup(db: og.Database):
    from omni.isaac.core.articulations import Articulation
    import numpy as np
    
    pass


def cleanup(db: og.Database):
    pass


def compute(db: og.Database):
    prim_path = "/World/pendulum_scene/Pendulum/rail"

    # wrap the prim as an articulation
    prim = Articulation(prim_path=prim_path, name="pendulum")    

    data = prim.get_joint_positions()
    db.outputs.joint_positions = data.tolist()
    
    return True