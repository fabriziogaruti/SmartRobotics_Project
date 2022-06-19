

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

arm_chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="arm_base_joint",
      origin_translation=[0, 0, 15],
      origin_orientation=[0.0, 0.0, 0.0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="shoulder_joint",
      origin_translation=[-0.05166, 0.0, 0.20271],
      origin_orientation=[0, 0, 1.5708],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="bottom_wrist_joint",
      origin_translation=[0.0, -0.05194, 0.269],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    ),
    URDFLink(
      name="elbow_joint",
      origin_translation=[0.0, 0, 0.13522],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="top_wrist_joint",
      origin_translation=[0.0, 0, 0.20994],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
    
], active_links_mask=[False,True,True,True,True,True])
target_position = [ 0.7, 0, 0.25]
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

arm_chain.plot(arm_chain.inverse_kinematics([2, 2, 2]), ax)
matplotlib.pyplot.show()
print("The angles of each joints are : ", arm_chain.inverse_kinematics(target_position))

