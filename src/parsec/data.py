class Data():
    def __init__(self, data_type):

        if data_type == "basic":
            self.faults = {
                1: (('object', 'cup', 'angle', 'orientation/object'), 'You should have kept the cup upright'),
                2: (('object', 'cup', 'speed', 'speed/object_max'), 'You should have moved more slowly when holding the mug'),
                3: (('object', 'cup', 'table', 'above/object_object'), 'You should have kept the cup over the table'),
                4: (('object', 'cup', 'human', 'john', 'distance', 'distance/object_human'), 'You should have kept the mug farther away from john'),
                5: (('object', 'block', 'speed', 'speed/object_max'), 'You should not have pushed the block so quickly'),
                6: (('object', 'cup', 'angle', 'orientation/object'), 'You should not have flipped the glass upside down'),
                7: (('object', 'cup', 'angle', 'orientation/object'), 'Do not upend the cup'),
                8: (('object', 'computer', 'robot', 'sawyer', 'distance', 'distance/object_robot'), 'You were too close to the computer'),
                9: (('object', 'knife', 'human', 'jane', 'distance', 'distance/object_human'), 'You were too close'),
                10: (('object', 'computer', 'speed', 'speed/object_max'), 'You moved to fast'),
                11: (('object', 'cup', 'angle', 'orientation/object'), 'The cup'),
                12: (('object', 'knife', 'human', 'john', 'distance', 'distance/object_human'), 'Knife')
            }
        elif data_type == "handover":
            self.faults = {
                1: (('object', 'cup', 'angle', 'orientation/object'), "You should not tip the cup over when it is bringing it over to the person"),
                2: (('object', 'cup', 'angle', 'orientation/object'), "Make sure to keep the cup upright"),
                3: (('object', 'cup', 'angle', 'orientation/object'), "Don't turn the cup while moving it"),
                4: (('object', 'cup', 'angle', 'orientation/object'), "Keep the cup pointed up"),
                5: (('object', 'cup', 'angle', 'orientation/object'), "You should be aware of the orientation of the cup as it is moving said cup"),
                6: (('object', 'cup', 'angle', 'orientation/object'), "Keep the cup upright"),
                7: (('object', 'cup', 'angle', 'orientation/object'), "Keep the cup oriented so that the ball does not fall out when moving"),
                8: (('object', 'cup', 'angle', 'orientation/object'), "You shouldn't tip the cup over"),
                9: (('object', 'cup', 'angle', 'orientation/object'), "You should keep the cup level as you move it towards the person"),
                10: (('object', 'cup', 'angle', 'orientation/object'), "Don't turn the cup upside down"),
                11: (('object', 'cup', 'angle', 'orientation/object'), "You shouldn't tilt the cup until you are near the human/goal"),
                12: (('object', 'cup', 'angle', 'orientation/object'), "Keep the bottom of the cup parallel to the floor when moving"),
                13: (('object', 'cup', 'angle', 'orientation/object'), "Don't rotate your arm"),
                14: (('object', 'cup', 'angle', 'orientation/object'), "You dropped the object before reaching near the subject Drop the object only when are you closest to subject"),
                15: (('object', 'cup', 'angle', 'orientation/object'), "The mechanical hand should not roll as the arm is turning towards the person"),
                16: (('object', 'cup', 'angle', 'orientation/object'), "You rotated the cup unnecessarily and prematurely"),
                17: (('object', 'cup', 'angle', 'orientation/object'), "Do not tilt the cup when turning Priority placed on facing the cup up"),
                18: (('object', 'cup', 'angle', 'orientation/object'), "Keep the cup upright when you hand it to me, and stay away from my head")
            }
        elif data_type == "pour":
            self.faults = {
                1: (('object', 'cup', 'container', 'above/object_object'), "You needs to make sure it is over the Tupperware before dropping the ball out of the cup"),
                2: (('object', 'cup', 'container', 'above/object_object'), "Don't pour until you are over the container"),
                3: (('object', 'cup', 'container', 'above/object_object'), "You moved too far before pouring"),
                4: (('object', 'cup', 'container', 'above/object_object'), "Don't dump the ball until you are over the tupperware"),
                5: (('object', 'cup', 'container', 'above/object_object'), "You should stop at the cup before pouring"),
                6: (('object', 'cup', 'container', 'above/object_object'), "Pour the cup over the receptacle"),
                7: (('object', 'cup', 'container', 'above/object_object'), "Make sure the cup is over the container before pouring the ball out of the cup "),
                8: (('object', 'cup', 'container', 'above/object_object'), "You needs to pour into the container"),
                9: (('object', 'cup', 'container', 'above/object_object'), "You should pour into the plastic container"),
                10: (('object', 'cup', 'container', 'above/object_object'), "Make sure the cup us over the container before dumping it out"),
                11: (('object', 'cup', 'container', 'above/object_object'), "You should stop over the container and pour only when you are over the container"),
                12: (('object', 'cup', 'container', 'above/object_object'), "Make sure gripper is as close as possible to the bin height wise, then make sure the side of the cup that is going to be tipped over is at the edge of the bin, so the contents fall into the bin"),
                13: (('object', 'cup', 'container', 'above/object_object'), "Rotate your arm once its above the container"),
                14: (('object', 'cup', 'container', 'above/object_object'), "You are pouring at the wrong location Pour the object when you are directly above the container"),
                15: (('object', 'cup', 'container', 'above/object_object'), "Drop the ball in the container"),
                16: (('object', 'cup', 'container', 'above/object_object'), "You failed to identify the target bowl"),
                17: (('object', 'cup', 'container', 'above/object_object'), "Stop over the bowl and pour into it"),
                18: (('object', 'cup', 'container', 'above/object_object'), "Make sure you pour the cup into the bowl")
            }
        elif data_type == "cleaning":
            self.faults = {
                1: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "You needs to respect the space around the human It should not go near the face of the person"),
                2: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Watch out for people in your workspace"),
                3: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Stay out of the human's vicinity while moving"),
                4: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Always maintain an acceptable distance from a partner you are working with"),
                5: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "You should know where the human is"),
                6: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Keep farther away from the human"),
                7: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "When moving stay further away from the person then in the previous iteration"),
                8: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "You should keep away from the person"),
                9: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "You should avoid getting too close to the person"),
                10: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Stay away from the person at the table"),
                11: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "You should not come so close to the human"),
                12: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Keep arm at least a foot away from the working space of the person "),
                13: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Don't go too close to other objects"),
                14: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Don't move objects close to subject while cleaning Keep maximum distance from the subject while cleaning"),
                15: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "You should keep a distance of two feet from the person's face when moving"),
                16: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "You should be more aware of the surrounding space during motion"),
                17: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Keep a safe distance from me when you are cleaning objects"),
                18: (('robot', 'sawyer', 'human', 'person', 'distance', 'distance/robot_human'), "Stay away from my face when you are cleaning")
            }
        elif data_type == "generated":
            self.faults = {
                "You failed in passing the ball4": (('object', 'cup', 'angle', 'orientation/object'), "Do not flip the cup before passing the ball"),
                "You failed in passing the ball10": (('object', 'cup', 'angle', 'orientation/object'), "The ball falls down when you flip the cup"),
                "You failed in passing the ball6": (('object', 'cup', 'angle', 'orientation/object'), "Pass me the cup without flipping it"),
                "You failed in passing the ball3": (('object', 'cup', 'angle', 'orientation/object'), "No need to flip the cup"),
                "You failed in passing the ball1": (('object', 'cup', 'angle', 'orientation/object'), "Keep the cup upright"),
                "You failed in passing the ball9": (('object', 'cup', 'angle', 'orientation/object'), "Ensure that the ball is always in the cup"),
                "You failed in passing the ball2": (('object', 'cup', 'angle', 'orientation/object'), "Pass the cup so that I can take the ball"),
                "You failed in passing the ball7": (('object', 'cup', 'angle', 'orientation/object'), "Pass me the ball without dropping it"),
                "You failed in passing the ball8": (('object', 'cup', 'angle', 'orientation/object'), "Do not drop the ball"),
                "You failed in passing the ball5": (('object', 'cup', 'angle', 'orientation/object'), "Pass me the ball"),
                "The pouring task is wrong2": (('object', 'cup', 'container', 'above/object_object'), "You need to pour the cup above the container"),
                "The pouring task is wrong5": (('object', 'cup', 'container', 'above/object_object'), "The container is to the left of the cup"),
                "The pouring task is wrong6": (('object', 'cup', 'container', 'above/object_object'), "The container is not in the center of the table"),
                "The pouring task is wrong10": (('object', 'cup', 'container', 'above/object_object'), "The ball should be dropped in the container"),
                "The pouring task is wrong8": (('object', 'cup', 'container', 'above/object_object'), "You did not put the ball in the container"),
                "The pouring task is wrong7": (('object', 'cup', 'container', 'above/object_object'), "Put the ball in the container"),
                "The pouring task is wrong4": (('object', 'cup', 'container', 'above/object_object'), "The cup needs to be above it"),
                "The pouring task is wrong1": (('object', 'cup', 'container', 'above/object_object'), "The container is over there"),
                "The pouring task is wrong3": (('object', 'cup', 'container', 'above/object_object'), "The container is not there"),
                "The pouring task is wrong9": (('object', 'cup', 'container', 'above/object_object'), "That is not where you drop the ball"),
                "Cleaning Task gone wrong5": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "Keep your distance from john with the cup"),
                "Cleaning Task gone wrong9": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "Keep more space between john and the cup"),
                "Cleaning Task gone wrong1": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "You almost hit john with the cup"),
                "Cleaning Task gone wrong7": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "You should be far from john"),
                "Cleaning Task gone wrong2": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "You were too close to john"),
                "Cleaning Task gone wrong4": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "Stay away from john"),
                "Cleaning Task gone wrong3": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "You scared john"),
                "Cleaning Task gone wrong10": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "You are not utilizing the common space properly"),
                "Cleaning Task gone wrong6": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "You should do the task near you"),
                "Cleaning Task gone wrong8": (('robot', 'sawyer', 'human', 'john', 'distance', 'distance/robot_human'), "You did not do the task safely")
            }
        elif data_type == "rl":
            self.faults = {
                1: (('area', 'bottom', 'left', 'location/area_area'), "You shouldn't go into the bottom left area"),
                2: (('area', 'bottom', 'right', 'location/area_area'), "You shouldn't go into the bottom right area"),
                3: (('area', 'bottom', 'dimension', 'small', 'location/area_size'), "You shouldn't go into a small area in the bottom right area"),
                4: (('area', 'bottom', 'cell', 'cell/cell_area'), "You shouldn't go into a specific cell in the bottom of the maze"),
                5: (('direction', 'east', 'cell', 'cell/cell_area'), "You shouldn't go into a specific cell from a certain direction")
            }
        else:
            print("Bad data_type: {}".format(data_type))
