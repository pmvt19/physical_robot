ASSIGN_ROOM_LABEL_ONLY_PROMPT = \
f"""
You are an image analysis expert.

Identify what kind of room the inputed image depicts.

Please answer with only the room (can be one or multiple words).
"""

ASSIGN_ROOM_LABEL = \
"""
You are an image analysis expert.

Identify what kind of room the inputed image depicts.

Here are a list of already existing rooms in the map: {}. 
Please reuse any existing room names if they fit the provided image.

These are a list of invalid room names: {}
Do not assign any of these room labels to the image.

Please answer with only the room (can be one or multiple words). Input this answer in the room_label field.
"""

EXTRACT_SEMANTIC_TARGETS = \
"""
The user provides a sentence providing a command for where the robot should travel to.

Extract the location and state whether it is a room level location (such as kitchen or office) or
object level location (such as table or bed).

User Input: {}

Please answer with whether it is a room or object level location and the location itself. 
Whether it is a room or an object is known as the semantic_level and the location itself is known as the item_name.
If you can deduce this information from the user input, mark the output as valid in the valid field.

These are a list of valid rooms that a user can plan for: {}
These are a list of valid objects that a user can plan for: {}
These are a list of invalid objects that a user should not be able to plan for: {}

If the user input is nonsensical or you cannot understand the semantic information from the user input, mark the output invalid for the valid field. 
Since in this case the semantic information is nonexistent, you can output "N/A" for both the semantic_level and item_name fields.

Additionally, please provide a reason in the reason field for why a user input was valid or invalid.
Some examples might be if the user input was nonsensical, then the reason would be something like "nonsensical user input".
If the user input was a room or object that does not exist in the list of valid objects, then the reason in the reasoning field could be stated as such.
"""

EXTRACT_POSE_TARGET = \
"""
The user provides a sentence providing a command for where the robot should travel to.

Extract the 2-dimensional location coordinates (x, y). Sometimes, the user would have provided a theta parameter as well.
In the case theta is provided, extract that as well.

User Input: {}

If the user has provided a theta value. Please set the valid_theta field in the output schema as true. 
If the user has not provided a valid theta value. Please set the valid_theta field in the output schema as false.

Here is the valid x-range: {}-{}
Here is the valid y-range: {}-{}

If the user input is nonsensical or you cannot understand the location information from the user input, mark the output invalid in the valid field in the output schema. 
Since in this case the location information is nonexistent, you can output 0s for all fields: x, y, and theta.
"""