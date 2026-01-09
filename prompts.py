ASSIGN_ROOM_LABEL_ONLY_PROMPT = \
f"""
You are an image analysis expert.

Identify what kind of room the inputed image depicts.

Please answer with only the room (can be one or multiple words).
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

If the user input is nonsensical or you cannot understand the semantic information from the user input, mark the output invalid for the valid field. 
Since in this case the semantic information is nonexistent, you can output "N/A" for both the semantic_level and item_name fields.
"""