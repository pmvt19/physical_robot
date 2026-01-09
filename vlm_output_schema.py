from pydantic import BaseModel

class UserSemanticTarget(BaseModel):
    valid: bool
    semantic_level: str
    item_name: str
    reason: str

class RoomType(BaseModel):
    room_type: str