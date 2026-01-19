"""数据录制模型"""
from sqlalchemy import Column, Integer, String, BigInteger, Float, DateTime, ForeignKey, JSON, Enum
from datetime import datetime
from app.database import Base
import enum


class RecordingFormat(str, enum.Enum):
    """录制格式"""
    ROSBAG = "rosbag"
    CSV = "csv"


class Recording(Base):
    """数据录制表"""
    __tablename__ = "recordings"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(200), nullable=False)
    format = Column(Enum(RecordingFormat), default=RecordingFormat.ROSBAG)
    file_path = Column(String(500), nullable=False)
    file_size = Column(BigInteger, nullable=True)  # 字节
    duration = Column(Float, nullable=True)  # 秒
    topics = Column(JSON, nullable=False)  # 录制的话题列表
    creator_id = Column(Integer, ForeignKey("users.id"))
    created_at = Column(DateTime, default=datetime.utcnow)
