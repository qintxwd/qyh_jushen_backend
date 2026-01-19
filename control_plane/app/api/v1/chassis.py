"""
QYH Jushen Control Plane - 底盘配置 API

提供底盘配置的持久化管理（速度级别、音量、避障策略等）
"""
import json
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field

from app.dependencies import get_current_operator, get_current_user
from app.models.user import User
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes

router = APIRouter()


# ==================== 配置文件管理 ====================

def _get_chassis_config_file() -> Path:
    """获取底盘配置文件路径"""
    # 配置文件位于 workspace_root/persistent/web/chassis_config.json
    import os
    workspace_root = Path(os.environ.get('QYH_WORKSPACE_ROOT', Path.home() / 'qyh-robot-system'))
    config_dir = workspace_root / "persistent" / "web"
    config_dir.mkdir(parents=True, exist_ok=True)
    return config_dir / "chassis_config.json"


def _load_chassis_config() -> dict:
    """加载底盘配置"""
    config_file = _get_chassis_config_file()
    if config_file.exists():
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            pass
    
    # 默认配置
    return {
        "speed_level": 50,
        "volume": 50,
        "obstacle_strategy": 1,
    }


def _save_chassis_config(config: dict):
    """保存底盘配置"""
    config_file = _get_chassis_config_file()
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)


# ==================== 数据模型 ====================

class ChassisConfig(BaseModel):
    """底盘配置"""
    speed_level: int = Field(
        default=50,
        ge=1,
        le=100,
        description="速度级别 (1-100)"
    )
    volume: int = Field(
        default=50,
        ge=0,
        le=100,
        description="音量 (0-100)"
    )
    obstacle_strategy: int = Field(
        default=1,
        ge=0,
        le=2,
        description="避障策略: 0=禁用, 1=正常, 2=激进"
    )


class ChassisConfigUpdate(BaseModel):
    """底盘配置更新（允许部分更新）"""
    speed_level: Optional[int] = Field(
        default=None,
        ge=1,
        le=100,
        description="速度级别 (1-100)"
    )
    volume: Optional[int] = Field(
        default=None,
        ge=0,
        le=100,
        description="音量 (0-100)"
    )
    obstacle_strategy: Optional[int] = Field(
        default=None,
        ge=0,
        le=2,
        description="避障策略: 0=禁用, 1=正常, 2=激进"
    )


# ==================== API 端点 ====================

@router.get("/config", response_model=ApiResponse)
async def get_chassis_config(
    current_user: User = Depends(get_current_user),
):
    """
    获取底盘配置
    
    返回当前的速度级别、音量、避障策略等配置
    """
    config_dict = _load_chassis_config()
    config = ChassisConfig(**config_dict)
    
    return success_response(
        data=config.model_dump(),
        message="获取底盘配置成功"
    )


@router.put("/config", response_model=ApiResponse)
async def update_chassis_config(
    update: ChassisConfigUpdate,
    current_user: User = Depends(get_current_operator),
):
    """
    更新底盘配置
    
    支持部分更新，只需提供要修改的字段。
    需要操作员权限。
    """
    # 加载现有配置
    config_dict = _load_chassis_config()
    
    # 更新字段
    if update.speed_level is not None:
        config_dict["speed_level"] = update.speed_level
    if update.volume is not None:
        config_dict["volume"] = update.volume
    if update.obstacle_strategy is not None:
        config_dict["obstacle_strategy"] = update.obstacle_strategy
    
    # 保存配置
    try:
        _save_chassis_config(config_dict)
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"保存配置失败: {str(e)}"
        )
    
    # TODO: 通知 Data Plane 配置已更新
    # 可以通过 ROS2 参数服务器或专用话题通知
    
    config = ChassisConfig(**config_dict)
    return success_response(
        data=config.model_dump(),
        message="底盘配置已更新"
    )


@router.post("/config/reset", response_model=ApiResponse)
async def reset_chassis_config(
    current_user: User = Depends(get_current_operator),
):
    """
    重置底盘配置为默认值
    
    需要操作员权限。
    """
    default_config = {
        "speed_level": 50,
        "volume": 50,
        "obstacle_strategy": 1,
    }
    
    try:
        _save_chassis_config(default_config)
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"重置配置失败: {str(e)}"
        )
    
    config = ChassisConfig(**default_config)
    return success_response(
        data=config.model_dump(),
        message="底盘配置已重置为默认值"
    )
