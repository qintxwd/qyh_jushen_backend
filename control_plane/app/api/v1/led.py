"""LED灯带控制 API

提供RGBW LED控制接口，包括:
- 纯色设置
- 预设颜色
- 闪烁模式

注意: LED控制为低频操作(用户设置)，适合HTTP接口
"""
from typing import List, Optional
from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field

from app.dependencies import get_current_operator
from app.models.user import User
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes
from app.services.ros2_client import get_ros2_client_dependency, ROS2ServiceClient

router = APIRouter()


# ==================== 数据模型 ====================

class RGBWColor(BaseModel):
    """RGBW颜色模型"""
    r: int = Field(ge=0, le=255, description="红色 (0-255)")
    g: int = Field(ge=0, le=255, description="绿色 (0-255)")
    b: int = Field(ge=0, le=255, description="蓝色 (0-255)")
    w: int = Field(ge=0, le=255, default=0, description="白色 (0-255)")


class SetColorRequest(BaseModel):
    """设置纯色请求"""
    r: int = Field(ge=0, le=255, description="红色 (0-255)")
    g: int = Field(ge=0, le=255, description="绿色 (0-255)")
    b: int = Field(ge=0, le=255, description="蓝色 (0-255)")
    w: int = Field(ge=0, le=255, default=0, description="白色 (0-255)")


class BlinkRequest(BaseModel):
    """闪烁模式请求"""
    colors: List[RGBWColor] = Field(
        ...,
        min_length=1,
        max_length=10,
        description="闪烁颜色序列"
    )
    interval_ms: int = Field(
        ge=50,
        le=10000,
        default=500,
        description="切换间隔(毫秒)"
    )


class LEDState(BaseModel):
    """LED当前状态"""
    is_blinking: bool = Field(
        default=False, description="是否处于闪烁模式"
    )
    current_color: Optional[RGBWColor] = Field(
        default=None, description="当前纯色"
    )
    blink_colors: Optional[List[RGBWColor]] = Field(
        default=None, description="闪烁颜色序列"
    )
    blink_interval_ms: Optional[int] = Field(
        default=None, description="闪烁间隔"
    )


# ==================== 预设颜色 ====================

PRESET_COLORS = {
    "red": RGBWColor(r=255, g=0, b=0, w=0),
    "green": RGBWColor(r=0, g=255, b=0, w=0),
    "blue": RGBWColor(r=0, g=0, b=255, w=0),
    "white": RGBWColor(r=0, g=0, b=0, w=255),          # W通道纯白
    "white_rgb": RGBWColor(r=255, g=255, b=255, w=0),  # RGB混合白
    "yellow": RGBWColor(r=255, g=255, b=0, w=0),
    "cyan": RGBWColor(r=0, g=255, b=255, w=0),
    "magenta": RGBWColor(r=255, g=0, b=255, w=0),
    "orange": RGBWColor(r=255, g=128, b=0, w=0),
    "purple": RGBWColor(r=128, g=0, b=255, w=0),
    "warm_white": RGBWColor(r=50, g=30, b=0, w=200),
    "off": RGBWColor(r=0, g=0, b=0, w=0),
}

# 预设闪烁模式
PRESET_BLINK_MODES = {
    "warning": BlinkRequest(
        colors=[
            RGBWColor(r=255, g=200, b=0, w=0),
            RGBWColor(r=0, g=0, b=0, w=0)
        ],
        interval_ms=200
    ),
    "error": BlinkRequest(
        colors=[
            RGBWColor(r=255, g=0, b=0, w=0),
            RGBWColor(r=0, g=0, b=0, w=0)
        ],
        interval_ms=300
    ),
    "success": BlinkRequest(
        colors=[
            RGBWColor(r=0, g=255, b=0, w=0),
            RGBWColor(r=0, g=0, b=0, w=0)
        ],
        interval_ms=500
    ),
    "processing": BlinkRequest(
        colors=[
            RGBWColor(r=0, g=0, b=255, w=0),
            RGBWColor(r=0, g=255, b=255, w=0)
        ],
        interval_ms=400
    ),
    "rainbow": BlinkRequest(
        colors=[
            RGBWColor(r=255, g=0, b=0, w=0),
            RGBWColor(r=255, g=128, b=0, w=0),
            RGBWColor(r=255, g=255, b=0, w=0),
            RGBWColor(r=0, g=255, b=0, w=0),
            RGBWColor(r=0, g=255, b=255, w=0),
            RGBWColor(r=0, g=0, b=255, w=0),
            RGBWColor(r=128, g=0, b=255, w=0),
        ],
        interval_ms=300
    ),
    "police": BlinkRequest(
        colors=[
            RGBWColor(r=255, g=0, b=0, w=0),
            RGBWColor(r=0, g=0, b=255, w=0)
        ],
        interval_ms=150
    ),
}


# ==================== 状态缓存 ====================

_led_state = LEDState()


# ==================== API 端点 ====================

@router.get("/state", response_model=ApiResponse)
async def get_led_state(
    current_user: User = Depends(get_current_operator),
) -> ApiResponse:
    """获取LED当前状态
    
    返回当前LED状态，包括颜色和闪烁模式
    """
    return success_response(data=_led_state.model_dump())


@router.post("/color", response_model=ApiResponse)
async def set_led_color(
    request: SetColorRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
) -> ApiResponse:
    """设置LED纯色
    
    设置纯色会自动停止闪烁模式
    """
    # 调用ROS2服务设置LED颜色
    result = await ros2.set_led_color(
        r=request.r,
        g=request.g,
        b=request.b,
        w=request.w
    )
    
    if result.success:
        # 更新状态缓存
        _led_state.is_blinking = False
        _led_state.current_color = RGBWColor(
            r=request.r, g=request.g, b=request.b, w=request.w
        )
        _led_state.blink_colors = None
        _led_state.blink_interval_ms = None
        
        return success_response(
            data=_led_state.model_dump(),
            message=(
                f"LED颜色已设置: R={request.r}, G={request.g}, "
                f"B={request.b}, W={request.w}"
            )
        )
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "LED颜色设置失败"
    )


@router.post("/preset/{preset_name}", response_model=ApiResponse)
async def set_led_preset(
    preset_name: str,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
) -> ApiResponse:
    """设置LED预设颜色
    
    可用预设:
    - red, green, blue: 三原色
    - white: 纯白(W通道), white_rgb: RGB混合白
    - yellow, cyan, magenta: 次色
    - orange, purple: 混合色
    - warm_white: 暖白色
    - off: 关闭
    """
    preset_name = preset_name.lower()
    if preset_name not in PRESET_COLORS:
        return error_response(
            code=ErrorCodes.INVALID_PARAMETER,
            message=f"未知预设: {preset_name}"
        )
    
    color = PRESET_COLORS[preset_name]
    
    result = await ros2.set_led_color(
        r=color.r,
        g=color.g,
        b=color.b,
        w=color.w
    )
    
    if result.success:
        _led_state.is_blinking = False
        _led_state.current_color = color
        _led_state.blink_colors = None
        _led_state.blink_interval_ms = None
        
        return success_response(
            data=_led_state.model_dump(),
            message=f"LED已设置为预设颜色: {preset_name}"
        )
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "LED预设颜色设置失败"
    )


@router.post("/blink", response_model=ApiResponse)
async def set_led_blink(
    request: BlinkRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
) -> ApiResponse:
    """设置LED闪烁模式
    
    支持自定义颜色序列和切换间隔
    """
    # 调用ROS2服务设置闪烁模式
    colors_str = ";".join(
        [f"{c.r},{c.g},{c.b},{c.w}" for c in request.colors]
    )
    blink_command = f"{request.interval_ms}:{colors_str}"

    result = await ros2.set_led_blink(blink_command)
    
    if result.success:
        _led_state.is_blinking = True
        _led_state.current_color = request.colors[0]
        _led_state.blink_colors = request.colors
        _led_state.blink_interval_ms = request.interval_ms
        
        return success_response(
            data=_led_state.model_dump(),
            message=(
                f"LED闪烁模式已启动: {len(request.colors)}色, "
                f"{request.interval_ms}ms间隔"
            )
        )
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "LED闪烁模式设置失败"
    )


@router.post("/blink/preset/{preset_name}", response_model=ApiResponse)
async def set_led_blink_preset(
    preset_name: str,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
) -> ApiResponse:
    """设置LED预设闪烁模式
    
    可用预设:
    - warning: 警告(黄色闪烁)
    - error: 错误(红色闪烁)
    - success: 成功(绿色闪烁)
    - processing: 处理中(蓝青交替)
    - rainbow: 彩虹循环
    """
    preset_name = preset_name.lower()
    if preset_name not in PRESET_BLINK_MODES:
        return error_response(
            code=ErrorCodes.INVALID_PARAMETER,
            message=f"未知预设闪烁模式: {preset_name}"
        )
    
    blink_mode = PRESET_BLINK_MODES[preset_name]
    
    colors_str = ";".join(
        [f"{c.r},{c.g},{c.b},{c.w}" for c in blink_mode.colors]
    )
    blink_command = f"{blink_mode.interval_ms}:{colors_str}"

    result = await ros2.set_led_blink(blink_command)
    
    if result.success:
        _led_state.is_blinking = True
        _led_state.current_color = blink_mode.colors[0]
        _led_state.blink_colors = blink_mode.colors
        _led_state.blink_interval_ms = blink_mode.interval_ms
        
        return success_response(
            data=_led_state.model_dump(),
            message=f"LED已设置为预设闪烁模式: {preset_name}"
        )
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "LED预设闪烁模式设置失败"
    )


@router.post("/stop", response_model=ApiResponse)
async def stop_led_blink(
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
) -> ApiResponse:
    """停止LED闪烁，恢复为纯色或关闭"""
    # 停止闪烁，设置为关闭
    result = await ros2.set_led_blink("stop")
    
    if result.success:
        _led_state.is_blinking = False
        _led_state.blink_colors = None
        _led_state.blink_interval_ms = None
        
        return success_response(
            data=_led_state.model_dump(),
            message="LED闪烁已停止"
        )
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "LED闪烁停止失败"
    )


@router.get("/presets", response_model=ApiResponse)
async def get_led_presets(
    current_user: User = Depends(get_current_operator),
) -> ApiResponse:
    """获取所有预设颜色与闪烁模式"""
    return success_response(
        data={
            "colors": {
                name: color.model_dump() for name, color in PRESET_COLORS.items()
            },
            "blink_modes": {
                name: {
                    "colors": [c.model_dump() for c in mode.colors],
                    "interval_ms": mode.interval_ms
                }
                for name, mode in PRESET_BLINK_MODES.items()
            }
        }
    )
