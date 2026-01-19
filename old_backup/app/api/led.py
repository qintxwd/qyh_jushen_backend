"""LED灯带控制 API"""
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, List

from app.dependencies import get_current_operator
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


# ==================== 数据模型 ====================

class RGBWColor(BaseModel):
    """RGBW颜色"""
    r: int = Field(ge=0, le=255, description="红色 (0-255)")
    g: int = Field(ge=0, le=255, description="绿色 (0-255)")
    b: int = Field(ge=0, le=255, description="蓝色 (0-255)")
    w: int = Field(ge=0, le=255, default=0, description="白色 (0-255), W通道用于产生更纯净的白光")


class SetColorRequest(BaseModel):
    """设置纯色请求"""
    r: int = Field(ge=0, le=255, description="红色 (0-255)")
    g: int = Field(ge=0, le=255, description="绿色 (0-255)")
    b: int = Field(ge=0, le=255, description="蓝色 (0-255)")
    w: int = Field(ge=0, le=255, default=0, description="白色 (0-255)")


class BlinkRequest(BaseModel):
    """闪烁请求"""
    colors: List[RGBWColor] = Field(..., min_length=1, description="闪烁颜色序列")
    interval_ms: int = Field(ge=50, le=10000, default=500, description="切换间隔(毫秒), 最小50ms")


class PresetColorRequest(BaseModel):
    """预设颜色请求"""
    preset: str = Field(..., description="预设名称: red, green, blue, white, yellow, cyan, magenta, orange, purple, off")


class LEDState(BaseModel):
    """LED状态"""
    is_blinking: bool = False
    current_color: Optional[RGBWColor] = None
    blink_colors: Optional[List[RGBWColor]] = None
    blink_interval_ms: Optional[int] = None


# ==================== 预设颜色 ====================

PRESET_COLORS = {
    "red": RGBWColor(r=255, g=0, b=0, w=0),
    "green": RGBWColor(r=0, g=255, b=0, w=0),
    "blue": RGBWColor(r=0, g=0, b=255, w=0),
    "white": RGBWColor(r=0, g=0, b=0, w=255),      # 纯白色LED
    "white_rgb": RGBWColor(r=255, g=255, b=255, w=0),  # RGB混合白色
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
    "warning": {
        "colors": [RGBWColor(r=255, g=200, b=0, w=0), RGBWColor(r=0, g=0, b=0, w=0)],
        "interval_ms": 200
    },
    "error": {
        "colors": [RGBWColor(r=255, g=0, b=0, w=0), RGBWColor(r=0, g=0, b=0, w=0)],
        "interval_ms": 300
    },
    "success": {
        "colors": [RGBWColor(r=0, g=255, b=0, w=0), RGBWColor(r=0, g=0, b=0, w=0)],
        "interval_ms": 500
    },
    "processing": {
        "colors": [RGBWColor(r=0, g=0, b=255, w=0), RGBWColor(r=0, g=255, b=255, w=0)],
        "interval_ms": 400
    },
    "rainbow": {
        "colors": [
            RGBWColor(r=255, g=0, b=0, w=0),
            RGBWColor(r=255, g=128, b=0, w=0),
            RGBWColor(r=255, g=255, b=0, w=0),
            RGBWColor(r=0, g=255, b=0, w=0),
            RGBWColor(r=0, g=255, b=255, w=0),
            RGBWColor(r=0, g=0, b=255, w=0),
            RGBWColor(r=128, g=0, b=255, w=0),
        ],
        "interval_ms": 300
    },
    "police": {
        "colors": [RGBWColor(r=255, g=0, b=0, w=0), RGBWColor(r=0, g=0, b=255, w=0)],
        "interval_ms": 150
    }
}

# 当前LED状态缓存
_led_state = LEDState()


# ==================== API 端点 ====================

@router.get("/led/state", response_model=ApiResponse)
async def get_led_state(current_user=Depends(get_current_operator)):
    """获取LED当前状态"""
    watchdog.heartbeat()
    return success_response(data=_led_state.model_dump())


@router.post("/led/color", response_model=ApiResponse)
async def set_led_color(request: SetColorRequest, current_user=Depends(get_current_operator)):
    """
    设置LED纯色
    
    设置纯色会自动停止闪烁模式
    
    - **r**: 红色 (0-255)
    - **g**: 绿色 (0-255)
    - **b**: 蓝色 (0-255)
    - **w**: 白色 (0-255), W通道用于产生更纯净的白光
    """
    watchdog.heartbeat()
    
    try:
        # 发送到 ROS2
        success = ros2_bridge.set_led_color(request.r, request.g, request.b, request.w)
        
        if success:
            # 更新状态缓存
            _led_state.is_blinking = False
            _led_state.current_color = RGBWColor(r=request.r, g=request.g, b=request.b, w=request.w)
            _led_state.blink_colors = None
            _led_state.blink_interval_ms = None
            
            return success_response(
                message=f"LED颜色已设置: R={request.r}, G={request.g}, B={request.b}, W={request.w}",
                data=_led_state.model_dump()
            )
        else:
            return error_response(
                code=ErrorCodes.ROS_SERVICE_FAILED,
                message="LED颜色设置失败，ROS2通信错误"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"设置LED颜色失败: {str(e)}"
        )


@router.post("/led/preset/{preset_name}", response_model=ApiResponse)
async def set_led_preset(preset_name: str, current_user=Depends(get_current_operator)):
    """
    设置LED预设颜色
    
    可用预设:
    - **red**: 红色
    - **green**: 绿色
    - **blue**: 蓝色
    - **white**: 纯白色 (W通道)
    - **white_rgb**: RGB混合白色
    - **yellow**: 黄色
    - **cyan**: 青色
    - **magenta**: 品红
    - **orange**: 橙色
    - **purple**: 紫色
    - **warm_white**: 暖白色
    - **off**: 关闭
    """
    watchdog.heartbeat()
    
    preset_name = preset_name.lower()
    if preset_name not in PRESET_COLORS:
        return error_response(
            code=ErrorCodes.INVALID_PARAMETER,
            message=f"未知预设: {preset_name}，可用预设: {', '.join(PRESET_COLORS.keys())}"
        )
    
    color = PRESET_COLORS[preset_name]
    
    try:
        success = ros2_bridge.set_led_color(color.r, color.g, color.b, color.w)
        
        if success:
            _led_state.is_blinking = False
            _led_state.current_color = color
            _led_state.blink_colors = None
            _led_state.blink_interval_ms = None
            
            return success_response(
                message=f"LED已设置为预设颜色: {preset_name}",
                data=_led_state.model_dump()
            )
        else:
            return error_response(
                code=ErrorCodes.ROS_SERVICE_FAILED,
                message="LED颜色设置失败"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"设置预设颜色失败: {str(e)}"
        )


@router.post("/led/blink", response_model=ApiResponse)
async def set_led_blink(request: BlinkRequest, current_user=Depends(get_current_operator)):
    """
    设置LED闪烁模式
    
    支持多颜色循环闪烁，颜色按顺序循环显示
    
    - **colors**: 颜色序列，至少1个颜色
    - **interval_ms**: 切换间隔(毫秒)，最小50ms，最大10000ms
    
    示例:
    - 红绿交替: colors=[{r:255,g:0,b:0,w:0}, {r:0,g:255,b:0,w:0}], interval_ms=500
    - RGB循环: colors=[{r:255,...}, {g:255,...}, {b:255,...}], interval_ms=300
    """
    watchdog.heartbeat()
    
    try:
        # 构建闪烁命令字符串
        colors_str = ";".join([f"{c.r},{c.g},{c.b},{c.w}" for c in request.colors])
        blink_command = f"{request.interval_ms}:{colors_str}"
        
        success = ros2_bridge.set_led_blink(blink_command)
        
        if success:
            _led_state.is_blinking = True
            _led_state.current_color = request.colors[0]
            _led_state.blink_colors = request.colors
            _led_state.blink_interval_ms = request.interval_ms
            
            return success_response(
                message=f"LED闪烁已启动: {len(request.colors)}种颜色, {request.interval_ms}ms间隔",
                data=_led_state.model_dump()
            )
        else:
            return error_response(
                code=ErrorCodes.ROS_SERVICE_FAILED,
                message="LED闪烁设置失败"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"设置LED闪烁失败: {str(e)}"
        )


@router.post("/led/blink/preset/{mode_name}", response_model=ApiResponse)
async def set_led_blink_preset(mode_name: str, current_user=Depends(get_current_operator)):
    """
    设置LED预设闪烁模式
    
    可用模式:
    - **warning**: 黄色警告闪烁 (200ms)
    - **error**: 红色错误闪烁 (300ms)
    - **success**: 绿色成功闪烁 (500ms)
    - **processing**: 蓝色处理中闪烁 (400ms)
    - **rainbow**: 彩虹循环 (300ms)
    - **police**: 红蓝警灯 (150ms)
    """
    watchdog.heartbeat()
    
    mode_name = mode_name.lower()
    if mode_name not in PRESET_BLINK_MODES:
        return error_response(
            code=ErrorCodes.INVALID_PARAMETER,
            message=f"未知闪烁模式: {mode_name}，可用模式: {', '.join(PRESET_BLINK_MODES.keys())}"
        )
    
    mode = PRESET_BLINK_MODES[mode_name]
    
    try:
        colors_str = ";".join([f"{c.r},{c.g},{c.b},{c.w}" for c in mode["colors"]])
        blink_command = f"{mode['interval_ms']}:{colors_str}"
        
        success = ros2_bridge.set_led_blink(blink_command)
        
        if success:
            _led_state.is_blinking = True
            _led_state.current_color = mode["colors"][0]
            _led_state.blink_colors = mode["colors"]
            _led_state.blink_interval_ms = mode["interval_ms"]
            
            return success_response(
                message=f"LED已设置为预设闪烁模式: {mode_name}",
                data=_led_state.model_dump()
            )
        else:
            return error_response(
                code=ErrorCodes.ROS_SERVICE_FAILED,
                message="LED闪烁设置失败"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"设置预设闪烁模式失败: {str(e)}"
        )


@router.post("/led/stop", response_model=ApiResponse)
async def stop_led_blink(current_user=Depends(get_current_operator)):
    """
    停止LED闪烁并恢复默认颜色
    """
    watchdog.heartbeat()
    
    try:
        success = ros2_bridge.set_led_blink("stop")
        
        if success:
            _led_state.is_blinking = False
            _led_state.blink_colors = None
            _led_state.blink_interval_ms = None
            # current_color 会被节点设置为默认颜色
            
            return success_response(
                message="LED闪烁已停止，恢复默认颜色",
                data=_led_state.model_dump()
            )
        else:
            return error_response(
                code=ErrorCodes.ROS_SERVICE_FAILED,
                message="停止LED闪烁失败"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"停止LED闪烁失败: {str(e)}"
        )


@router.get("/led/presets", response_model=ApiResponse)
async def get_led_presets(current_user=Depends(get_current_operator)):
    """
    获取所有可用的预设颜色和闪烁模式
    """
    watchdog.heartbeat()
    
    return success_response(data={
        "colors": {name: color.model_dump() for name, color in PRESET_COLORS.items()},
        "blink_modes": {
            name: {
                "colors": [c.model_dump() for c in mode["colors"]],
                "interval_ms": mode["interval_ms"]
            }
            for name, mode in PRESET_BLINK_MODES.items()
        }
    })
